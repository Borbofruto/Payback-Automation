# robot_adapter.py — wrapper V17.2 com correção de sincronismo TCP/IHM.
#
# Este arquivo carrega o adapter histórico da pasta de solda e aplica patches
# pequenos para garantir que, em robô real, o TCP lido pela SDK seja publicado
# continuamente para a página via Socket.IO.

from pathlib import Path
import importlib.util
import sys
import time
import threading
from types import MethodType
from typing import Any, Optional, List

_REAL_DIR = Path(__file__).resolve().parents[1] / "Programas de Robôs" / "JAKA" / "Solda"
_REAL_FILE = _REAL_DIR / "robot_adapter.py"

# Garante que os imports do adapter real resolvam para os módulos reais
# daquele diretório, não para os proxies desta pasta.
if str(_REAL_DIR) not in sys.path:
    sys.path.insert(0, str(_REAL_DIR))

_SPEC = importlib.util.spec_from_file_location("_payback_real_robot_adapter_v17_2", _REAL_FILE)
_REAL_MOD = importlib.util.module_from_spec(_SPEC)
sys.modules["_payback_real_robot_adapter_v17_2"] = _REAL_MOD
_SPEC.loader.exec_module(_REAL_MOD)

RobotAdapter = _REAL_MOD.RobotAdapter
adapter = _REAL_MOD.adapter


def _is_number(v: Any) -> bool:
    try:
        float(v)
        return True
    except Exception:
        return False


def _coerce_pose(value: Any, depth: int = 0) -> Optional[List[float]]:
    """Aceita variações comuns de retorno do SDK JAKA e extrai [x,y,z,rx,ry,rz].

    A versão original só aceitava exatamente [0, [pose]]. Se o SDK devolver direto
    [pose], tupla, dict ou payload aninhado, a IHM ficava com TCP velho.
    """
    if value is None or depth > 5:
        return None

    if isinstance(value, dict):
        keys = (
            "tcp", "pose", "tcp_position", "tool_pos", "tool_position",
            "cartesian", "cartesian_pose", "cartesiantran_position",
            "actual_tcp_pose", "tcp_pos", "pos",
        )
        for key in keys:
            if key in value:
                pose = _coerce_pose(value[key], depth + 1)
                if pose is not None:
                    return pose
        for item in value.values():
            pose = _coerce_pose(item, depth + 1)
            if pose is not None:
                return pose
        return None

    if isinstance(value, (list, tuple)):
        if len(value) >= 6 and all(_is_number(x) for x in value[:6]):
            return [float(x) for x in value[:6]]

        # Formato típico: [0, [x,y,z,rx,ry,rz]] ou (0, pose)
        if len(value) >= 2 and _is_number(value[0]):
            pose = _coerce_pose(value[1], depth + 1)
            if pose is not None:
                return pose

        for item in value:
            pose = _coerce_pose(item, depth + 1)
            if pose is not None:
                return pose

    # Alguns SDKs retornam objeto com atributos.
    for attr in ("tcp", "pose", "tcp_position", "cartesian", "cartesian_pose"):
        if hasattr(value, attr):
            pose = _coerce_pose(getattr(value, attr), depth + 1)
            if pose is not None:
                return pose

    return None


def _refresh_tcp_from_robot(self, origem: str = "sdk") -> bool:
    """Lê TCP real do robô e atualiza o estado publicado para a IHM."""
    if getattr(self, "modo_simulacao", True):
        return False

    try:
        has_robot = self.driver.has_robot()
    except Exception:
        has_robot = False
    if not has_robot:
        return False

    try:
        ret = self.driver.get_tcp_position()
        pose = _coerce_pose(ret)
        if pose is None:
            with self._state_lock:
                self.diagnosticos["tcp_status"] = "get_tcp_position sem pose reconhecida"
                self.diagnosticos["tcp_raw_preview"] = repr(ret)[:240]
            return False

        self._set_tcp(pose)
        with self._state_lock:
            self.diagnosticos["tcp_status"] = "OK"
            self.diagnosticos["tcp_origem"] = origem
            self.diagnosticos["ultima_leitura_tcp_ts"] = time.time()
        return True
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["tcp_status"] = f"Falha get_tcp_position: {e}"
        return False


_original_conectar = adapter.conectar
_original_salvar_ponto_atual = adapter.salvar_ponto_atual
_original_update_telemetry_low_freq = adapter._update_telemetry_low_freq


def _patched_conectar(self, ip="192.168.0.200"):
    ok = _original_conectar(ip)
    if ok and not getattr(self, "modo_simulacao", True):
        # Primeira leitura imediata para a tela não abrir congelada em zero.
        for _ in range(5):
            if self._refresh_tcp_from_robot("connect"):
                break
            time.sleep(0.08)
        if self.on_state_update:
            self.on_state_update(self.snapshot_state())
    return ok


def _patched_salvar_ponto_atual(self, tipo):
    # Antes de gravar ponto, força leitura real. Sem isso, ponto físico podia sair
    # com pose antiga se o TCP10000/telemetria não estivesse alimentando o estado.
    self._refresh_tcp_from_robot("antes_salvar_ponto")
    return _original_salvar_ponto_atual(tipo)


def _patched_update_telemetry_low_freq(self):
    # Mantém a telemetria antiga, mas não depende dela para atualizar TCP.
    self._refresh_tcp_from_robot("low_freq")
    return _original_update_telemetry_low_freq()


def _patched_iniciar_loop_controle(self):
    pygame = _REAL_MOD.pygame
    pygame.init()
    pygame.joystick.init()
    try:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"[JOYSTICK] {joy.get_name()} inicializado.")
    except Exception:
        joy = None
        print("[JOYSTICK] Nenhum controle encontrado.")

    def loop():
        nonlocal joy
        last_tcp_poll = 0.0
        while True:
            try:
                if joy is None and pygame.joystick.get_count() > 0:
                    try:
                        joy = pygame.joystick.Joystick(0)
                        joy.init()
                        print(f"[JOYSTICK] {joy.get_name()} reconectado.")
                    except Exception:
                        joy = None

                self._contador_telemetria += 1
                if self._contador_telemetria >= 15:
                    self._contador_telemetria = 0
                    self._update_telemetry_low_freq()

                if joy:
                    try:
                        if not joy.get_init():
                            joy.init()
                        pygame.event.pump()
                        self._processar_joystick(joy)
                    except (pygame.error, AttributeError) as e:
                        print(f"[JOYSTICK] Falha/desconectado: {e}")
                        self.parar_grupo([0, 1, 2, 3, 4, 5])
                        self.grupo_ativo = None
                        joy = None

                # Ponto central da correção:
                # Em robô real, busca TCP por SDK continuamente, não só pela telemetria.
                now = time.time()
                if now - last_tcp_poll >= 0.08:
                    self._refresh_tcp_from_robot("loop")
                    last_tcp_poll = now

                if self.on_state_update:
                    self.on_state_update(self.snapshot_state())
            except Exception as e:
                print(f"[LOOP] Erro inesperado: {e}")
            time.sleep(0.03)

    threading.Thread(target=loop, daemon=True).start()


# Aplica patches na instância usada pelo app.py.
adapter._refresh_tcp_from_robot = MethodType(_refresh_tcp_from_robot, adapter)
adapter.conectar = MethodType(_patched_conectar, adapter)
adapter.salvar_ponto_atual = MethodType(_patched_salvar_ponto_atual, adapter)
adapter._update_telemetry_low_freq = MethodType(_patched_update_telemetry_low_freq, adapter)
adapter.iniciar_loop_controle = MethodType(_patched_iniciar_loop_controle, adapter)
