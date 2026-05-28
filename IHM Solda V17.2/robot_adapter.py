# robot_adapter.py — wrapper V17.2 com correção de sincronismo TCP/IHM.
#
# Este arquivo carrega o adapter histórico da pasta de solda e aplica patches
# pequenos para garantir que, em robô real, o TCP lido pela SDK seja publicado
# continuamente para a página. Também corrige uma falha crítica: telemetria/diagnóstico
# não pode sobrescrever o TCP usado para salvar pontos, porque algumas fontes retornam
# orientação em graus enquanto a SDK de movimento usa radianos.

from pathlib import Path
import importlib.util
import sys
import time
import math
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

_TWO_PI = math.pi * 2.0
_DEG_THRESHOLD = _TWO_PI + 0.25


def _is_number(v: Any) -> bool:
    try:
        float(v)
        return True
    except Exception:
        return False


def _normalizar_pose_unidade(pose: List[float], origem: str = "sdk") -> List[float]:
    """Normaliza pose para o contrato interno: XYZ em mm e RX/RY/RZ em radianos.

    O bug crítico observado é típico de mistura rad/grau: 0.496 rad ~= 28.4°.
    Se algum retorno trouxer rotação com magnitude maior que 2π, tratamos como grau
    e convertemos para radiano antes de publicar/salvar/executar.
    """
    out = [float(x) for x in pose[:6]]
    rot = out[3:6]
    if any(abs(v) > _DEG_THRESHOLD for v in rot):
        out[3:6] = [math.radians(v) for v in rot]
        try:
            print(f"[TCP] Orientação de {origem} parecia em graus; convertida para radianos: {rot} -> {out[3:6]}")
        except Exception:
            pass
    return out


def _coerce_pose(value: Any, depth: int = 0, origem: str = "sdk") -> Optional[List[float]]:
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
                pose = _coerce_pose(value[key], depth + 1, origem)
                if pose is not None:
                    return pose
        for item in value.values():
            pose = _coerce_pose(item, depth + 1, origem)
            if pose is not None:
                return pose
        return None

    if isinstance(value, (list, tuple)):
        if len(value) >= 6 and all(_is_number(x) for x in value[:6]):
            return _normalizar_pose_unidade([float(x) for x in value[:6]], origem)

        # Formato típico: [0, [x,y,z,rx,ry,rz]] ou (0, pose)
        if len(value) >= 2 and _is_number(value[0]):
            pose = _coerce_pose(value[1], depth + 1, origem)
            if pose is not None:
                return pose

        for item in value:
            pose = _coerce_pose(item, depth + 1, origem)
            if pose is not None:
                return pose

    # Alguns SDKs retornam objeto com atributos.
    for attr in ("tcp", "pose", "tcp_position", "cartesian", "cartesian_pose"):
        if hasattr(value, attr):
            pose = _coerce_pose(getattr(value, attr), depth + 1, origem)
            if pose is not None:
                return pose

    return None


def _rot_delta_abs(a: List[float], b: List[float]) -> float:
    return max(abs(float(a[i]) - float(b[i])) for i in (3, 4, 5))


def _xyz_delta_abs(a: List[float], b: List[float]) -> float:
    return max(abs(float(a[i]) - float(b[i])) for i in (0, 1, 2))


def _pose_parece_salto_falso(self, nova: List[float]) -> bool:
    """Rejeita salto brusco de rotação quando não há comando rotacional ativo.

    Não é filtro de segurança certificado; é só uma trava contra pacote/parse falso.
    A fonte oficial segue sendo a SDK get_tcp_position().
    """
    atual = self._copy_tcp()
    if not atual or len(atual) < 6:
        return False

    # Durante execução de trajetória não filtramos aqui: os pontos já foram validados.
    if getattr(self, "executando_trajetoria", False):
        return False

    grupo = getattr(self, "grupo_ativo", None)
    comando_rotacional = grupo in ("ROTAT_TCP", "EIXO_RZ")
    if comando_rotacional:
        return False

    # Se XYZ quase não mudou mas rotação saltou > 20°, é suspeito para robô parado/jog linear.
    if _xyz_delta_abs(atual, nova) < 2.0 and _rot_delta_abs(atual, nova) > math.radians(20.0):
        return True
    return False


def _refresh_tcp_from_robot(self, origem: str = "sdk", aplicar_filtro: bool = True) -> bool:
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
        pose = _coerce_pose(ret, origem=origem)
        if pose is None:
            with self._state_lock:
                self.diagnosticos["tcp_status"] = "get_tcp_position sem pose reconhecida"
                self.diagnosticos["tcp_raw_preview"] = repr(ret)[:240]
            return False

        if aplicar_filtro and _pose_parece_salto_falso(self, pose):
            with self._state_lock:
                self.diagnosticos["tcp_status"] = "TCP descartado: salto falso de rotação"
                self.diagnosticos["tcp_origem_descartada"] = origem
                self.diagnosticos["tcp_descartado"] = pose
                self.diagnosticos["ultima_leitura_tcp_descartada_ts"] = time.time()
            print(f"[TCP] Descartado salto falso de rotação ({origem}): {pose}")
            return False

        self._set_tcp(pose)
        with self._state_lock:
            self.diagnosticos["tcp_status"] = "OK"
            self.diagnosticos["tcp_origem"] = origem
            self.diagnosticos["tcp_unidade_rotacao"] = "rad"
            self.diagnosticos["ultima_leitura_tcp_ts"] = time.time()
        return True
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["tcp_status"] = f"Falha get_tcp_position: {e}"
        return False


def _amostrar_tcp_estavel_para_ponto(self) -> bool:
    """Antes de salvar ponto, faz leituras curtas e usa mediana por componente.

    Isso evita marcar ponto exatamente no meio de uma amostra ruim/atrasada.
    """
    if getattr(self, "modo_simulacao", True):
        return False
    amostras = []
    for _ in range(5):
        try:
            ret = self.driver.get_tcp_position()
            pose = _coerce_pose(ret, origem="amostra_ponto")
            if pose is not None:
                amostras.append(pose)
        except Exception:
            pass
        time.sleep(0.025)

    if not amostras:
        return False

    def mediana(vals):
        vals = sorted(vals)
        return vals[len(vals) // 2]

    pose_mediana = [mediana([p[i] for p in amostras]) for i in range(6)]
    self._set_tcp(pose_mediana)
    with self._state_lock:
        self.diagnosticos["tcp_status"] = "OK - ponto por mediana"
        self.diagnosticos["tcp_origem"] = "amostra_ponto"
        self.diagnosticos["tcp_unidade_rotacao"] = "rad"
        self.diagnosticos["ultima_leitura_tcp_ts"] = time.time()
    return True


_original_conectar = adapter.conectar
_original_salvar_ponto_atual = adapter.salvar_ponto_atual
_original_update_telemetry_low_freq = adapter._update_telemetry_low_freq
_original_apply_telemetry_updates = adapter._apply_telemetry_updates


def _patched_apply_telemetry_updates(self, updates, debug):
    """Telemetria não pode escrever TCP de movimento.

    Mantemos correntes/temperaturas/status, mas descartamos qualquer campo 'tcp'.
    Motivo: fontes de telemetria podem retornar orientação em graus; movimento JAKA
    usa radianos. Misturar isso salva ponto falso e pode fazer a tocha virar na mesa.
    """
    try:
        if isinstance(updates, dict) and "tcp" in updates:
            tcp_descartado = updates.pop("tcp")
            with self._state_lock:
                self.diagnosticos["tcp_telemetria_descartado"] = True
                self.diagnosticos["tcp_telemetria_raw_preview"] = repr(tcp_descartado)[:240]
                self.diagnosticos["tcp_fonte_oficial"] = "sdk_get_tcp_position"
    except Exception:
        pass
    return _original_apply_telemetry_updates(updates, debug)


def _patched_conectar(self, ip="192.168.0.200"):
    ok = _original_conectar(ip)
    if ok and not getattr(self, "modo_simulacao", True):
        # Primeira leitura imediata para a tela não abrir congelada em zero.
        for _ in range(5):
            if self._refresh_tcp_from_robot("connect", aplicar_filtro=False):
                break
            time.sleep(0.08)
        if self.on_state_update:
            self.on_state_update(self.snapshot_state())
    return ok


def _patched_salvar_ponto_atual(self, tipo):
    # Antes de gravar ponto, amostra TCP real por SDK e usa mediana.
    # Não usamos telemetria aqui, jamais.
    self._amostrar_tcp_estavel_para_ponto()
    return _original_salvar_ponto_atual(tipo)


def _patched_update_telemetry_low_freq(self):
    # Mantém diagnósticos antigos, mas o TCP oficial vem antes/depois pela SDK.
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

                # Em robô real, busca TCP por SDK continuamente.
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
adapter._amostrar_tcp_estavel_para_ponto = MethodType(_amostrar_tcp_estavel_para_ponto, adapter)
adapter._apply_telemetry_updates = MethodType(_patched_apply_telemetry_updates, adapter)
adapter.conectar = MethodType(_patched_conectar, adapter)
adapter.salvar_ponto_atual = MethodType(_patched_salvar_ponto_atual, adapter)
adapter._update_telemetry_low_freq = MethodType(_patched_update_telemetry_low_freq, adapter)
adapter.iniciar_loop_controle = MethodType(_patched_iniciar_loop_controle, adapter)
