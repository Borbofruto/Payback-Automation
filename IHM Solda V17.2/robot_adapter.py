# robot_adapter.py — wrapper V17.2 com correções de TCP, unidade e workspace.
#
# Este arquivo carrega o adapter histórico da pasta de solda e aplica patches
# pequenos para garantir que, em robô real, o TCP lido pela SDK seja publicado
# continuamente para a página. Também corrige uma falha crítica: telemetria/diagnóstico
# não pode sobrescrever o TCP usado para salvar pontos, porque algumas fontes retornam
# orientação em graus enquanto a SDK de movimento usa radianos.
#
# Workspace: dois pontos opostos da mesa definem o retângulo XY e a superfície Z.
# Se o TCP estiver fora do workspace, TODO jog por controle é bloqueado. O retorno
# deve ser feito por Drag Mode / Free Drive, como decisão operacional explícita.

from pathlib import Path
import importlib.util
import sys
import time
import math
import copy
import threading
from types import MethodType
from typing import Any, Optional, List, Dict

_REAL_DIR = Path(__file__).resolve().parents[1] / "Programas de Robôs" / "JAKA" / "Solda"
_REAL_FILE = _REAL_DIR / "robot_adapter.py"

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

        if len(value) >= 2 and _is_number(value[0]):
            pose = _coerce_pose(value[1], depth + 1, origem)
            if pose is not None:
                return pose

        for item in value:
            pose = _coerce_pose(item, depth + 1, origem)
            if pose is not None:
                return pose

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
    atual = self._copy_tcp()
    if not atual or len(atual) < 6:
        return False
    if getattr(self, "executando_trajetoria", False):
        return False
    grupo = getattr(self, "grupo_ativo", None)
    if grupo in ("ROTAT_TCP", "EIXO_RZ"):
        return False
    return _xyz_delta_abs(atual, nova) < 2.0 and _rot_delta_abs(atual, nova) > math.radians(20.0)


# ---------------------------------------------------------------------------
# Workspace / mesa
# ---------------------------------------------------------------------------

def _workspace_default() -> Dict[str, Any]:
    return {
        "enabled": False,
        "p1": None,
        "p2": None,
        "z_margin_mm": 10.0,
        "xy_margin_mm": 5.0,
        "slow_zone_mm": 30.0,
    }


def _pose3_from_any(v) -> Optional[List[float]]:
    if v is None:
        return None
    if isinstance(v, dict):
        try:
            return [float(v.get("x")), float(v.get("y")), float(v.get("z"))]
        except Exception:
            return None
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        try:
            return [float(v[0]), float(v[1]), float(v[2])]
        except Exception:
            return None
    return None


def _workspace_limits_from_cfg(cfg: Dict[str, Any]) -> Optional[Dict[str, float]]:
    p1 = _pose3_from_any(cfg.get("p1"))
    p2 = _pose3_from_any(cfg.get("p2"))
    if p1 is None or p2 is None:
        return None

    x_min = min(p1[0], p2[0])
    x_max = max(p1[0], p2[0])
    y_min = min(p1[1], p2[1])
    y_max = max(p1[1], p2[1])
    z_surface = (p1[2] + p2[2]) / 2.0
    z_margin = float(cfg.get("z_margin_mm", 10.0))
    xy_margin = float(cfg.get("xy_margin_mm", 5.0))
    slow_zone = max(1.0, float(cfg.get("slow_zone_mm", 30.0)))

    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "z_surface": z_surface,
        "z_min_tcp": z_surface + z_margin,
        "x_min_safe": x_min + xy_margin,
        "x_max_safe": x_max - xy_margin,
        "y_min_safe": y_min + xy_margin,
        "y_max_safe": y_max - xy_margin,
        "xy_margin_mm": xy_margin,
        "z_margin_mm": z_margin,
        "slow_zone_mm": slow_zone,
    }


def _workspace_status(self) -> Dict[str, Any]:
    cfg = getattr(self, "workspace", _workspace_default())
    limits = _workspace_limits_from_cfg(cfg)
    tcp = self._copy_tcp()
    status = {
        "enabled": bool(cfg.get("enabled")),
        "configured": limits is not None,
        "inside": True,
        "jog_locked": False,
        "outside_axes": [],
        "message": "Workspace desabilitado ou não configurado.",
        "limits": limits,
    }
    if not status["enabled"]:
        status["message"] = "Workspace desabilitado."
        return status
    if limits is None:
        status["inside"] = False
        status["jog_locked"] = True
        status["message"] = "Workspace habilitado, mas P1/P2 ainda não foram definidos."
        return status

    outside = []
    x, y, z = float(tcp[0]), float(tcp[1]), float(tcp[2])
    if x < limits["x_min_safe"]:
        outside.append("X-")
    elif x > limits["x_max_safe"]:
        outside.append("X+")
    if y < limits["y_min_safe"]:
        outside.append("Y-")
    elif y > limits["y_max_safe"]:
        outside.append("Y+")
    if z < limits["z_min_tcp"]:
        outside.append("Z-")

    status["outside_axes"] = outside
    status["inside"] = len(outside) == 0
    status["jog_locked"] = not status["inside"]
    status["message"] = (
        "TCP dentro da área de trabalho."
        if status["inside"] else
        "TCP fora da área de trabalho. Controle bloqueado. Use Drag Mode / Free Drive para recolocar o TCP dentro da área da mesa."
    )
    return status


def _get_workspace_config(self):
    cfg = copy.deepcopy(getattr(self, "workspace", _workspace_default()))
    return {"config": cfg, "status": self._workspace_status()}


def _set_workspace_config(self, dados):
    cfg = copy.deepcopy(getattr(self, "workspace", _workspace_default()))
    dados = dados or {}

    if "enabled" in dados:
        cfg["enabled"] = bool(dados.get("enabled"))
    if "p1" in dados:
        cfg["p1"] = _pose3_from_any(dados.get("p1"))
    if "p2" in dados:
        cfg["p2"] = _pose3_from_any(dados.get("p2"))

    for key, lo, hi, default in (
        ("z_margin_mm", 0.0, 300.0, 10.0),
        ("xy_margin_mm", 0.0, 300.0, 5.0),
        ("slow_zone_mm", 1.0, 500.0, 30.0),
    ):
        if key in dados:
            val = float(dados.get(key))
            if not (lo <= val <= hi):
                raise ValueError(f"{key} fora do intervalo permitido [{lo}, {hi}].")
            cfg[key] = val
        elif key not in cfg:
            cfg[key] = default

    p1 = _pose3_from_any(cfg.get("p1"))
    p2 = _pose3_from_any(cfg.get("p2"))
    if p1 and p2:
        if abs(p1[2] - p2[2]) > 20.0:
            raise ValueError("P1 e P2 têm diferença de Z maior que 20 mm. A mesa foi assumida horizontal.")
        if abs(p1[0] - p2[0]) < 20.0 or abs(p1[1] - p2[1]) < 20.0:
            raise ValueError("P1 e P2 precisam formar cantos opostos com diferença útil em X e Y.")

    with self._state_lock:
        self.workspace = cfg
        self.diagnosticos["workspace"] = self._workspace_status()
        if self.diagnosticos["workspace"].get("inside"):
            self._workspace_stop_emitido = False
    return self.get_workspace_config()


def _set_workspace_point_from_tcp(self, ponto: str):
    ponto = str(ponto).lower()
    if ponto not in ("p1", "p2"):
        raise ValueError("Ponto inválido. Use p1 ou p2.")
    self._amostrar_tcp_estavel_para_ponto()
    tcp = self._copy_tcp()
    cfg = copy.deepcopy(getattr(self, "workspace", _workspace_default()))
    cfg[ponto] = [float(tcp[0]), float(tcp[1]), float(tcp[2])]
    return self.set_workspace_config(cfg)


def _limitar_velocidade_workspace(self, eixo: int, vel: float) -> float:
    cfg = getattr(self, "workspace", _workspace_default())
    if not cfg.get("enabled"):
        return vel
    limits = _workspace_limits_from_cfg(cfg)
    if limits is None:
        return 0.0

    status = self._workspace_status()
    if status.get("jog_locked"):
        # Política exigida: se está fora, não se volta pelo controle.
        # Retorno para dentro deve ser por Drag Mode / Free Drive.
        if not getattr(self, "_workspace_stop_emitido", False):
            self.parar_grupo([0, 1, 2, 3, 4, 5])
            self._workspace_stop_emitido = True
            print(f"[WORKSPACE] Controle bloqueado: {status.get('message')}")
        with self._state_lock:
            self.diagnosticos["workspace"] = status
            self.diagnosticos["workspace_bloqueio"] = {
                "motivo": "TCP fora da área; retorno apenas por Drag Mode / Free Drive",
                "eixo": int(eixo),
                "vel_original": float(vel),
                "ts": time.time(),
            }
        return 0.0

    self._workspace_stop_emitido = False

    if eixo not in (0, 1, 2):
        return vel

    tcp = self._copy_tcp()
    pos = float(tcp[eixo])
    vel = float(vel)
    if abs(vel) < 1e-9:
        return vel

    slow = float(limits["slow_zone_mm"])

    def slow_factor(dist):
        return max(0.0, min(1.0, dist / slow))

    original = vel
    bloqueado = False

    if eixo == 0:
        lo, hi = limits["x_min_safe"], limits["x_max_safe"]
        if vel < 0:
            if pos <= lo:
                vel = 0.0; bloqueado = True
            elif pos - lo < slow:
                vel *= slow_factor(pos - lo)
        elif vel > 0:
            if pos >= hi:
                vel = 0.0; bloqueado = True
            elif hi - pos < slow:
                vel *= slow_factor(hi - pos)
    elif eixo == 1:
        lo, hi = limits["y_min_safe"], limits["y_max_safe"]
        if vel < 0:
            if pos <= lo:
                vel = 0.0; bloqueado = True
            elif pos - lo < slow:
                vel *= slow_factor(pos - lo)
        elif vel > 0:
            if pos >= hi:
                vel = 0.0; bloqueado = True
            elif hi - pos < slow:
                vel *= slow_factor(hi - pos)
    elif eixo == 2:
        lo = limits["z_min_tcp"]
        if vel < 0:
            if pos <= lo:
                vel = 0.0; bloqueado = True
            elif pos - lo < slow:
                vel *= slow_factor(pos - lo)

    if bloqueado:
        with self._state_lock:
            self.diagnosticos["workspace_bloqueio"] = {
                "motivo": "limite interno atingido",
                "eixo": int(eixo),
                "vel_original": original,
                "pos": pos,
                "ts": time.time(),
            }
    return vel


# ---------------------------------------------------------------------------
# TCP oficial / telemetria
# ---------------------------------------------------------------------------

def _refresh_tcp_from_robot(self, origem: str = "sdk", aplicar_filtro: bool = True) -> bool:
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
            self.diagnosticos["workspace"] = self._workspace_status()
        return True
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["tcp_status"] = f"Falha get_tcp_position: {e}"
        return False


def _amostrar_tcp_estavel_para_ponto(self) -> bool:
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
        self.diagnosticos["workspace"] = self._workspace_status()
    return True


_original_conectar = adapter.conectar
_original_salvar_ponto_atual = adapter.salvar_ponto_atual
_original_update_telemetry_low_freq = adapter._update_telemetry_low_freq
_original_apply_telemetry_updates = adapter._apply_telemetry_updates
_original_snapshot_state = adapter.snapshot_state
_original_enviar_jog = adapter.enviar_jog

adapter.workspace = _workspace_default()
adapter._workspace_stop_emitido = False


def _patched_snapshot_state(self):
    dados = _original_snapshot_state()
    dados["workspace"] = self.get_workspace_config()
    return dados


def _patched_apply_telemetry_updates(self, updates, debug):
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
        for _ in range(5):
            if self._refresh_tcp_from_robot("connect", aplicar_filtro=False):
                break
            time.sleep(0.08)
        if self.on_state_update:
            self.on_state_update(self.snapshot_state())
    return ok


def _patched_salvar_ponto_atual(self, tipo):
    self._amostrar_tcp_estavel_para_ponto()
    return _original_salvar_ponto_atual(tipo)


def _patched_update_telemetry_low_freq(self):
    self._refresh_tcp_from_robot("low_freq")
    result = _original_update_telemetry_low_freq()
    with self._state_lock:
        self.diagnosticos["workspace"] = self._workspace_status()
    return result


def _patched_enviar_jog(self, eixo, vel, coord):
    vel_filtrada = self._limitar_velocidade_workspace(int(eixo), float(vel))
    return _original_enviar_jog(eixo, vel_filtrada, coord)


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


adapter._refresh_tcp_from_robot = MethodType(_refresh_tcp_from_robot, adapter)
adapter._amostrar_tcp_estavel_para_ponto = MethodType(_amostrar_tcp_estavel_para_ponto, adapter)
adapter._workspace_status = MethodType(_workspace_status, adapter)
adapter.get_workspace_config = MethodType(_get_workspace_config, adapter)
adapter.set_workspace_config = MethodType(_set_workspace_config, adapter)
adapter.set_workspace_point_from_tcp = MethodType(_set_workspace_point_from_tcp, adapter)
adapter._limitar_velocidade_workspace = MethodType(_limitar_velocidade_workspace, adapter)
adapter.snapshot_state = MethodType(_patched_snapshot_state, adapter)
adapter._apply_telemetry_updates = MethodType(_patched_apply_telemetry_updates, adapter)
adapter.conectar = MethodType(_patched_conectar, adapter)
adapter.salvar_ponto_atual = MethodType(_patched_salvar_ponto_atual, adapter)
adapter._update_telemetry_low_freq = MethodType(_patched_update_telemetry_low_freq, adapter)
adapter.enviar_jog = MethodType(_patched_enviar_jog, adapter)
adapter.iniciar_loop_controle = MethodType(_patched_iniciar_loop_controle, adapter)
