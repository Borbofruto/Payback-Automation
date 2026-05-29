# robot_adapter.py — wrapper V17.2
#
# Wrapper local para a IHM Solda V17.2.
# Carrega o adapter histórico em Programas de Robôs/JAKA/Solda e aplica camadas
# de correção/segurança sem mexer na pasta histórica:
# - TCP real publicado continuamente;
# - telemetria não sobrescreve o TCP oficial usado para pontos;
# - workspace orientado por P1/P2 + base do robô;
# - joystick ignorado durante trajetória, sem mandar jog_stop;
# - watchdog de colisão/parada em alta frequência para forçar solda OFF;
# - saída digital de solda bloqueada e forçada OFF fora de trajetória;
# - jog com soft-start para toque curto não virar deslocamento gigante;
# - trajetória não aborta por "robô parado".

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
    if getattr(self, "grupo_ativo", None) in ("ROTAT_TCP", "EIXO_RZ"):
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
        "z_margin_mm": 0.5,
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


def _dot2(a, b):
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1])


def _sub2(a, b):
    return [float(a[0]) - float(b[0]), float(a[1]) - float(b[1])]


def _add2(a, b):
    return [float(a[0]) + float(b[0]), float(a[1]) + float(b[1])]


def _mul2(a, s):
    return [float(a[0]) * float(s), float(a[1]) * float(s)]


def _norm2(a):
    return math.hypot(float(a[0]), float(a[1]))


def _workspace_limits_from_cfg(cfg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    p1 = _pose3_from_any(cfg.get("p1"))
    p2 = _pose3_from_any(cfg.get("p2"))
    if p1 is None or p2 is None:
        return None

    a = [p1[0], p1[1]]
    b = [p2[0], p2[1]]
    origin = [0.0, 0.0]
    ab = _sub2(b, a)
    edge_len = _norm2(ab)
    if edge_len < 1e-6:
        return None

    u = [ab[0] / edge_len, ab[1] / edge_len]
    ao = _sub2(origin, a)
    s_origin = _dot2(ao, u)
    q = _add2(a, _mul2(u, s_origin))
    q_to_origin = _sub2(origin, q)
    depth = _norm2(q_to_origin)
    if depth < 1e-6:
        return None

    n = [q_to_origin[0] / depth, q_to_origin[1] / depth]
    near_a = _add2(a, _mul2(n, depth))
    near_b = _add2(b, _mul2(n, depth))

    z_surface = min(float(p1[2]), float(p2[2]))
    z_margin = float(cfg.get("z_margin_mm", 0.5))
    xy_margin = float(cfg.get("xy_margin_mm", 5.0))
    slow_zone = max(1.0, float(cfg.get("slow_zone_mm", 30.0)))

    verts = [a, b, near_b, near_a]
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]

    return {
        "mode": "oriented_from_far_edge_and_robot_base",
        "p1_xy": a,
        "p2_xy": b,
        "origin_xy": origin,
        "projection_origin_on_edge": q,
        "near_p1_xy": near_a,
        "near_p2_xy": near_b,
        "vertices_xy": verts,
        "u_edge": u,
        "n_depth": n,
        "edge_length_mm": edge_len,
        "depth_mm": depth,
        "s_origin_mm": s_origin,
        "s_min": 0.0,
        "s_max": edge_len,
        "t_min": 0.0,
        "t_max": depth,
        "s_min_safe": xy_margin,
        "s_max_safe": edge_len - xy_margin,
        "t_min_safe": xy_margin,
        "t_max_safe": depth - xy_margin,
        "z_p1": float(p1[2]),
        "z_p2": float(p2[2]),
        "z_surface_source": "min_p1_p2",
        "z_surface": z_surface,
        "z_min_tcp": z_surface + z_margin,
        "xy_margin_mm": xy_margin,
        "z_margin_mm": z_margin,
        "slow_zone_mm": slow_zone,
        "x_min": min(xs),
        "x_max": max(xs),
        "y_min": min(ys),
        "y_max": max(ys),
        "x_min_safe": min(xs) + xy_margin,
        "x_max_safe": max(xs) - xy_margin,
        "y_min_safe": min(ys) + xy_margin,
        "y_max_safe": max(ys) - xy_margin,
    }


def _workspace_coords_xy(limits: Dict[str, Any], x: float, y: float):
    p = [float(x), float(y)]
    rel = _sub2(p, limits["p1_xy"])
    return _dot2(rel, limits["u_edge"]), _dot2(rel, limits["n_depth"])


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
        status.update({
            "inside": False,
            "jog_locked": True,
            "message": "Workspace habilitado, mas P1/P2 não formam uma área geométrica válida.",
        })
        return status

    x, y, z = float(tcp[0]), float(tcp[1]), float(tcp[2])
    s, t = _workspace_coords_xy(limits, x, y)
    outside = []
    if s < limits["s_min_safe"]:
        outside.append("S-")
    elif s > limits["s_max_safe"]:
        outside.append("S+")
    if t < limits["t_min_safe"]:
        outside.append("T-")
    elif t > limits["t_max_safe"]:
        outside.append("T+")
    if z < limits["z_min_tcp"]:
        outside.append("Z-")

    status["workspace_coords"] = {"s_mm": s, "t_mm": t}
    status["outside_axes"] = outside
    status["inside"] = len(outside) == 0
    status["jog_locked"] = not status["inside"]
    status["message"] = (
        "TCP dentro da área de trabalho."
        if status["inside"] else
        "TCP fora da área de trabalho. Controle aceita apenas movimento de retorno."
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
        ("z_margin_mm", 0.0, 300.0, 0.5),
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


def _bloquear_jog_workspace(self, motivo: str, eixo: int, vel: float) -> float:
    self.parar_grupo([0, 1, 2, 3, 4, 5])
    self._workspace_stop_emitido = True
    status = self._workspace_status()
    with self._state_lock:
        self.diagnosticos["workspace"] = status
        self.diagnosticos["workspace_bloqueio"] = {
            "motivo": motivo,
            "eixo": int(eixo),
            "vel_original": float(vel),
            "ts": time.time(),
        }
    print(f"[WORKSPACE] Jog bloqueado: {motivo}")
    return 0.0


def _limitar_velocidade_workspace(self, eixo: int, vel: float) -> float:
    cfg = getattr(self, "workspace", _workspace_default())
    if not cfg.get("enabled"):
        return vel

    try:
        self._refresh_tcp_from_robot("workspace_limit")
    except Exception:
        pass

    limits = _workspace_limits_from_cfg(cfg)
    if limits is None:
        return self._bloquear_jog_workspace("workspace inválido ou não configurado", eixo, vel)

    status = self._workspace_status()
    if status.get("jog_locked"):
        return self._bloquear_jog_workspace("TCP fora da área; movimento não autorizado", eixo, vel)

    self._workspace_stop_emitido = False
    if eixo not in (0, 1, 2):
        return vel

    tcp = self._copy_tcp()
    vel = float(vel)
    if abs(vel) < 1e-9:
        return vel

    if eixo == 2:
        z = float(tcp[2])
        lo = float(limits["z_min_tcp"])
        slow = float(limits["slow_zone_mm"])
        if vel < 0:
            if z <= lo:
                return self._bloquear_jog_workspace(f"limite Z atingido: TCP Z={z:.3f}, limite={lo:.3f}", eixo, vel)
            dist = z - lo
            if dist < slow:
                vel = vel * max(0.0, min(1.0, dist / slow))
                if abs(vel) < 0.01:
                    return self._bloquear_jog_workspace(f"limite Z próximo: TCP Z={z:.3f}, limite={lo:.3f}", eixo, vel)
        return vel

    x, y = float(tcp[0]), float(tcp[1])
    s, t = _workspace_coords_xy(limits, x, y)
    slow = float(limits["slow_zone_mm"])
    move_vec = [vel, 0.0] if eixo == 0 else [0.0, vel]
    ds = _dot2(move_vec, limits["u_edge"])
    dt = _dot2(move_vec, limits["n_depth"])
    factor = 1.0

    def apply_boundary(local_pos, local_vel, lo, hi, factor):
        if local_vel < 0:
            if local_pos <= lo:
                return 0.0
            dist = local_pos - lo
            if dist < slow:
                return min(factor, max(0.0, min(1.0, dist / slow)))
        elif local_vel > 0:
            if local_pos >= hi:
                return 0.0
            dist = hi - local_pos
            if dist < slow:
                return min(factor, max(0.0, min(1.0, dist / slow)))
        return factor

    factor = apply_boundary(s, ds, limits["s_min_safe"], limits["s_max_safe"], factor)
    factor = apply_boundary(t, dt, limits["t_min_safe"], limits["t_max_safe"], factor)
    if factor <= 0.0:
        return self._bloquear_jog_workspace("limite XY orientado atingido", eixo, vel)
    return vel * factor


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


# ---------------------------------------------------------------------------
# Segurança de trajetória / solda / colisão
# ---------------------------------------------------------------------------

def _parse_bool(v):
    try:
        if v is None:
            return None
        return bool(int(v))
    except Exception:
        return bool(v)


def _truthy_fault_value(v: Any) -> bool:
    if v is None:
        return False
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return abs(float(v)) > 1e-9
    if isinstance(v, str):
        s = v.strip().lower()
        return s not in ("", "0", "false", "none", "normal", "ok", "no", "off")
    if isinstance(v, (list, tuple)):
        if len(v) >= 2 and isinstance(v[0], (int, float)) and int(v[0]) == 0:
            return _truthy_fault_value(v[1])
        return any(_truthy_fault_value(x) for x in v)
    if isinstance(v, dict):
        return any(_truthy_fault_value(x) for x in v.values())
    return bool(v)


_COLLISION_KEYWORDS = (
    "collision", "collide", "collided", "colisao", "colisão",
    "collisiondetected", "collision_detected", "collision_status", "collision_stop",
    "is_collision", "in_collision", "collision_state",
)


def _scan_collision_keys(value: Any, path: str = "status", depth: int = 0) -> List[str]:
    if value is None or depth > 5:
        return []
    reasons: List[str] = []
    if isinstance(value, dict):
        for key, val in value.items():
            key_s = str(key).lower().replace("_", "")
            child_path = f"{path}.{key}"
            if any(k.replace("_", "") in key_s for k in _COLLISION_KEYWORDS) and _truthy_fault_value(val):
                reasons.append(f"{child_path}={repr(val)[:120]}")
            reasons.extend(_scan_collision_keys(val, child_path, depth + 1))
    elif isinstance(value, (list, tuple)):
        for i, item in enumerate(value[:120]):
            reasons.extend(_scan_collision_keys(item, f"{path}[{i}]", depth + 1))
    elif hasattr(value, "__dict__"):
        reasons.extend(_scan_collision_keys(vars(value), path, depth + 1))
    return reasons


def _probe_collision_methods(self) -> List[str]:
    robot = getattr(getattr(self, "driver", None), "robot", None)
    if robot is None:
        return []
    reasons: List[str] = []

    explicit = {
        "is_collision",
        "is_in_collision",
        "is_collision_detected",
        "get_collision_status",
        "get_collision_state",
        "get_collision_detected",
        "get_collision_protect_status",
    }
    try:
        dynamic = {
            name for name in dir(robot)
            if ("collis" in name.lower() or "collision" in name.lower())
            and not name.startswith("set_")
            and not name.startswith("clear")
        }
    except Exception:
        dynamic = set()

    for name in sorted(explicit | dynamic):
        fn = getattr(robot, name, None)
        if not callable(fn):
            continue
        try:
            ret = fn()
            if _truthy_fault_value(ret):
                reasons.append(f"{name}()={repr(ret)[:180]}")
        except TypeError:
            continue
        except Exception as e:
            with self._state_lock:
                self.diagnosticos.setdefault("collision_method_errors", {})[name] = str(e)
    return reasons


def _robot_status_updates(self) -> Dict[str, Any]:
    if getattr(self, "modo_simulacao", True):
        return {}
    try:
        if not self.driver.has_robot():
            return {"driver_sem_robo": True}
    except Exception:
        return {"driver_sem_robo": True}

    updates: Dict[str, Any] = {}
    try:
        raw = self.driver.get_robot_status()
        if isinstance(raw, (list, tuple)) and len(raw) > 0 and raw[0] != 0:
            updates["robot_status_bad_ret"] = True
            updates["robot_status_retcode"] = raw[0]
        parsed, debug = _REAL_MOD.TelemetryParser.parse_robot_status(raw)
        updates.update(dict(parsed or {}))

        collision_reasons = _scan_collision_keys(raw, "robot_status")
        collision_reasons.extend(_probe_collision_methods(self))
        if collision_reasons:
            updates["collision_detected"] = True
            updates["collision_reasons"] = collision_reasons[:16]

        with self._state_lock:
            self.diagnosticos["robot_status_raw_preview"] = repr(raw)[:900]
            self.diagnosticos["robot_status_collision_scan"] = collision_reasons[:16]
        return updates
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["robot_status_probe_error"] = str(e)
        return {"robot_status_probe_failed": True, "robot_status_probe_error": str(e)}


def _robot_fault_reasons_from_diag(self) -> List[str]:
    with self._state_lock:
        d = copy.deepcopy(self.diagnosticos)
    reasons = []
    code = d.get("codigo_erro", 0)
    try:
        if int(code) != 0:
            reasons.append(f"codigo_erro={code}")
    except Exception:
        if code not in (None, "", 0, "0"):
            reasons.append(f"codigo_erro={code}")

    for key, label in (
        ("status_emergencia", "emergência"),
        ("protective_stop", "protective_stop"),
        ("driver_sem_robo", "driver_sem_robo"),
        ("robot_status_bad_ret", "robot_status_bad_ret"),
        ("robot_status_probe_failed", "robot_status_probe_failed"),
        ("collision_detected", "collision_detected"),
        ("collision_status", "collision_status"),
        ("collision_stop", "collision_stop"),
    ):
        if _parse_bool(d.get(key)) is True:
            reasons.append(label)

    collision_reasons = d.get("collision_reasons") or d.get("robot_status_collision_scan")
    if collision_reasons:
        reasons.append(f"collision_reasons={collision_reasons}")
    return reasons


def _probe_robot_fault(self) -> List[str]:
    updates = self._robot_status_updates()
    if updates:
        with self._state_lock:
            self.diagnosticos.update(updates)
    return self._robot_fault_reasons_from_diag()


def _force_weld_off_raw(self, origem="raw", throttle_s: Optional[float] = None) -> bool:
    now = time.time()
    if throttle_s is not None:
        last = float(getattr(self, "_weld_safety_last_force_off_ts", 0.0) or 0.0)
        if now - last < throttle_s:
            return False
        self._weld_safety_last_force_off_ts = now
    try:
        ok = _original_set_saida_digital(False, confirmar=False)
        with self._state_lock:
            self.saida_digital_ativa = False
            self.diagnosticos["saida_digital_ativa"] = False
            self.diagnosticos["solda_forcada_off_origem"] = origem
            self.diagnosticos["solda_forcada_off_ts"] = now
        return bool(ok)
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["solda_forcada_off_erro"] = str(e)
            self.diagnosticos["solda_forcada_off_origem"] = origem
            self.diagnosticos["solda_forcada_off_ts"] = now
        print(f"[SEGURANÇA] Falha forçando solda OFF ({origem}): {e}")
        return False


def _forcar_saida_solda_off_fora_de_trajetoria(self, origem="watchdog") -> bool:
    if getattr(self, "executando_trajetoria", False):
        return False
    return self._force_weld_off_raw(origem, throttle_s=0.25)


def _marcar_interrupcao_trajetoria(self, motivo: str, reasons=None) -> None:
    reasons = [str(x) for x in (reasons or [])]
    print(f"[TRAJ] Interrupção: {motivo}. Reasons={reasons}")
    with self._state_lock:
        self.diagnosticos["trajetoria_interrompida"] = True
        self.diagnosticos["trajetoria_interrupcao_motivo"] = motivo
        self.diagnosticos["trajetoria_interrupcao_reasons"] = reasons
        self.diagnosticos["trajetoria_interrupcao_ts"] = time.time()
        self.executando_trajetoria = False
        self.diagnosticos["executando_trajetoria"] = False
    self._force_weld_off_raw("falha_trajetoria", throttle_s=None)
    try:
        self.definir_erro_trajetoria([f"{motivo}: " + "; ".join(reasons[:4])])
    except Exception:
        pass


def _trajectory_fault_watchdog_loop(self):
    while True:
        try:
            if getattr(self, "executando_trajetoria", False):
                reasons = self._probe_robot_fault()
                if reasons:
                    self._marcar_interrupcao_trajetoria("falha/parada detectada pelo watchdog de trajetória", reasons)
            else:
                self._forcar_saida_solda_off_fora_de_trajetoria("traj_fault_watchdog_idle")
        except Exception as e:
            with self._state_lock:
                self.diagnosticos["traj_fault_watchdog_error"] = str(e)
        time.sleep(0.05)


def _start_trajectory_fault_watchdog(self):
    th = getattr(self, "_traj_fault_watchdog_thread", None)
    if th is not None and th.is_alive():
        return
    th = threading.Thread(target=lambda: self._trajectory_fault_watchdog_loop(), daemon=True)
    self._traj_fault_watchdog_thread = th
    th.start()
    print("[SEGURANÇA] Watchdog de colisão/trajetória iniciado (50 ms).")


def _consumir_botoes_joystick(self, joy) -> None:
    for b in (4, 5, 6, 7, 8, 15):
        try:
            self.last_btns[b] = joy.get_button(b)
        except Exception:
            pass


def _bloquear_joystick_por_trajetoria(self, joy=None) -> None:
    self.grupo_ativo = None
    if not getattr(self, "_traj_joystick_block_log_emitido", False):
        self._traj_joystick_block_log_emitido = True
        print("[SEGURANÇA] Joystick ignorado durante execução de trajetória.")
    with self._state_lock:
        self.diagnosticos["joystick_bloqueado_por_trajetoria"] = True
        self.diagnosticos["joystick_bloqueio_ts"] = time.time()
    if joy is not None:
        self._consumir_botoes_joystick(joy)


def _patched_aguardar_chegada_por_tcp(self, alvo, tol_mm=2.0, timeout_s=120.0, ciclos_estaveis=5, exigir_inpos=False) -> bool:
    if self.modo_simulacao:
        return True

    t0 = time.time()
    stable = 0
    last = None

    while time.time() - t0 < timeout_s:
        try:
            self._refresh_tcp_from_robot("traj_wait")
        except Exception:
            pass

        fault_reasons = self._probe_robot_fault()
        if fault_reasons:
            self._marcar_interrupcao_trajetoria("status de falha/parada do robô durante trajetória", fault_reasons)
            return False

        atual = self._copy_tcp()
        dist = self._dist_xyz(atual, alvo)
        delta = self._dist_xyz(atual, last) if last is not None else 999999.0
        last = atual

        inpos_ok = True
        if exigir_inpos:
            try:
                inpos = self._is_in_pos()
                inpos_ok = True if inpos is None else bool(inpos)
            except Exception:
                inpos_ok = True

        if dist <= tol_mm and delta <= 0.35 and inpos_ok:
            stable += 1
            if stable >= ciclos_estaveis:
                return True
        else:
            stable = 0

        time.sleep(0.03)

    self._marcar_interrupcao_trajetoria("timeout aguardando chegada ao último ponto", [f"timeout={timeout_s}s"])
    return False


# ---------------------------------------------------------------------------
# Patches sobre adapter histórico
# ---------------------------------------------------------------------------

_original_conectar = adapter.conectar
_original_salvar_ponto_atual = adapter.salvar_ponto_atual
_original_update_telemetry_low_freq = adapter._update_telemetry_low_freq
_original_apply_telemetry_updates = adapter._apply_telemetry_updates
_original_snapshot_state = adapter.snapshot_state
_original_enviar_jog = adapter.enviar_jog
_original_processar_joystick = adapter._processar_joystick
_original_set_saida_digital = adapter.set_saida_digital
_original_executar_trajetoria = adapter.executar_trajetoria

adapter.workspace = _workspace_default()
adapter._workspace_stop_emitido = False
adapter._traj_joystick_block_log_emitido = False
adapter._weld_safety_last_force_off_ts = 0.0
adapter._traj_fault_watchdog_thread = None
adapter._jog_soft_group = None
adapter._jog_soft_since = 0.0
adapter._jog_soft_eixo_ts = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}


def _patched_snapshot_state(self):
    dados = _original_snapshot_state()
    dados["workspace"] = self.get_workspace_config()
    dados["trajectory_safety"] = {
        "executando_trajetoria": bool(getattr(self, "executando_trajetoria", False)),
        "joystick_bloqueado_por_trajetoria": bool(self.diagnosticos.get("joystick_bloqueado_por_trajetoria", False)),
        "trajetoria_interrompida": bool(self.diagnosticos.get("trajetoria_interrompida", False)),
        "trajetoria_interrupcao_motivo": self.diagnosticos.get("trajetoria_interrupcao_motivo", ""),
        "solda_saida_digital_ativa": bool(getattr(self, "saida_digital_ativa", False)),
        "solda_forcada_off_fora_trajetoria": bool(self.diagnosticos.get("solda_forcada_off_fora_trajetoria", False)),
        "collision_detected": bool(self.diagnosticos.get("collision_detected", False)),
        "collision_reasons": self.diagnosticos.get("collision_reasons", []),
    }
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
        self._probe_robot_fault()
        self._forcar_saida_solda_off_fora_de_trajetoria("connect")
        self._start_trajectory_fault_watchdog()
        if self.on_state_update:
            self.on_state_update(self.snapshot_state())
    return ok


def _patched_set_saida_digital(self, ativo: bool, confirmar=False, timeout_s=2.0) -> bool:
    ativo = bool(ativo)
    if ativo and not getattr(self, "executando_trajetoria", False):
        with self._state_lock:
            self.diagnosticos["solda_bloqueada_fora_de_trajetoria"] = True
            self.diagnosticos["solda_bloqueio_ts"] = time.time()
        print("[SEGURANÇA] Saída digital de solda bloqueada fora de trajetória. Forçando OFF.")
        self._forcar_saida_solda_off_fora_de_trajetoria("bloqueio_set_true")
        return False

    ok = _original_set_saida_digital(ativo, confirmar=confirmar, timeout_s=timeout_s)
    if not ativo:
        with self._state_lock:
            self.diagnosticos["solda_bloqueada_fora_de_trajetoria"] = False
    return ok


def _patched_salvar_ponto_atual(self, tipo):
    if getattr(self, "executando_trajetoria", False):
        with self._state_lock:
            self.diagnosticos["ponto_bloqueado_por_trajetoria"] = True
            self.diagnosticos["ponto_bloqueio_ts"] = time.time()
        print("[SEGURANÇA] Marcação de ponto bloqueada durante trajetória.")
        return False
    self._amostrar_tcp_estavel_para_ponto()
    return _original_salvar_ponto_atual(tipo)


def _patched_update_telemetry_low_freq(self):
    self._refresh_tcp_from_robot("low_freq")
    result = _original_update_telemetry_low_freq()

    try:
        if getattr(self, "executando_trajetoria", False):
            fault_reasons = self._probe_robot_fault()
            if fault_reasons:
                self._marcar_interrupcao_trajetoria("status de falha/parada do robô", fault_reasons)
        else:
            self._forcar_saida_solda_off_fora_de_trajetoria("telemetry_low_freq")
    except Exception as e:
        with self._state_lock:
            self.diagnosticos["trajectory_safety_update_error"] = str(e)

    with self._state_lock:
        self.diagnosticos["workspace"] = self._workspace_status()
    return result


def _jog_soft_start_factor(self, eixo: int, vel: float) -> float:
    if abs(float(vel)) < getattr(self, "DEADZONE", 0.2):
        return 1.0
    now = time.time()
    grupo = getattr(self, "grupo_ativo", None) or f"EIXO_{eixo}"
    if self._jog_soft_group != grupo:
        self._jog_soft_group = grupo
        self._jog_soft_since = now
        self._jog_soft_eixo_ts = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
    self._jog_soft_eixo_ts[int(eixo)] = now

    elapsed = max(0.0, now - float(self._jog_soft_since or now))
    if elapsed < 0.22:
        return 0.12
    if elapsed < 0.85:
        return 0.12 + ((elapsed - 0.22) / 0.63) * 0.88
    return 1.0


def _patched_enviar_jog(self, eixo, vel, coord):
    if getattr(self, "executando_trajetoria", False):
        self._bloquear_joystick_por_trajetoria()
        return None
    self._traj_joystick_block_log_emitido = False
    with self._state_lock:
        self.diagnosticos["joystick_bloqueado_por_trajetoria"] = False

    self._forcar_saida_solda_off_fora_de_trajetoria("antes_jog")
    vel_filtrada = self._limitar_velocidade_workspace(int(eixo), float(vel))
    factor = self._jog_soft_start_factor(int(eixo), vel_filtrada)
    vel_filtrada = float(vel_filtrada) * factor
    with self._state_lock:
        self.diagnosticos["jog_soft_start_factor"] = round(float(factor), 3)
    return _original_enviar_jog(eixo, vel_filtrada, coord)


def _patched_processar_joystick(self, joy):
    if getattr(self, "executando_trajetoria", False):
        self._bloquear_joystick_por_trajetoria(joy)
        return
    self._traj_joystick_block_log_emitido = False
    with self._state_lock:
        self.diagnosticos["joystick_bloqueado_por_trajetoria"] = False
    self._forcar_saida_solda_off_fora_de_trajetoria("joystick_idle")
    result = _original_processar_joystick(joy)
    if getattr(self, "grupo_ativo", None) is None:
        self._jog_soft_group = None
        self._jog_soft_since = 0.0
    return result


def _patched_executar_trajetoria(self) -> str:
    with self._state_lock:
        self.diagnosticos["trajetoria_interrompida"] = False
        self.diagnosticos["trajetoria_interrupcao_motivo"] = ""
        self.diagnosticos["trajetoria_interrupcao_reasons"] = []
        self.diagnosticos["joystick_bloqueado_por_trajetoria"] = False
        self.diagnosticos["solda_bloqueada_fora_de_trajetoria"] = False
        self.diagnosticos["collision_detected"] = False
        self.diagnosticos["collision_reasons"] = []

    self._start_trajectory_fault_watchdog()

    try:
        if getattr(self, "grupo_ativo", None) is not None:
            self.parar_grupo([0, 1, 2, 3, 4, 5])
            self.grupo_ativo = None
    except Exception:
        pass

    try:
        return _original_executar_trajetoria()
    finally:
        self._force_weld_off_raw("fim_trajetoria", throttle_s=None)
        self._traj_joystick_block_log_emitido = False
        with self._state_lock:
            self.diagnosticos["joystick_bloqueado_por_trajetoria"] = False


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

    self._start_trajectory_fault_watchdog()

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

                if not getattr(self, "executando_trajetoria", False):
                    self._forcar_saida_solda_off_fora_de_trajetoria("loop")

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
adapter._robot_status_updates = MethodType(_robot_status_updates, adapter)
adapter._robot_fault_reasons_from_diag = MethodType(_robot_fault_reasons_from_diag, adapter)
adapter._probe_robot_fault = MethodType(_probe_robot_fault, adapter)
adapter._force_weld_off_raw = MethodType(_force_weld_off_raw, adapter)
adapter._marcar_interrupcao_trajetoria = MethodType(_marcar_interrupcao_trajetoria, adapter)
adapter._forcar_saida_solda_off_fora_de_trajetoria = MethodType(_forcar_saida_solda_off_fora_de_trajetoria, adapter)
adapter._trajectory_fault_watchdog_loop = MethodType(_trajectory_fault_watchdog_loop, adapter)
adapter._start_trajectory_fault_watchdog = MethodType(_start_trajectory_fault_watchdog, adapter)
adapter._consumir_botoes_joystick = MethodType(_consumir_botoes_joystick, adapter)
adapter._bloquear_joystick_por_trajetoria = MethodType(_bloquear_joystick_por_trajetoria, adapter)
adapter._jog_soft_start_factor = MethodType(_jog_soft_start_factor, adapter)
adapter.snapshot_state = MethodType(_patched_snapshot_state, adapter)
adapter._apply_telemetry_updates = MethodType(_patched_apply_telemetry_updates, adapter)
adapter.conectar = MethodType(_patched_conectar, adapter)
adapter.set_saida_digital = MethodType(_patched_set_saida_digital, adapter)
adapter.salvar_ponto_atual = MethodType(_patched_salvar_ponto_atual, adapter)
adapter.aguardar_chegada_por_tcp = MethodType(_patched_aguardar_chegada_por_tcp, adapter)
adapter.executar_trajetoria = MethodType(_patched_executar_trajetoria, adapter)
adapter._update_telemetry_low_freq = MethodType(_patched_update_telemetry_low_freq, adapter)
adapter.enviar_jog = MethodType(_patched_enviar_jog, adapter)
adapter._processar_joystick = MethodType(_patched_processar_joystick, adapter)
adapter.iniciar_loop_controle = MethodType(_patched_iniciar_loop_controle, adapter)
