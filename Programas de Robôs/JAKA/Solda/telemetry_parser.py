# telemetry_parser.py
# Parser defensivo para telemetria JAKA. Não depende de Flask nem de pygame.

import copy
import time
from typing import Any, Dict, List, Optional, Tuple


class TelemetryParser:
    @staticmethod
    def default_diagnostics() -> Dict[str, Any]:
        return {
            "temperaturas": [0.0] * 6,
            "correntes": [0.0] * 6,
            "tensoes": [0.0] * 6,
            "torques": [0.0] * 6,
            "uptime_segundos": 0,
            "backend_uptime_segundos": 0,
            "robot_uptime_segundos": None,
            "uptime_fonte": "backend",
            "status_emergencia": False,
            "protective_stop": False,
            "power_on": False,
            "enabled": False,
            "inpos": False,
            "codigo_erro": 0,
            "controller_temperature": None,
            "robot_average_voltage": None,
            "robot_average_current": None,
            "executando_trajetoria": False,
            "saida_digital_ativa": False,
            "telemetria_real": False,
            "telemetria_origem": "placeholder",
            "telemetria_status": "Telemetria real ainda não validada",
            "telemetria_parser": "none",
            "telemetria_confianca": 0,
            "ultima_telemetria_real_ts": None,
            "debug_telemetria": {},
        }

    @staticmethod
    def _to_float(value: Any, default: Optional[float] = None) -> Optional[float]:
        try:
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _to_bool(value: Any) -> bool:
        try:
            return bool(int(value))
        except Exception:
            return bool(value)

    @staticmethod
    def _shape(value: Any) -> Any:
        if isinstance(value, dict):
            return {k: TelemetryParser._shape(v) for k, v in list(value.items())[:10]}
        if isinstance(value, (list, tuple)):
            if not value:
                return [0]
            return [len(value), TelemetryParser._shape(value[0])]
        return type(value).__name__

    @staticmethod
    def _preview(value: Any, max_len: int = 900) -> str:
        text = repr(value)
        return text[:max_len] + ("..." if len(text) > max_len else "")

    @staticmethod
    def _plausible(candidate: Dict[str, Any]) -> int:
        currents = candidate.get("correntes", [])
        temps = candidate.get("temperaturas", [])
        volts = candidate.get("tensoes", [])
        score = 0

        if len(currents) == 6 and all(isinstance(x, (int, float)) for x in currents):
            if all(-0.05 <= x <= 30 for x in currents):
                score += 3
            if any(abs(x) > 0.001 for x in currents):
                score += 1
        if len(temps) == 6 and all(isinstance(x, (int, float)) for x in temps):
            if all(5 <= x <= 120 for x in temps):
                score += 4
            if 15 <= (sum(temps) / 6.0) <= 80:
                score += 2
            if max(temps) - min(temps) < 40:
                score += 1
        if len(volts) == 6 and all(isinstance(x, (int, float)) for x in volts):
            if all(0 <= x <= 100 for x in volts):
                score += 1
        return score

    @staticmethod
    def _parse_joint_rows(rows: Any, parser_name: str) -> Optional[Dict[str, Any]]:
        if not isinstance(rows, (list, tuple)) or len(rows) < 6:
            return None

        correntes: List[float] = []
        tensoes: List[float] = []
        temperaturas: List[float] = []
        torques: List[float] = []
        boot_times: List[float] = []

        for idx in range(6):
            row = rows[idx]
            if isinstance(row, dict):
                cur = TelemetryParser._to_float(row.get("current", row.get("cur")))
                volt = TelemetryParser._to_float(row.get("voltage", row.get("volt")))
                temp = TelemetryParser._to_float(row.get("temperature", row.get("temp")))
                torque = TelemetryParser._to_float(row.get("torque"))
                boot_time = TelemetryParser._to_float(row.get("running_time_after_boot", row.get("boot_time")))
            elif isinstance(row, (list, tuple)) and len(row) >= 3:
                cur = TelemetryParser._to_float(row[0])
                volt = TelemetryParser._to_float(row[1])
                temp = TelemetryParser._to_float(row[2])
                torque = TelemetryParser._to_float(row[9] if len(row) > 9 else None)
                boot_time = TelemetryParser._to_float(row[8] if len(row) > 8 else None)
            else:
                return None

            if cur is None or volt is None or temp is None:
                return None
            correntes.append(round(cur, 3))
            tensoes.append(round(volt, 3))
            temperaturas.append(round(temp, 2))
            torques.append(round(torque or 0.0, 3))
            if boot_time is not None and boot_time >= 0:
                boot_times.append(float(boot_time))

        cand = {
            "parser": parser_name,
            "correntes": correntes,
            "tensoes": tensoes,
            "temperaturas": temperaturas,
            "torques": torques,
            "robot_uptime_segundos": int(max(boot_times)) if boot_times else None,
        }
        cand["score"] = TelemetryParser._plausible(cand)
        return cand

    @staticmethod
    def _parse_transposed(matrix: Any, parser_name: str) -> Optional[Dict[str, Any]]:
        if not isinstance(matrix, (list, tuple)) or len(matrix) < 3:
            return None
        try:
            currents = matrix[0]
            volts = matrix[1]
            temps = matrix[2]
            if not all(isinstance(v, (list, tuple)) and len(v) >= 6 for v in (currents, volts, temps)):
                return None
            torques = matrix[9] if len(matrix) > 9 and isinstance(matrix[9], (list, tuple)) and len(matrix[9]) >= 6 else [0] * 6
            boot_times = matrix[8] if len(matrix) > 8 and isinstance(matrix[8], (list, tuple)) and len(matrix[8]) >= 6 else []
            cand = {
                "parser": parser_name,
                "correntes": [round(float(x), 3) for x in currents[:6]],
                "tensoes": [round(float(x), 3) for x in volts[:6]],
                "temperaturas": [round(float(x), 2) for x in temps[:6]],
                "torques": [round(float(x), 3) for x in torques[:6]],
                "robot_uptime_segundos": int(max(float(x) for x in boot_times[:6])) if boot_times else None,
            }
            cand["score"] = TelemetryParser._plausible(cand)
            return cand
        except Exception:
            return None

    @staticmethod
    def _parse_flat(flat: Any) -> List[Dict[str, Any]]:
        if not isinstance(flat, (list, tuple)) or len(flat) < 60:
            return []
        vals = list(flat[:60])
        candidates = []
        rows = [vals[i * 10:(i + 1) * 10] for i in range(6)]
        c1 = TelemetryParser._parse_joint_rows(rows, "flat_row_major_6x10")
        if c1:
            candidates.append(c1)
        transposed = [vals[i * 6:(i + 1) * 6] for i in range(10)]
        c2 = TelemetryParser._parse_transposed(transposed, "flat_transposed_10x6")
        if c2:
            candidates.append(c2)
        return candidates

    @staticmethod
    def parse_monitor_data(monitor: Any, origem: str) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
        """Tenta interpretar robot_monitor_data/monitor_data.

        Retorna (dados, debug). dados=None significa que não houve formato validado.
        """
        debug: Dict[str, Any] = {
            "origem": origem,
            "monitor_shape": TelemetryParser._shape(monitor),
            "monitor_preview": TelemetryParser._preview(monitor),
            "candidates": [],
        }
        if monitor is None:
            return None, debug

        candidates: List[Dict[str, Any]] = []
        controller_temp = None
        avg_voltage = None
        avg_current = None
        joints = monitor

        if isinstance(monitor, dict):
            joints = monitor.get("joints") or monitor.get("joint") or monitor.get("joint_data") or monitor.get("jointMonitor") or monitor.get("monitor_data") or monitor.get("robot_monitor_data")
            controller_temp = TelemetryParser._to_float(monitor.get("controller_temperature"))
            avg_voltage = TelemetryParser._to_float(monitor.get("robot_average_voltage"))
            avg_current = TelemetryParser._to_float(monitor.get("robot_average_current"))

        elif isinstance(monitor, (list, tuple)):
            # Formato documentado mais provável: [major, minor, cab_temp, avg_voltage, avg_current, joint_data]
            if len(monitor) >= 6 and isinstance(monitor[5], (list, tuple, dict)):
                controller_temp = TelemetryParser._to_float(monitor[2])
                avg_voltage = TelemetryParser._to_float(monitor[3])
                avg_current = TelemetryParser._to_float(monitor[4])
                joints = monitor[5]

        c = TelemetryParser._parse_joint_rows(joints, "joint_rows_6x10")
        if c:
            candidates.append(c)
        c = TelemetryParser._parse_transposed(joints, "transposed_10x6")
        if c:
            candidates.append(c)
        candidates.extend(TelemetryParser._parse_flat(joints))

        candidates.sort(key=lambda x: x.get("score", 0), reverse=True)
        debug["candidates"] = [copy.deepcopy(c) for c in candidates[:4]]

        if not candidates or candidates[0].get("score", 0) < 6:
            return None, debug

        best = copy.deepcopy(candidates[0])
        best.update({
            "telemetria_real": True,
            "telemetria_origem": origem,
            "telemetria_status": f"Telemetria real validada via {origem} ({best['parser']})",
            "telemetria_parser": best["parser"],
            "telemetria_confianca": best.get("score", 0),
            "ultima_telemetria_real_ts": time.time(),
        })
        if controller_temp is not None:
            best["controller_temperature"] = round(controller_temp, 2)
        if avg_voltage is not None:
            best["robot_average_voltage"] = round(avg_voltage, 3)
        if avg_current is not None:
            best["robot_average_current"] = round(avg_current, 3)
        if best.get("robot_uptime_segundos") is not None:
            best["uptime_segundos"] = best["robot_uptime_segundos"]
            best["uptime_fonte"] = f"{origem}.{best['parser']}.boot_time"
        return best, debug

    @staticmethod
    def parse_robot_status(status_res: Any) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        updates: Dict[str, Any] = {}
        debug: Dict[str, Any] = {"source": "SDK get_robot_status", "status_shape": TelemetryParser._shape(status_res), "status_preview": TelemetryParser._preview(status_res)}
        if not status_res or not isinstance(status_res, (list, tuple)) or len(status_res) < 2 or status_res[0] != 0:
            return updates, debug

        status = status_res[1]
        monitor = None
        if isinstance(status, (list, tuple)):
            if len(status) > 0:
                updates["codigo_erro"] = status[0]
            if len(status) > 1:
                updates["inpos"] = TelemetryParser._to_bool(status[1])
            if len(status) > 2:
                updates["power_on"] = TelemetryParser._to_bool(status[2])
            if len(status) > 3:
                updates["enabled"] = TelemetryParser._to_bool(status[3])
            if len(status) > 5:
                updates["protective_stop"] = TelemetryParser._to_bool(status[5])
            if len(status) > 18 and isinstance(status[18], (list, tuple)) and len(status[18]) >= 6:
                updates["tcp"] = [float(x) for x in status[18][:6]]
            # Documentos/sumários anteriores indicavam campo 21 como robot_monitor_data.
            # Tentamos vários índices defensivamente porque algumas versões divergem.
            for idx in (20, 21):
                if len(status) > idx and isinstance(status[idx], (list, tuple, dict)):
                    monitor = status[idx]
                    debug["monitor_index_used_candidate"] = idx
                    break
            if len(status) > 23:
                updates["status_emergencia"] = TelemetryParser._to_bool(status[23])
        elif isinstance(status, dict):
            updates["codigo_erro"] = status.get("errcode", status.get("err_code", 0))
            for key in ("inpos", "power_on", "enabled", "protective_stop", "emergency_stop"):
                if key in status:
                    updates["status_emergencia" if key == "emergency_stop" else key] = TelemetryParser._to_bool(status[key])
            cart = status.get("cart_position") or status.get("actual_position") or status.get("tcp")
            if isinstance(cart, (list, tuple)) and len(cart) >= 6:
                updates["tcp"] = [float(x) for x in cart[:6]]
            monitor = status.get("robot_monitor_data") or status.get("monitor_data")

        parsed, mon_debug = TelemetryParser.parse_monitor_data(monitor, "SDK get_robot_status")
        debug["monitor_debug"] = mon_debug
        if parsed:
            updates.update(parsed)
        return updates, debug

    @staticmethod
    def parse_tcp_payload(payload: Any) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        updates: Dict[str, Any] = {}
        debug: Dict[str, Any] = {"source": "TCP10000", "payload_shape": TelemetryParser._shape(payload), "payload_preview": TelemetryParser._preview(payload)}
        if not isinstance(payload, dict):
            return updates, debug

        candidates = [payload]
        for key in ("data", "result", "res", "state"):
            val = payload.get(key)
            if isinstance(val, dict):
                candidates.append(val)

        monitor = None
        for obj in candidates:
            if monitor is None:
                monitor = obj.get("monitor_data") or obj.get("monitorData") or obj.get("robot_monitor_data")
            cart = obj.get("actual_position") or obj.get("cart_position") or obj.get("tcp")
            if isinstance(cart, (list, tuple)) and len(cart) >= 6:
                updates["tcp"] = [float(x) for x in cart[:6]]
            for key in ("emergency_stop", "protective_stop", "enabled", "inpos"):
                if key in obj:
                    updates["status_emergencia" if key == "emergency_stop" else key] = TelemetryParser._to_bool(obj[key])
            dout = obj.get("dout") or obj.get("digital_output")
            if isinstance(dout, (list, tuple)) and len(dout) > 1:
                updates["saida_digital_ativa"] = bool(dout[1])

        parsed, mon_debug = TelemetryParser.parse_monitor_data(monitor, "TCP10000")
        debug["monitor_debug"] = mon_debug
        if parsed:
            updates.update(parsed)
        return updates, debug
