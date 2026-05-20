# robot_adapter.py
# Adapter de alto nível da IHM de Solda Payback.
# Cola driver JAKA, planner de trajetória, telemetria e joystick.

import copy
import json
import math
import random
import socket
import threading
import time
from typing import Any, Dict, List, Optional

import pygame

from jaka_driver import JakaDriver
from models import SegmentKind
from telemetry_parser import TelemetryParser
from trajectory_planner import TrajectoryPlanner


class RobotAdapter:
    def __init__(self):
        self.driver = JakaDriver()
        self.modo_simulacao = not self.driver.sdk_available()
        self.ip_atual: Optional[str] = None

        # Parâmetros de controle
        self.MAX_SPD_LINEAR = 120.0
        self.MAX_SPD_ROTAT = 6.0
        self.DEADZONE = 0.2
        self.vel_reproducao = 15.0
        self.vel_aproximacao = 150.0

        # Estado
        self._state_lock = threading.RLock()
        self._exec_lock = threading.Lock()
        self.posicao_atual_tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lista_pontos = []
        self.executando_trajetoria = False
        self.saida_digital_ativa = False
        self.angulo_operador = 0.0
        self.grupo_ativo = None
        self.last_sent_vels = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
        self.last_btns = {4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 15: 0}
        self.b5_press_time = None
        self.b5_triggered_long_press = False

        # Telemetria
        self.tempo_inicio_sistema = time.time()
        self.diagnosticos = TelemetryParser.default_diagnostics()
        self._last_telemetry_debug: Dict[str, Any] = {}
        self._contador_telemetria = 0
        self._tcp10000_stop = threading.Event()
        self._tcp10000_thread = None

        # Callbacks Socket.IO preenchidos pelo app.py
        self.on_state_update = None
        self.on_point_saved = None
        self.on_execution_status = None

        if self.modo_simulacao:
            print(f"[AVISO] SDK JAKA não carregado ({self.driver.sdk_error()}). Modo simulação ativo.")

    # ------------------------------------------------------------------
    # Estado público seguro
    # ------------------------------------------------------------------
    def sdk_carregado(self) -> bool:
        return self.driver.sdk_available()

    def _copy_tcp(self) -> List[float]:
        with self._state_lock:
            return list(self.posicao_atual_tcp)

    def _set_tcp(self, pose) -> None:
        with self._state_lock:
            self.posicao_atual_tcp = [float(x) for x in pose[:6]]

    def _get_pontos_snapshot(self):
        with self._state_lock:
            return copy.deepcopy(self.lista_pontos)

    def _set_pontos(self, pontos) -> None:
        with self._state_lock:
            self.lista_pontos = copy.deepcopy(pontos)
        self._emit_pontos()

    def _emit_pontos(self) -> None:
        if self.on_point_saved:
            self.on_point_saved(self._get_pontos_snapshot())

    def _emit_exec_status(self, message: str, status: str = "info") -> None:
        if self.on_execution_status:
            self.on_execution_status({"message": message, "status": status})

    def snapshot_state(self) -> Dict[str, Any]:
        return {
            "tcp": self._copy_tcp(),
            "pontos": self._get_pontos_snapshot(),
            "modo_sim": self.modo_simulacao,
            "angulo_operador": math.degrees(self.angulo_operador),
            "diagnosticos": copy.deepcopy(self.diagnosticos),
        }

    # ------------------------------------------------------------------
    # Conexão
    # ------------------------------------------------------------------
    def conectar(self, ip="192.168.0.200") -> bool:
        if not self.sdk_carregado():
            self.modo_simulacao = True
            self.ip_atual = ip
            print("[SIMULAÇÃO] SDK indisponível; conexão física ignorada.")
            return False

        try:
            ok = self.driver.connect(ip)
            self.modo_simulacao = not ok
            if ok:
                self.ip_atual = ip
                print(f"[ROBÔ] Conectado e habilitado em {ip}.")
                self._start_tcp10000_monitor()
                return True
            print(f"[ERRO] Falha ao conectar no robô em {ip}.")
            return False
        except Exception as e:
            self.modo_simulacao = True
            print(f"[ERRO] Falha crítica ao conectar no robô: {e}")
            return False

    # ------------------------------------------------------------------
    # Planejamento/validação
    # ------------------------------------------------------------------
    def validar_trajetoria_atual(self):
        return TrajectoryPlanner.plan(self._get_pontos_snapshot())

    # ------------------------------------------------------------------
    # Movimento e I/O
    # ------------------------------------------------------------------
    @staticmethod
    def _ret_ok(ret: Any) -> bool:
        return JakaDriver.ret_ok(ret)

    @staticmethod
    def _dist_xyz(a, b) -> float:
        return math.sqrt((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2 + (float(a[2]) - float(b[2])) ** 2)

    def _is_in_pos(self) -> Optional[bool]:
        if self.modo_simulacao:
            return True
        try:
            ret = self.driver.is_in_pos()
            if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0:
                return bool(ret[1])
        except Exception as e:
            print(f"[INPOS] Falha: {e}")
        return None

    def aguardar_chegada_por_tcp(self, alvo, tol_mm=2.0, timeout_s=120.0, ciclos_estaveis=5, exigir_inpos=False) -> bool:
        if self.modo_simulacao:
            return True
        t0 = time.time()
        stable = 0
        last = None
        while time.time() - t0 < timeout_s:
            atual = self._copy_tcp()
            dist = self._dist_xyz(atual, alvo)
            delta = self._dist_xyz(atual, last) if last is not None else 999999.0
            last = atual
            inpos_ok = True
            if exigir_inpos:
                inpos = self._is_in_pos()
                inpos_ok = True if inpos is None else bool(inpos)
            if dist <= tol_mm and delta <= 0.35 and inpos_ok:
                stable += 1
                if stable >= ciclos_estaveis:
                    return True
            else:
                stable = 0
            time.sleep(0.03)
        return False

    def ler_saida_digital(self):
        if self.modo_simulacao:
            return bool(self.saida_digital_ativa)
        try:
            ret = self.driver.get_digital_output(0, 1)
            if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0:
                return bool(ret[1])
        except Exception as e:
            print(f"[DO] Falha lendo saída digital: {e}")
        return None

    def set_saida_digital(self, ativo: bool, confirmar=False, timeout_s=2.0) -> bool:
        ativo = bool(ativo)
        if self.modo_simulacao:
            self.saida_digital_ativa = ativo
            self.diagnosticos["saida_digital_ativa"] = ativo
            return True
        try:
            ret = self.driver.set_digital_output(0, 1, ativo)
            if ret is not None and not self._ret_ok(ret):
                print(f"[DO] set_digital_output retornou: {ret}")
                return False
            if confirmar:
                t0 = time.time()
                stable = 0
                unavailable = False
                while time.time() - t0 < timeout_s:
                    val = self.ler_saida_digital()
                    if val is None:
                        unavailable = True
                        break
                    if val == ativo:
                        stable += 1
                        if stable >= 3:
                            break
                    else:
                        stable = 0
                    time.sleep(0.03)
                if unavailable:
                    time.sleep(0.12)
                elif stable < 3:
                    return False
            self.saida_digital_ativa = ativo
            self.diagnosticos["saida_digital_ativa"] = ativo
            return True
        except Exception as e:
            print(f"[DO] Falha setando saída digital: {e}")
            return False

    def parar_movimento_processo(self) -> bool:
        if self.modo_simulacao:
            return True
        try:
            self.driver.motion_abort()
            return True
        except Exception as e:
            print(f"[STOP] motion_abort falhou: {e}")
            return False

    def _linear_move_blocking(self, pose, vel, confirm=False, name="MoveL") -> bool:
        """MoveL bloqueante nativo da SDK, sem barreira extra por TCP.

        V13 tinha confirmação adicional via TCP/is_in_pos depois do movimento. Isso
        gerou delays grandes entre comandos no robô real. Aqui voltamos para o
        comportamento enxuto: se a chamada bloqueante da SDK retornou OK, seguimos.
        """
        pose = list(pose)
        if self.modo_simulacao:
            self._set_tcp(pose)
            time.sleep(0.05)
            return True
        try:
            ret = self.driver.linear_move(pose, 0, True, vel)
            if ret is not None and not self._ret_ok(ret):
                print(f"[{name}] retorno JAKA: {ret}")
                return False
            return True
        except Exception as e:
            print(f"[{name}] Falha: {e}")
            return False

    def _linear_move_nonblocking(self, pose, vel) -> bool:
        pose = list(pose)
        if self.modo_simulacao:
            self._set_tcp(pose)
            time.sleep(0.12)
            return True
        try:
            ret = self.driver.linear_move(pose, 0, False, vel)
            if ret is not None and not self._ret_ok(ret):
                print(f"[MoveL NB] retorno JAKA: {ret}")
                return False
            return True
        except Exception as e:
            print(f"[MoveL NB] Falha: {e}")
            return False

    def _circular_move_nonblocking(self, mid_pose, end_pose, vel) -> bool:
        if self.modo_simulacao:
            self._set_tcp(mid_pose)
            time.sleep(0.12)
            self._set_tcp(end_pose)
            time.sleep(0.12)
            return True
        try:
            # SDK JAKA: circular_move(end_pos, mid_pos, move_mode, is_block, speed, acc, tol)
            ret = self.driver.circular_move(list(end_pose), list(mid_pose), 0, False, vel, 800, 0)
            if ret is not None and not self._ret_ok(ret):
                print(f"[MoveC NB] retorno JAKA: {ret}")
                return False
            return True
        except Exception as e:
            print(f"[MoveC NB] Falha: {e}")
            return False

    def _fase_entrada(self, p1_pose, clearance):
        """Entrada comum e enxuta.

        Ordem mantida:
        DO off -> ponto alto -> P1 -> DO on.
        Sem barreiras extras por TCP para não introduzir delays artificiais.
        """
        p1 = list(p1_pose)
        p1_aprox = [a + b for a, b in zip(p1, clearance)]

        if not self.set_saida_digital(False, confirmar=False):
            return False, "Falha desligando saída digital antes da entrada."
        if not self._linear_move_blocking(p1_aprox, self.vel_aproximacao, confirm=False, name="Entrada alto"):
            return False, "Falha indo ao ponto alto de entrada."
        if not self._linear_move_blocking(p1, self.vel_aproximacao, confirm=False, name="Entrada P1"):
            return False, "Falha descendo ao primeiro ponto."
        if not self.set_saida_digital(True, confirmar=False):
            return False, "Falha ligando saída digital no primeiro ponto."
        return True, "Entrada concluída."

    def _executar_segmentos(self, plan) -> (bool, str):
        for seg in plan.segments:
            # Não usamos mais a DO como intertravamento rígido aqui por causa da variabilidade observada,
            # mas mantemos o estado publicado. O intertravamento real deve ser revisto com logs do robô.
            if seg.kind == SegmentKind.LINEAR:
                print(f"[PLANO] MoveL para ponto #{seg.target.index + 1}")
                if not self._linear_move_nonblocking(seg.target.pose, self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, f"Falha enviando MoveL para ponto #{seg.target.index + 1}."
            elif seg.kind == SegmentKind.CIRCULAR:
                print(f"[PLANO] MoveC start #{seg.start.index + 1} mid #{seg.mid.index + 1} end #{seg.end.index + 1}")
                if not self._circular_move_nonblocking(seg.mid.pose, seg.end.pose, self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, f"Falha enviando MoveC pontos #{seg.mid.index + 1}/#{seg.end.index + 1}."
            time.sleep(0.005)
        return True, "Trajetória principal enviada."

    def _fase_saida(self, last_pose, clearance):
        """Saída comum e enxuta: DO off -> subida sobre o último ponto."""
        last = list(last_pose)
        p_saida = [a + b for a, b in zip(last, clearance)]
        if not self.set_saida_digital(False, confirmar=False):
            self.parar_movimento_processo()
            return False, "Falha desligando saída digital no fim."
        if not self._linear_move_blocking(p_saida, self.vel_aproximacao, confirm=False, name="Saída"):
            return False, "Falha no movimento de saída."
        return True, "Saída concluída."

    def executar_trajetoria(self) -> str:
        with self._exec_lock:
            if self.executando_trajetoria:
                return "Já existe uma trajetória em execução."
            self.executando_trajetoria = True
            self.diagnosticos["executando_trajetoria"] = True

        try:
            pontos = self._get_pontos_snapshot()
            plan = TrajectoryPlanner.plan(pontos)
            if not plan.ok:
                msg = plan.message
                self._emit_exec_status(msg, "error")
                return msg

            clearance = [0, 0, 50, 0, 0, 0]
            ok, msg = self._fase_entrada(plan.entry.pose, clearance)
            if not ok:
                self._emit_exec_status(msg, "error")
                return msg

            ok, msg = self._executar_segmentos(plan)
            if not ok:
                self.set_saida_digital(False, confirmar=False)
                self._emit_exec_status(msg, "error")
                return msg

            if not self.aguardar_chegada_por_tcp(plan.exit.pose, tol_mm=2.0, timeout_s=120.0, ciclos_estaveis=2, exigir_inpos=False):
                self.set_saida_digital(False, confirmar=False)
                msg = "Timeout aguardando chegada ao último ponto. Saída digital desligada por segurança."
                self._emit_exec_status(msg, "error")
                return msg

            ok, msg = self._fase_saida(plan.exit.pose, clearance)
            if not ok:
                self._emit_exec_status(msg, "error")
                return msg

            self._emit_pontos()
            self._emit_exec_status("Trajetória física executada com sucesso.", "success")
            return "Trajetória física executada com sucesso."
        except Exception as e:
            try:
                self.parar_movimento_processo()
                self.set_saida_digital(False, confirmar=False)
            except Exception:
                pass
            msg = f"Erro na execução da trajetória: {e}"
            self._emit_exec_status(msg, "error")
            return msg
        finally:
            self.executando_trajetoria = False
            self.diagnosticos["executando_trajetoria"] = False

    # ------------------------------------------------------------------
    # Jog/manual
    # ------------------------------------------------------------------
    def enviar_jog(self, eixo, vel, coord):
        if self.modo_simulacao:
            if abs(vel) > 0.01:
                tcp = self._copy_tcp()
                tcp[eixo] += vel * 0.05
                self._set_tcp(tcp)
            return
        vel = round(float(vel), 2)
        if abs(vel) < self.DEADZONE:
            vel = 0.0
        if vel != self.last_sent_vels[eixo]:
            try:
                if vel != 0.0:
                    self.driver.jog(eixo, 2, coord, vel, 0)
                else:
                    self.driver.jog_stop(eixo)
                self.last_sent_vels[eixo] = vel
            except Exception as e:
                print(f"[JOG] Falha eixo {eixo}: {e}")

    def parar_grupo(self, eixos):
        for eixo in eixos:
            try:
                if not self.modo_simulacao:
                    self.driver.jog_stop(eixo)
            except Exception:
                pass
            self.last_sent_vels[eixo] = 0.0

    def _processar_joystick(self, joy):
        b7 = joy.get_button(7)
        if b7 == 1 and self.last_btns[7] == 0:
            self.angulo_operador -= math.pi / 2
        self.last_btns[7] = b7
        b8 = joy.get_button(8)
        if b8 == 1 and self.last_btns[8] == 0:
            self.angulo_operador += math.pi / 2
        self.last_btns[8] = b8

        cos_t = math.cos(self.angulo_operador)
        sin_t = math.sin(self.angulo_operador)
        btn_14, btn_13, btn_11, btn_12 = joy.get_button(14), joy.get_button(13), joy.get_button(11), joy.get_button(12)
        btn_0, btn_3, btn_1, btn_2 = joy.get_button(0), joy.get_button(3), joy.get_button(1), joy.get_button(2)
        axis_4, axis_5 = joy.get_axis(4), joy.get_axis(5)
        btn_9 = joy.get_button(9)
        btn_10 = joy.get_button(10) if joy.get_init() and joy.get_numbuttons() > 10 else 0

        pressing_linear = (btn_14 or btn_13 or btn_11 or btn_12)
        pressing_rot = (btn_0 or btn_3 or btn_1 or btn_2)
        pressing_z = (axis_4 > -0.9 or axis_5 > -0.9)
        pressing_rz = (btn_9 or btn_10)

        if self.grupo_ativo is None:
            if pressing_linear:
                self.grupo_ativo = "LINEAR"
            elif pressing_rot:
                self.grupo_ativo = "ROTAT_TCP"
            elif pressing_z:
                self.grupo_ativo = "EIXO_Z"
            elif pressing_rz:
                self.grupo_ativo = "EIXO_RZ"

        if self.grupo_ativo == "LINEAR" and not pressing_linear:
            self.parar_grupo([0, 1]); self.grupo_ativo = None
        elif self.grupo_ativo == "ROTAT_TCP" and not pressing_rot:
            self.parar_grupo([3, 4]); self.grupo_ativo = None
        elif self.grupo_ativo == "EIXO_Z" and not pressing_z:
            self.parar_grupo([2]); self.grupo_ativo = None
        elif self.grupo_ativo == "EIXO_RZ" and not pressing_rz:
            self.parar_grupo([5]); self.grupo_ativo = None

        if self.grupo_ativo == "LINEAR":
            vx_raw = float(btn_14 - btn_13) * self.MAX_SPD_LINEAR
            vy_raw = float(btn_11 - btn_12) * self.MAX_SPD_LINEAR
            self.enviar_jog(0, vx_raw * cos_t - vy_raw * sin_t, 0)
            self.enviar_jog(1, vx_raw * sin_t + vy_raw * cos_t, 0)
        elif self.grupo_ativo == "ROTAT_TCP":
            vrx_raw = float(btn_0 - btn_3) * self.MAX_SPD_ROTAT
            vry_raw = float(btn_1 - btn_2) * self.MAX_SPD_ROTAT
            self.enviar_jog(3, vrx_raw * cos_t - vry_raw * sin_t, 2)
            self.enviar_jog(4, vrx_raw * sin_t + vry_raw * cos_t, 2)
        elif self.grupo_ativo == "EIXO_Z":
            down = (axis_4 + 1.0) / 2.0 if axis_4 > -0.9 else 0.0
            up = (axis_5 + 1.0) / 2.0 if axis_5 > -0.9 else 0.0
            self.enviar_jog(2, (up - down) * self.MAX_SPD_LINEAR, 0)
        elif self.grupo_ativo == "EIXO_RZ":
            self.enviar_jog(5, float(btn_9 - btn_10) * self.MAX_SPD_ROTAT, 1)

        # salvar pontos
        b6 = joy.get_button(6)
        if b6 == 1 and self.last_btns[6] == 0:
            self.salvar_ponto_atual("L")
        self.last_btns[6] = b6
        b4 = joy.get_button(4)
        if b4 == 1 and self.last_btns[4] == 0:
            self.salvar_ponto_atual("C")
        self.last_btns[4] = b4

        # botão 5: toque apaga último, segurar 2s limpa tudo
        b5 = joy.get_button(5)
        if b5 == 1:
            if self.last_btns[5] == 0:
                self.b5_press_time = time.time(); self.b5_triggered_long_press = False
            elif self.b5_press_time and not self.b5_triggered_long_press and time.time() - self.b5_press_time >= 2.0:
                self._set_pontos([]); self.b5_triggered_long_press = True
        elif b5 == 0 and self.last_btns[5] == 1:
            if not self.b5_triggered_long_press:
                pts = self._get_pontos_snapshot()
                if pts:
                    pts.pop()
                    self._set_pontos(pts)
            self.b5_press_time = None; self.b5_triggered_long_press = False
        self.last_btns[5] = b5

        b15 = joy.get_button(15)
        if b15 == 1 and self.last_btns[15] == 0:
            backup = self._get_pontos_snapshot()
            def run():
                self.executar_trajetoria()
                self._set_pontos(backup)
            threading.Thread(target=run, daemon=True).start()
        self.last_btns[15] = b15

    def salvar_ponto_atual(self, tipo):
        tipo = str(tipo).upper()
        if tipo not in ("L", "C"):
            return False
        with self._state_lock:
            self.lista_pontos.append((tipo, self._copy_tcp()))
        self._emit_pontos()
        return True

    # ------------------------------------------------------------------
    # Telemetria
    # ------------------------------------------------------------------
    @staticmethod
    def _extract_jsons(buffer: str):
        objs = []
        start = None
        depth = 0
        in_str = False
        esc = False
        last_end = 0
        for i, ch in enumerate(buffer):
            if in_str:
                if esc:
                    esc = False
                elif ch == "\\":
                    esc = True
                elif ch == '"':
                    in_str = False
                continue
            if ch == '"':
                in_str = True
            elif ch == "{":
                if depth == 0:
                    start = i
                depth += 1
            elif ch == "}":
                if depth > 0:
                    depth -= 1
                    if depth == 0 and start is not None:
                        objs.append(buffer[start:i+1])
                        last_end = i + 1
                        start = None
        return objs, buffer[last_end:]

    def _start_tcp10000_monitor(self):
        if self.modo_simulacao or not self.ip_atual:
            return
        if self._tcp10000_thread and self._tcp10000_thread.is_alive():
            return
        self._tcp10000_stop.clear()
        self._tcp10000_thread = threading.Thread(target=self._tcp10000_loop, daemon=True)
        self._tcp10000_thread.start()

    def _apply_telemetry_updates(self, updates: Dict[str, Any], debug: Dict[str, Any]):
        if "tcp" in updates:
            self._set_tcp(updates.pop("tcp"))
        with self._state_lock:
            self._last_telemetry_debug = debug
            if updates:
                self.diagnosticos.update(updates)

    def _tcp10000_loop(self):
        while not self._tcp10000_stop.is_set():
            if not self.ip_atual or self.modo_simulacao:
                time.sleep(1.0); continue
            sock = None
            try:
                sock = socket.create_connection((self.ip_atual, 10000), timeout=3.0)
                sock.settimeout(1.0)
                buffer = ""
                while not self._tcp10000_stop.is_set() and not self.modo_simulacao:
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("socket fechado")
                    buffer += chunk.decode("utf-8", errors="ignore")
                    if len(buffer) > 2_000_000:
                        buffer = buffer[-200_000:]
                    raws, buffer = self._extract_jsons(buffer)
                    for raw in raws:
                        try:
                            payload = json.loads(raw)
                        except Exception:
                            continue
                        updates, debug = TelemetryParser.parse_tcp_payload(payload)
                        self._apply_telemetry_updates(updates, debug)
            except Exception as e:
                with self._state_lock:
                    self.diagnosticos["telemetria_status"] = f"TCP10000 indisponível/não validado: {e}"
                time.sleep(1.0)
            finally:
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass

    def _update_telemetry_low_freq(self):
        backend_uptime = int(time.time() - self.tempo_inicio_sistema)
        with self._state_lock:
            self.diagnosticos["backend_uptime_segundos"] = backend_uptime
            if not self.diagnosticos.get("robot_uptime_segundos"):
                self.diagnosticos["uptime_segundos"] = backend_uptime
                self.diagnosticos["uptime_fonte"] = "backend"
            self.diagnosticos["executando_trajetoria"] = self.executando_trajetoria
            self.diagnosticos["saida_digital_ativa"] = self.saida_digital_ativa

        if self.modo_simulacao:
            with self._state_lock:
                self.diagnosticos["telemetria_real"] = False
                self.diagnosticos["telemetria_origem"] = "simulação"
                self.diagnosticos["telemetria_status"] = "Modo simulação: sem dados reais do robô"
                self.diagnosticos["telemetria_parser"] = "none"
                self.diagnosticos["telemetria_confianca"] = 0
                # Valores zerados propositalmente; não fingimos corrente/temperatura real.
                self.diagnosticos["correntes"] = [0.0] * 6
                self.diagnosticos["temperaturas"] = [0.0] * 6
            return

        if not self.driver.has_robot():
            return

        try:
            ret = self.driver.get_tcp_position()
            if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0 and isinstance(ret[1], (list, tuple)):
                self._set_tcp(ret[1][:6])
        except Exception as e:
            print(f"[TCP] Falha get_tcp_position: {e}")

        # Se TCP10000 está atualizando dados reais, não sobrescrevemos por SDK.
        last = self.diagnosticos.get("ultima_telemetria_real_ts")
        if last and time.time() - float(last) < 2.0 and self.diagnosticos.get("telemetria_origem") == "TCP10000":
            return

        try:
            status = self.driver.get_robot_status()
            updates, debug = TelemetryParser.parse_robot_status(status)
            self._apply_telemetry_updates(updates, debug)
        except Exception as e:
            with self._state_lock:
                self.diagnosticos["telemetria_real"] = False
                self.diagnosticos["telemetria_status"] = f"Falha lendo SDK get_robot_status: {e}"

        # Expira selo real se nenhum parser validado atualizar recentemente.
        last = self.diagnosticos.get("ultima_telemetria_real_ts")
        if not last or time.time() - float(last) > 5.0:
            with self._state_lock:
                self.diagnosticos["telemetria_real"] = False
                if "validada" not in str(self.diagnosticos.get("telemetria_status", "")):
                    self.diagnosticos["telemetria_status"] = "Sem telemetria real validada nos últimos 5s"

    def get_telemetry_debug(self) -> Dict[str, Any]:
        with self._state_lock:
            return {
                "diagnosticos": copy.deepcopy(self.diagnosticos),
                "debug": copy.deepcopy(self._last_telemetry_debug),
                "sdk_carregado": self.sdk_carregado(),
                "modo_simulacao": self.modo_simulacao,
                "ip_atual": self.ip_atual,
            }

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------
    def iniciar_loop_controle(self):
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
            while True:
                try:
                    if joy is None and pygame.joystick.get_count() > 0:
                        try:
                            joy = pygame.joystick.Joystick(0)
                            joy.init()
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

                    if self.on_state_update:
                        self.on_state_update(self.snapshot_state())
                except Exception as e:
                    print(f"[LOOP] Erro inesperado: {e}")
                time.sleep(0.03)

        threading.Thread(target=loop, daemon=True).start()


adapter = RobotAdapter()
