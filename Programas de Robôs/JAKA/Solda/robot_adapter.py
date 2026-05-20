# robot_adapter.py
# Backend JAKA — IHM Solda Payback
# Versão híbrida V10:
#   - Entrada/saída com movimentos bloqueantes para sincronizar DO.
#   - Trajetória principal com movimentos não bloqueantes para manter telemetria TCP fluindo.
#   - Fim da trajetória aguardado pela posição TCP atualizada no loop de telemetria.
#   - Saída digital confirmada por leitura de DO antes/depois da trajetória.
#   - Watchdog interrompe movimento se a DO cair durante a trajetória principal.

import sys
import os
import time
import math
import pygame
import threading
import copy
import random
import socket
import json

SDK_DIR = r"C:\jakaAPI_V2.1.7stable\SDK2.1.7\Windows\python3\x64"

try:
    sys.path.insert(0, SDK_DIR)
    os.add_dll_directory(SDK_DIR)
    import jkrc
    _SDK_DISPONIVEL = True
except Exception as e:
    _SDK_DISPONIVEL = False
    print(f"[AVISO] SDK JAKA não carregado ({e}). Modo simulação ativo.")


class RobotAdapter:
    def __init__(self):
        self.robot = None
        self.modo_simulacao = not _SDK_DISPONIVEL
        self.ip_atual = None

        # Parâmetros de Controle
        self.MAX_SPD_LINEAR = 120.0
        self.MAX_SPD_ROTAT = 6.0
        self.DEADZONE = 0.2

        # Estado dinâmico do robô
        self.posicao_atual_tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lista_pontos = []
        self.vel_reproducao = 15.0
        self.vel_aproximacao = 150.0

        # Processo / execução
        self.executando_trajetoria = False
        self.saida_digital_ativa = False
        self._last_wait_reason = ""
        self._last_do_read_value = None
        self._exec_lock = threading.Lock()
        self._state_lock = threading.RLock()

        # Diagnóstico / telemetria
        self.tempo_inicio_sistema = time.time()
        self.diagnosticos = {
            # Dados principais exibidos na HMI
            "temperaturas": [32.5, 33.1, 31.8, 34.2, 32.9, 33.5],
            "correntes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tensoes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "torques": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],

            # Uptime: preferimos tempo interno do controlador/juntas quando disponível;
            # se não vier pela JAKA, usamos o uptime do backend, que persiste ao refresh da página.
            "uptime_segundos": 0,
            "backend_uptime_segundos": 0,
            "robot_uptime_segundos": None,
            "uptime_fonte": "backend",

            # Estado do controlador / processo
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
            "telemetria_status": "Sem telemetria real confirmada",
            "ultima_telemetria_real_ts": None,
        }
        self.contador_ciclos_telemetria = 0
        self._monitor_tcp_thread = None
        self._monitor_tcp_stop = threading.Event()
        self._monitor_tcp_connected = False

        # Debounce / retenção do botão 5
        self.b5_press_time = None
        self.b5_triggered_long_press = False

        # Sistema de coordenadas do operador
        self.angulo_operador = 0.0

        # Histórico de controle para debounce / edge detection
        self.last_sent_vels = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
        self.last_btns = {4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 15: 0}

        # Trava de exclusividade de grupo de movimento manual
        self.grupo_ativo = None

        # Callbacks para notificar o app.py via Socket.IO
        self.on_state_update = None
        self.on_point_saved = None

    # ----------------------------------------------------------------------
    # Utilidades SDK / estado
    # ----------------------------------------------------------------------
    def sdk_carregado(self):
        return _SDK_DISPONIVEL

    @staticmethod
    def _ret_ok(ret):
        """Aceita retornos JAKA no formato 0, [0, ...] ou (0, ...)."""
        if ret == 0:
            return True
        if isinstance(ret, (list, tuple)) and len(ret) > 0 and ret[0] == 0:
            return True
        return False

    def _copy_tcp(self):
        with self._state_lock:
            return list(self.posicao_atual_tcp)

    def _set_tcp(self, pose):
        with self._state_lock:
            self.posicao_atual_tcp = list(pose)

    def _get_pontos_snapshot(self):
        with self._state_lock:
            return copy.deepcopy(self.lista_pontos)

    def _set_pontos(self, pontos):
        with self._state_lock:
            self.lista_pontos = copy.deepcopy(pontos)
        if self.on_point_saved:
            self.on_point_saved(self._get_pontos_snapshot())

    # ----------------------------------------------------------------------
    # Conexão
    # ----------------------------------------------------------------------
    def conectar(self, ip="192.168.0.200"):
        if not self.sdk_carregado():
            self.modo_simulacao = True
            print("[SIMULAÇÃO] SDK JAKA indisponível; conexão física ignorada.")
            return False

        try:
            print(f"[ROBÔ] Tentando conectar ao controlador JAKA em {ip}...")
            self.robot = jkrc.RC(ip)

            login_ret = self.robot.login()
            if not self._ret_ok(login_ret):
                print(f"[ERRO DE CONEXÃO] login() falhou. Retorno JAKA: {login_ret}")
                self.modo_simulacao = True
                return False

            # Algumas versões retornam código; outras só executam.
            try:
                power_ret = self.robot.power_on()
                if power_ret is not None and not self._ret_ok(power_ret):
                    print(f"[AVISO] power_on() retornou: {power_ret}")
            except Exception as e:
                print(f"[AVISO] Falha em power_on(): {e}")

            try:
                enable_ret = self.robot.enable_robot()
                if enable_ret is not None and not self._ret_ok(enable_ret):
                    print(f"[AVISO] enable_robot() retornou: {enable_ret}")
            except Exception as e:
                print(f"[AVISO] Falha em enable_robot(): {e}")

            # Solicita ao controlador que atualize os dados de status em intervalo menor.
            # A SDK V2.1.7 documenta set_status_data_update_time_interval(ms).
            try:
                if hasattr(self.robot, "set_status_data_update_time_interval"):
                    self.robot.set_status_data_update_time_interval(100)
                    print("[ROBÔ] Intervalo de atualização de status configurado para 100 ms.")
            except Exception as e:
                print(f"[AVISO] Não foi possível configurar intervalo de status: {e}")

            self.ip_atual = ip
            self.modo_simulacao = False
            self._start_monitor_tcp10000()
            print(f"[ROBÔ] Conectado e habilitado em {ip}.")
            return True

        except Exception as e:
            print(f"[ERRO] Falha ao conectar no robô: {e}")
            self.modo_simulacao = True
            return False


    # ----------------------------------------------------------------------
    # Telemetria TCP 10000 — fonte preferencial para corrente/temperatura
    # ----------------------------------------------------------------------
    def _start_monitor_tcp10000(self):
        """Inicia leitura independente da porta 10000 do controlador JAKA.

        A documentação TCP separa comandos na porta 10001 e monitoramento na porta
        10000. Usar essa porta evita confundir dados simulados/SDK e aproxima a HMI
        do que o CoboPi/APP mostra na tela de monitoramento.
        """
        if self.modo_simulacao or not self.ip_atual:
            return
        if self._monitor_tcp_thread and self._monitor_tcp_thread.is_alive():
            return
        self._monitor_tcp_stop.clear()
        self._monitor_tcp_thread = threading.Thread(target=self._monitor_tcp10000_loop, daemon=True)
        self._monitor_tcp_thread.start()

    @staticmethod
    def _extrair_jsons_do_buffer(buffer):
        """Extrai objetos JSON de um buffer sem depender de quebra de linha.

        A porta 10000 pode enviar JSONs concatenados ou com quebras. Este parser por
        balanceamento de chaves é mais tolerante que split('\n').
        """
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
                elif ch == '\\':
                    esc = True
                elif ch == '"':
                    in_str = False
                continue
            if ch == '"':
                in_str = True
            elif ch == '{':
                if depth == 0:
                    start = i
                depth += 1
            elif ch == '}':
                if depth > 0:
                    depth -= 1
                    if depth == 0 and start is not None:
                        objs.append(buffer[start:i+1])
                        last_end = i + 1
                        start = None
        return objs, buffer[last_end:]

    def _monitor_tcp10000_loop(self):
        while not self._monitor_tcp_stop.is_set():
            ip = self.ip_atual
            if not ip or self.modo_simulacao:
                time.sleep(1.0)
                continue
            sock = None
            try:
                print(f"[TCP10000] Conectando em {ip}:10000 para telemetria real...")
                sock = socket.create_connection((ip, 10000), timeout=3.0)
                sock.settimeout(1.0)
                self._monitor_tcp_connected = True
                buffer = ""
                while not self._monitor_tcp_stop.is_set() and not self.modo_simulacao:
                    try:
                        chunk = sock.recv(8192)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("socket fechado pelo controlador")
                    text = chunk.decode("utf-8", errors="ignore")
                    buffer += text
                    # proteção contra crescimento infinito se chegar lixo
                    if len(buffer) > 2_000_000:
                        buffer = buffer[-200_000:]
                    jsons, buffer = self._extrair_jsons_do_buffer(buffer)
                    for raw in jsons:
                        try:
                            payload = json.loads(raw)
                        except Exception:
                            continue
                        self._processar_payload_tcp10000(payload)
            except Exception as e:
                self._monitor_tcp_connected = False
                if not self.modo_simulacao:
                    self.diagnosticos["telemetria_status"] = f"TCP10000 indisponível: {e}"
                time.sleep(1.0)
            finally:
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass
                self._monitor_tcp_connected = False

    def _processar_payload_tcp10000(self, payload):
        """Interpreta payloads vindos da porta 10000.

        Formatos observados/documentados variam por versão. Procuramos campos em
        root, data e result para tolerar envelopes diferentes.
        """
        if not isinstance(payload, dict):
            return
        candidates = [payload]
        for key in ("data", "result", "res", "state"):
            val = payload.get(key)
            if isinstance(val, dict):
                candidates.append(val)

        monitor = None
        actual_position = None
        emergency_stop = None
        protective_stop = None
        enabled = None
        inpos = None
        dout = None

        for obj in candidates:
            monitor = monitor or obj.get("monitor_data") or obj.get("monitorData") or obj.get("robot_monitor_data")
            actual_position = actual_position or obj.get("actual_position") or obj.get("cart_position") or obj.get("tcp")
            emergency_stop = obj.get("emergency_stop", emergency_stop)
            protective_stop = obj.get("protective_stop", protective_stop)
            enabled = obj.get("enabled", enabled)
            inpos = obj.get("inpos", inpos)
            dout = obj.get("dout", dout)

        if isinstance(actual_position, (list, tuple)) and len(actual_position) >= 6:
            self._set_tcp(actual_position[:6])
        if emergency_stop is not None:
            self.diagnosticos["status_emergencia"] = self._to_bool(emergency_stop)
        if protective_stop is not None:
            self.diagnosticos["protective_stop"] = self._to_bool(protective_stop)
        if enabled is not None:
            self.diagnosticos["enabled"] = self._to_bool(enabled)
        if inpos is not None:
            self.diagnosticos["inpos"] = self._to_bool(inpos)
        if isinstance(dout, (list, tuple)) and len(dout) > 1:
            try:
                self.saida_digital_ativa = bool(dout[1])
                self.diagnosticos["saida_digital_ativa"] = self.saida_digital_ativa
            except Exception:
                pass

        if monitor is not None:
            if self._extrair_robot_monitor_data(monitor, origem="tcp10000"):
                self.diagnosticos["telemetria_real"] = True
                self.diagnosticos["telemetria_origem"] = "TCP 10000"
                self.diagnosticos["telemetria_status"] = "Telemetria real via porta 10000"
                self.diagnosticos["ultima_telemetria_real_ts"] = time.time()

    # ----------------------------------------------------------------------
    # I/O
    # ----------------------------------------------------------------------
    def ler_saida_digital(self):
        """Lê a saída digital usada pela ferramenta/processo.

        Retorna True/False quando a leitura é possível; retorna None quando a binding
        da SDK não responder ou quando a leitura falhar. O índice/tipo foram mantidos
        iguais ao código validado: set_digital_output(0, 1, ...).
        """
        if self.modo_simulacao:
            return bool(self.saida_digital_ativa)

        if not self.robot:
            return None

        try:
            if not hasattr(self.robot, "get_digital_output"):
                return None
            ret = self.robot.get_digital_output(0, 1)
            if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0:
                val = bool(ret[1])
                self._last_do_read_value = val
                return val
            return None
        except Exception as e:
            print(f"[DO] Falha ao ler saída digital: {e}")
            return None

    def aguardar_saida_digital(self, esperado: bool, timeout_s=2.0, ciclos_estaveis=3):
        """Aguarda confirmação real da DO via get_digital_output.

        Se a SDK não permitir leitura de DO, usa um pequeno dwell e considera o comando
        aceito para não quebrar versões da binding que não exponham get_digital_output.
        """
        esperado = bool(esperado)
        t0 = time.time()
        stable = 0
        leitura_indisponivel = False

        while time.time() - t0 < timeout_s:
            atual = self.ler_saida_digital()

            if atual is None:
                leitura_indisponivel = True
                break

            if atual == esperado:
                stable += 1
                if stable >= ciclos_estaveis:
                    return True
            else:
                stable = 0

            time.sleep(0.03)

        if leitura_indisponivel:
            # Fallback pragmático: algumas versões do jkrc não expõem leitura de DO.
            # Mantemos a sequência, mas com dwell para reduzir corrida de I/O.
            time.sleep(0.12)
            return True

        return False

    def set_saida_digital(self, ativo: bool, confirmar=True, timeout_s=2.0):
        """Liga/desliga a saída digital da ferramenta de processo.

        Mantive a assinatura usada no código original: set_digital_output(0, 1, bool).
        Quando confirmar=True, a função só retorna True depois da leitura da DO confirmar
        o estado esperado, ou depois do fallback controlado quando a leitura não existe.
        """
        ativo = bool(ativo)
        if self.modo_simulacao:
            self.saida_digital_ativa = ativo
            self.diagnosticos["saida_digital_ativa"] = ativo
            print(f"[SIMULAÇÃO] DO ferramenta = {ativo}")
            return True

        try:
            ret = self.robot.set_digital_output(0, 1, ativo)
            if ret is not None and not self._ret_ok(ret):
                print(f"[AVISO] set_digital_output retornou: {ret}")
                return False

            if confirmar:
                ok = self.aguardar_saida_digital(ativo, timeout_s=timeout_s, ciclos_estaveis=3)
                if not ok:
                    print(f"[ERRO DO] DO não confirmou estado esperado: {ativo}")
                    return False

            self.saida_digital_ativa = ativo
            self.diagnosticos["saida_digital_ativa"] = ativo
            return True
        except Exception as e:
            print(f"[ERRO DO] Falha ao setar saída digital: {e}")
            return False

    def parar_movimento_processo(self):
        """Interrompe movimento externo enviado pela SDK, sem usar controle de programa.

        Importante: aqui NÃO usamos stop_program/program_abort/pause_program porque
        estes comandos se referem ao runtime de programas do pendant/JAKA APP. Como a
        trajetória desta IHM é enviada externamente via SDK/Ethernet, a parada correta
        para esta camada é motion_abort(). Mantemos stop_move apenas como fallback
        defensivo se a binding Python da versão instalada expuser esse método.
        """
        if self.modo_simulacao or not self.robot:
            return True

        # 1) Caminho documentado no SDK Python V2.1.7: motion_abort().
        fn = getattr(self.robot, "motion_abort", None)
        if callable(fn):
            try:
                ret = fn()
                print(f"[STOP] motion_abort() chamado. Retorno: {ret}")
                return True
            except Exception as e:
                print(f"[STOP] Falha em motion_abort(): {e}")

        # 2) Fallback TCP/algumas bindings: stop_move, se existir.
        fn = getattr(self.robot, "stop_move", None)
        if callable(fn):
            try:
                ret = fn()
                print(f"[STOP] stop_move() chamado como fallback. Retorno: {ret}")
                return True
            except Exception as e:
                print(f"[STOP] Falha em stop_move(): {e}")

        # 3) Último recurso: parar jog em todos os eixos, não é ideal para MoveL/MoveC.
        try:
            self.parar_grupo([-1])
            return True
        except Exception:
            try:
                self.parar_grupo([0, 1, 2, 3, 4, 5])
                return True
            except Exception:
                return False

    # ----------------------------------------------------------------------
    # Jog/manual
    # ----------------------------------------------------------------------
    def enviar_jog(self, eixo, vel, coord):
        if self.modo_simulacao:
            if abs(vel) > 0.01:
                atual = self._copy_tcp()
                atual[eixo] += (vel * 0.05)
                self._set_tcp(atual)
            return

        vel = round(float(vel), 2)
        if abs(vel) < self.DEADZONE:
            vel = 0.0

        if vel != self.last_sent_vels[eixo]:
            try:
                if vel != 0.0:
                    # Mantido como no backend funcional atual.
                    # Não alterei jog_mode para evitar mudar comportamento validado em campo.
                    self.robot.jog(eixo, 2, coord, vel, 0)
                else:
                    self.robot.jog_stop(eixo)
                self.last_sent_vels[eixo] = vel
            except Exception as e:
                print(f"[ERRO JOG] Eixo {eixo}: {e}")

    def parar_grupo(self, eixos_grupo):
        if self.modo_simulacao:
            for eixo in eixos_grupo:
                self.last_sent_vels[eixo] = 0.0
            return

        for eixo in eixos_grupo:
            try:
                self.robot.jog_stop(eixo)
            except Exception:
                pass
            self.last_sent_vels[eixo] = 0.0

    # ----------------------------------------------------------------------
    # Esperas / trajetória
    # ----------------------------------------------------------------------
    @staticmethod
    def _dist_xyz(a, b):
        return math.sqrt(
            (float(a[0]) - float(b[0])) ** 2 +
            (float(a[1]) - float(b[1])) ** 2 +
            (float(a[2]) - float(b[2])) ** 2
        )

    def _is_in_pos(self):
        """Consulta rápida de estado in-position via SDK.

        Retorna True/False quando a SDK responde; retorna None se a chamada não estiver
        disponível ou falhar. Não usa isso como única fonte de verdade; a barreira física
        combina in_pos + distância TCP + estabilidade.
        """
        if self.modo_simulacao:
            return True
        if not self.robot or not hasattr(self.robot, "is_in_pos"):
            return None
        try:
            ret = self.robot.is_in_pos()
            if isinstance(ret, (list, tuple)) and len(ret) >= 2 and ret[0] == 0:
                return bool(ret[1])
            return None
        except Exception as e:
            print(f"[INPOS] Falha ao consultar is_in_pos(): {e}")
            return None

    def aguardar_chegada_por_tcp(
        self,
        alvo,
        tol_mm=2.0,
        timeout_s=120.0,
        ciclos_estaveis=5,
        movimento_tol_mm=0.35,
        expected_do=None,
        abort_on_do_mismatch=False,
        do_check_period_s=0.10,
        do_mismatch_cycles=2,
        exigir_inpos=False,
    ):
        """Aguarda chegada usando a telemetria TCP já publicada no adapter.

        A regra é propositalmente conservadora:
        - TCP perto do alvo;
        - TCP estável por N ciclos;
        - opcionalmente in_pos=True;
        - opcionalmente DO no estado esperado durante todo o período.

        Se a DO cair durante a trajetória principal, aborta movimento com motion_abort().
        """
        t0 = time.time()
        stable = 0
        mismatch = 0
        last_do_check = 0.0
        last_atual = None
        alvo = list(alvo)
        self._last_wait_reason = ""

        while time.time() - t0 < timeout_s:
            atual = self._copy_tcp()
            dist = self._dist_xyz(atual, alvo)
            delta = self._dist_xyz(atual, last_atual) if last_atual is not None else 999999.0
            last_atual = atual

            if expected_do is not None and (time.time() - last_do_check) >= do_check_period_s:
                last_do_check = time.time()
                do_val = self.ler_saida_digital()
                if do_val is not None and do_val != bool(expected_do):
                    mismatch += 1
                    if abort_on_do_mismatch and mismatch >= do_mismatch_cycles:
                        self._last_wait_reason = (
                            f"Saída digital saiu do estado esperado ({expected_do}) durante o movimento."
                        )
                        self.parar_movimento_processo()
                        return False
                else:
                    mismatch = 0

            inpos_ok = True
            if exigir_inpos:
                inpos = self._is_in_pos()
                # Se a SDK não responder, não travamos a aplicação; seguimos pela barreira TCP.
                inpos_ok = True if inpos is None else bool(inpos)

            if dist <= tol_mm and delta <= movimento_tol_mm and inpos_ok:
                stable += 1
                if stable >= ciclos_estaveis:
                    return True
            else:
                stable = 0

            time.sleep(0.03)

        self._last_wait_reason = f"Timeout aguardando alvo XYZ {alvo[:3]}."
        return False

    def _barreira_fisica_no_ponto(self, pose, nome="ponto", tol_mm=1.0, timeout_s=60.0, ciclos_estaveis=10):
        """Barreira conservadora antes de qualquer transição de processo.

        Esta função é usada antes de ligar/desligar processo. Ela evita confiar apenas
        no retorno de linear_move(..., is_block=True), que no robô real mostrou
        variabilidade. A liberação só ocorre após TCP estabilizado perto do alvo e,
        quando disponível, is_in_pos=True.
        """
        if self.modo_simulacao:
            return True

        ok = self.aguardar_chegada_por_tcp(
            pose,
            tol_mm=tol_mm,
            timeout_s=timeout_s,
            ciclos_estaveis=ciclos_estaveis,
            movimento_tol_mm=0.25,
            exigir_inpos=True,
        )
        if not ok:
            print(f"[BARREIRA] Timeout em {nome}: alvo={pose[:3]}, tcp={self._copy_tcp()[:3]}")
            return False

        # Dwell pequeno, mas importante: separa o último frame de movimento do comando de DO.
        time.sleep(0.12)
        return True

    def _linear_move_bloqueante(self, pose, vel):
        if self.modo_simulacao:
            self._set_tcp(pose)
            time.sleep(0.2)
            return True

        ret = self.robot.linear_move(pose, 0, True, vel)
        if ret is not None and not self._ret_ok(ret):
            print(f"[AVISO] linear_move bloqueante retornou: {ret}")
            return False
        return True

    def _linear_move_bloqueante_confirmado(self, pose, vel, tol_mm=1.0, timeout_s=60.0, ciclos_estaveis=10, nome="MoveL"):
        self._linear_move_bloqueante(pose, vel)
        return self._barreira_fisica_no_ponto(
            pose,
            nome=nome,
            tol_mm=tol_mm,
            timeout_s=timeout_s,
            ciclos_estaveis=ciclos_estaveis,
        )

    def _linear_move_nao_bloqueante(self, pose, vel):
        if self.modo_simulacao:
            self._set_tcp(pose)
            time.sleep(0.15)
            return True

        ret = self.robot.linear_move(pose, 0, False, vel)
        if ret is not None and not self._ret_ok(ret):
            print(f"[AVISO] linear_move não bloqueante retornou: {ret}")
            return False
        return True

    def _circular_move_nao_bloqueante(self, end_pose, mid_pose, vel, acc=800, tol=0):
        if self.modo_simulacao:
            self._set_tcp(mid_pose)
            time.sleep(0.15)
            self._set_tcp(end_pose)
            time.sleep(0.15)
            return True

        ret = self.robot.circular_move(end_pose, mid_pose, 0, False, vel, acc, tol)
        if ret is not None and not self._ret_ok(ret):
            print(f"[AVISO] circular_move não bloqueante retornou: {ret}")
            return False
        return True

    def _fase_entrada_processo(self, p1_real, h_clearance):
        """Entrada comum para trajetória linear e circular.

        Ordem física obrigatória:
        1. DO off confirmada;
        2. MoveL bloqueante até ponto alto de entrada;
        3. MoveL bloqueante de descida até P1;
        4. barreira física no P1;
        5. DO on confirmada;
        6. só então a trajetória principal pode começar.
        """
        p1_real = list(p1_real)
        p1_aprox = [a + b for a, b in zip(p1_real, h_clearance)]

        print(f"[SEQ] Entrada: garantindo DO OFF.")
        if not self.set_saida_digital(False, confirmar=True, timeout_s=2.0):
            return False, "Falha ao garantir saída digital desligada antes da aproximação."

        print(f"[SEQ] Entrada: indo para ponto alto sobre P1 {p1_aprox[:3]}.")
        if not self._linear_move_bloqueante_confirmado(
            p1_aprox,
            self.vel_aproximacao,
            tol_mm=1.5,
            timeout_s=60.0,
            ciclos_estaveis=8,
            nome="ponto alto de entrada",
        ):
            return False, "Timeout na aproximação ao ponto de entrada. Saída digital mantida desligada."

        print(f"[SEQ] Entrada: descendo até P1 {p1_real[:3]}.")
        if not self._linear_move_bloqueante_confirmado(
            p1_real,
            self.vel_aproximacao,
            tol_mm=1.0,
            timeout_s=60.0,
            ciclos_estaveis=12,
            nome="P1 antes de DO ON",
        ):
            return False, "Timeout na descida até o primeiro ponto. Saída digital mantida desligada."

        print("[SEQ] Entrada: P1 confirmado; ligando DO.")
        if not self.set_saida_digital(True, confirmar=True, timeout_s=2.0):
            return False, "Falha ao confirmar saída digital ligada no primeiro ponto. Trajetória principal não iniciada."

        # Barreira solicitada: a trajetória principal só começa depois de DO ativa.
        if not self.aguardar_saida_digital(True, timeout_s=2.0, ciclos_estaveis=3):
            return False, "Saída digital não permaneceu ligada antes da trajetória principal."

        print("[SEQ] Entrada concluída: DO ON confirmada. Iniciando trajetória principal.")
        return True, "Entrada concluída."

    def _validar_e_planejar_trajetoria(self, pontos):
        """Valida e converte pontos L/C em segmentos executáveis.

        Semântica adotada para evitar o bug de mistura linear/circular:
        - A trajetória completa precisa de pelo menos 2 pontos.
        - Pontos L são alvos lineares individuais.
        - Um bloco de pontos C representa movimento circular.
        - Cada bloco C precisa ter 3, 5, 7... pontos: start, mid, end, mid, end...
        - Se um bloco C começa depois de um bloco L, o primeiro C é o início do arco;
          portanto o robô faz um MoveL até esse primeiro C antes do MoveC.

        Isso impede que o primeiro ponto circular seja usado indevidamente como mid_pos.
        """
        if len(pontos) < 2:
            return None, "Necessário ao menos 2 pontos para iniciar."

        norm = []
        for idx, p in enumerate(pontos):
            if not isinstance(p, (list, tuple)) or len(p) < 2:
                return None, f"Ponto #{idx+1} inválido."
            tipo = str(p[0]).upper()
            if tipo not in ("L", "C"):
                return None, f"Tipo do ponto #{idx+1} inválido: {tipo}. Use L ou C."
            pose = list(p[1])
            if len(pose) < 6:
                return None, f"Pose do ponto #{idx+1} inválida: esperado [x,y,z,rx,ry,rz]."
            norm.append((tipo, pose))

        segmentos = []
        i = 0
        n = len(norm)
        while i < n:
            tipo = norm[i][0]

            if tipo == "L":
                start = i
                while i < n and norm[i][0] == "L":
                    i += 1
                for idx in range(start, i):
                    # O primeiro ponto da trajetória já foi consumido pela fase de entrada.
                    if idx == 0:
                        continue
                    segmentos.append(("L", norm[idx][1], idx))
                continue

            # Bloco circular C...
            start = i
            while i < n and norm[i][0] == "C":
                i += 1
            count = i - start

            if count < 3:
                faltam = 3 - count
                return None, (
                    f"Bloco circular iniciado no ponto #{start+1} tem {count} ponto(s) C. "
                    f"Adicione mais {faltam} ponto(s) circular(es): início, passagem e fim."
                )
            if count % 2 == 0:
                return None, (
                    f"Bloco circular iniciado no ponto #{start+1} tem {count} pontos C. "
                    "Use 3, 5, 7... pontos C: início, passagem/fim, passagem/fim."
                )

            # Se o bloco C começa depois da trajetória já estar em outro ponto, o primeiro C
            # é o início do arco e precisa ser atingido linearmente antes do MoveC.
            if start > 0:
                segmentos.append(("L", norm[start][1], start))

            # Se start == 0, a fase de entrada já colocou o TCP no primeiro C.
            j = start + 1
            while j + 1 < i:
                mid_pose = norm[j][1]
                end_pose = norm[j + 1][1]
                segmentos.append(("C", mid_pose, end_pose, j, j + 1))
                j += 2

        if not segmentos:
            return None, "Trajetória sem segmento executável. Adicione pelo menos um destino após o ponto inicial."

        return segmentos, "OK"

    def _fase_trajetoria_principal(self, pontos, segmentos=None):
        """Executa segmentos já validados, sem I/O no meio."""
        if segmentos is None:
            segmentos, msg = self._validar_e_planejar_trajetoria(pontos)
            if segmentos is None:
                return False, msg

        for seg in segmentos:
            do_val = self.ler_saida_digital()
            if do_val is not None and not do_val:
                self.parar_movimento_processo()
                return False, "Saída digital desligada antes/durante a trajetória principal. Movimento abortado."

            if seg[0] == "L":
                _, pose, idx = seg
                print(f"[SEQ] Trajetória: MoveL para ponto #{idx+1} {list(pose)[:3]}.")
                if not self._linear_move_nao_bloqueante(list(pose), self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, f"Falha ao enviar MoveL para ponto #{idx+1}. Movimento abortado."

            elif seg[0] == "C":
                _, mid_pose, end_pose, mid_idx, end_idx = seg
                print(f"[SEQ] Trajetória: MoveC mid ponto #{mid_idx+1} {mid_pose[:3]} end ponto #{end_idx+1} {end_pose[:3]}.")
                if not self._circular_move_nao_bloqueante(end_pos=list(end_pose), mid_pose=list(mid_pose), vel=self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, f"Falha ao enviar MoveC pontos #{mid_idx+1}/#{end_idx+1}. Movimento abortado."
            else:
                return False, f"Segmento desconhecido: {seg[0]}"

            time.sleep(0.02)

        return True, "Trajetória principal enviada."

    def _fase_saida_processo(self, ultimo_real, h_clearance):
        """Saída comum para trajetória linear e circular.

        Ordem física obrigatória:
        1. DO off confirmada;
        2. só então MoveL bloqueante para ponto alto sobre o último ponto.
        """
        ultimo_real = list(ultimo_real)
        p_saida_aprox = [a + b for a, b in zip(ultimo_real, h_clearance)]

        print("[SEQ] Saída: desligando DO.")
        if not self.set_saida_digital(False, confirmar=True, timeout_s=2.0):
            self.parar_movimento_processo()
            return False, "Falha ao confirmar saída digital desligada no fim. Movimento de saída bloqueado."

        # Barreira solicitada: só sobe depois de DO realmente off.
        if not self.aguardar_saida_digital(False, timeout_s=2.0, ciclos_estaveis=3):
            self.parar_movimento_processo()
            return False, "Saída digital não permaneceu desligada antes do movimento de saída."

        print(f"[SEQ] Saída: subindo para ponto alto sobre último ponto {p_saida_aprox[:3]}.")
        if not self._linear_move_bloqueante_confirmado(
            p_saida_aprox,
            self.vel_aproximacao,
            tol_mm=2.0,
            timeout_s=60.0,
            ciclos_estaveis=8,
            nome="ponto alto de saída",
        ):
            return False, "Timeout no movimento de saída após desligar DO."

        print("[SEQ] Saída concluída.")
        return True, "Saída concluída."

    def executar_trajetoria(self):
        """Executa trajetória com fases comuns de entrada/miolo/saída.

        A mesma entrada e a mesma saída são usadas para trajetórias lineares e circulares.
        Isso remove a falsa diferença entre os dois casos: ambos precisam da mesma
        transação de processo antes e depois da trajetória principal.
        """
        with self._exec_lock:
            if self.executando_trajetoria:
                return "Já existe uma trajetória em execução."
            self.executando_trajetoria = True
            self.diagnosticos["executando_trajetoria"] = True

        pontos = self._get_pontos_snapshot()

        try:
            segmentos, valid_msg = self._validar_e_planejar_trajetoria(pontos)
            if segmentos is None:
                return valid_msg

            h_clearance = [0, 0, 50, 0, 0, 0]
            p1_real = list(pontos[0][1])
            ultimo_real = list(pontos[-1][1])

            if self.modo_simulacao:
                print("[SIMULAÇÃO] Executando trajetória híbrida V10.")

            ok, msg = self._fase_entrada_processo(p1_real, h_clearance)
            if not ok:
                return msg

            ok, msg = self._fase_trajetoria_principal(pontos, segmentos=segmentos)
            if not ok:
                self.set_saida_digital(False, confirmar=False)
                return msg

            # Aguarda fim físico da trajetória principal sem usar movimento/programa bloqueante.
            # Durante a espera, a DO é monitorada; se cair, motion_abort() é chamado.
            print(f"[SEQ] Aguardando chegada ao último ponto {ultimo_real[:3]} com DO ON supervisionada.")
            chegou = self.aguardar_chegada_por_tcp(
                ultimo_real,
                tol_mm=2.0,
                timeout_s=120.0,
                ciclos_estaveis=8,
                movimento_tol_mm=0.30,
                expected_do=True,
                abort_on_do_mismatch=True,
                do_check_period_s=0.10,
                do_mismatch_cycles=2,
                exigir_inpos=True,
            )

            if not chegou:
                self.set_saida_digital(False, confirmar=False)
                detalhe = self._last_wait_reason or "Timeout aguardando chegada ao último ponto."
                return f"{detalhe} Saída digital desligada por segurança."

            ok, msg = self._fase_saida_processo(ultimo_real, h_clearance)
            if not ok:
                return msg

            if self.on_point_saved:
                self.on_point_saved(pontos)

            return "Trajetória física executada com sucesso!"

        except Exception as e:
            try:
                self.parar_movimento_processo()
                self.set_saida_digital(False, confirmar=False)
            except Exception:
                pass
            return f"Erro na execução da trajetória: {e}"

        finally:
            self.executando_trajetoria = False
            self.diagnosticos["executando_trajetoria"] = False

    # ----------------------------------------------------------------------
    # Loop joystick + telemetria
    # ----------------------------------------------------------------------
    def iniciar_loop_controle(self):
        pygame.init()
        pygame.joystick.init()
        try:
            joy = pygame.joystick.Joystick(0)
            joy.init()
            print(f"[JOYSTICK] {joy.get_name()} inicializado com sucesso!")
        except Exception:
            print("[AVISO] Nenhum Joystick encontrado. Aguardando conexão física...")
            joy = None

        def loop():
            nonlocal joy
            while True:
                try:
                    # 1. Uptime / divisores.
                    # O uptime fica no backend/robô, não na página HTML.
                    backend_uptime = int(time.time() - self.tempo_inicio_sistema)
                    self.diagnosticos["backend_uptime_segundos"] = backend_uptime
                    if self.diagnosticos.get("robot_uptime_segundos") is None:
                        self.diagnosticos["uptime_segundos"] = backend_uptime
                        self.diagnosticos["uptime_fonte"] = "backend"
                    else:
                        self.diagnosticos["uptime_segundos"] = int(self.diagnosticos["robot_uptime_segundos"])
                        self.diagnosticos["uptime_fonte"] = "robot"
                    self.diagnosticos["executando_trajetoria"] = self.executando_trajetoria
                    self.diagnosticos["saida_digital_ativa"] = self.saida_digital_ativa
                    self.contador_ciclos_telemetria += 1

                    # Reconexão do joystick se plugado depois
                    if joy is None and pygame.joystick.get_count() > 0:
                        try:
                            joy = pygame.joystick.Joystick(0)
                            joy.init()
                            print(f"[JOYSTICK] {joy.get_name()} reconectado com sucesso!")
                        except Exception:
                            joy = None

                    # 2. Leitura de alta frequência da posição TCP (~33 ms)
                    if not self.modo_simulacao and self.robot:
                        try:
                            res = self.robot.get_tcp_position()
                            if res and len(res) > 1 and res[0] == 0:
                                self._set_tcp(res[1])
                        except Exception as re:
                            print(f"[TELEMETRIA] Erro temporário ao ler TCP: {re}")

                    # 3. Diagnóstico baixa frequência (~500 ms)
                    if self.contador_ciclos_telemetria >= 15:
                        self.contador_ciclos_telemetria = 0
                        self._atualizar_diagnosticos_baixa_freq()

                    # 4. Joystick
                    if joy:
                        try:
                            if not joy.get_init():
                                joy.init()
                            pygame.event.pump()
                            self._processar_joystick(joy)
                        except (pygame.error, AttributeError) as je:
                            print(f"[JOYSTICK] Controle desconectado ou falhou: {je}")
                            self.parar_grupo([0, 1, 2, 3, 4, 5])
                            self.grupo_ativo = None
                            joy = None

                    # 5. Marca telemetria real como indisponível se ficar velha.
                    last_real = self.diagnosticos.get("ultima_telemetria_real_ts")
                    if not self.modo_simulacao and (not last_real or (time.time() - last_real) > 5.0):
                        self.diagnosticos["telemetria_real"] = False
                        if self.diagnosticos.get("telemetria_origem") != "TCP 10000":
                            self.diagnosticos["telemetria_status"] = "Sem telemetria real recente"

                    # 6. Socket payload
                    if self.on_state_update:
                        self.on_state_update({
                            "tcp": self._copy_tcp(),
                            "pontos": self._get_pontos_snapshot(),
                            "modo_sim": self.modo_simulacao,
                            "angulo_operador": math.degrees(self.angulo_operador),
                            "diagnosticos": copy.deepcopy(self.diagnosticos),
                        })

                except Exception as e:
                    print(f"[LOOP] Erro inesperado no loop de controle: {e}")

                time.sleep(0.03)

        t = threading.Thread(target=loop, daemon=True)
        t.start()

    def _atualizar_diagnosticos_baixa_freq(self):
        """Atualiza telemetria de diagnóstico.

        A fonte principal é get_robot_status(), documentada na SDK Python V2.1.7.
        O retorno de sucesso é (0, robotstatus), onde robotstatus tem 24 campos.
        O campo 21 (índice 20 em Python) é robot_monitor_data, contendo:
          [SCB major, SCB minor, temperatura do controlador,
           tensão média, corrente média,
           dados das 6 juntas]
        Cada junta contém, conforme TCP Protocol/monitor_data:
          [corrente instantânea, tensão instantânea, temperatura,
           potência média, flutuação de corrente, ciclos acumulados,
           tempo acumulado, ciclos após boot, tempo após boot, torque]
        """
        if self.modo_simulacao:
            backend_uptime = int(time.time() - self.tempo_inicio_sistema)
            self.diagnosticos["uptime_segundos"] = backend_uptime
            self.diagnosticos["backend_uptime_segundos"] = backend_uptime
            self.diagnosticos["robot_uptime_segundos"] = None
            self.diagnosticos["uptime_fonte"] = "simulacao/backend"
            self.diagnosticos["telemetria_real"] = False
            self.diagnosticos["telemetria_origem"] = "Simulação"
            self.diagnosticos["telemetria_status"] = "Placeholder/simulação local"
            for i in range(6):
                fator_esforco = 1.8 if self.grupo_ativo is not None or self.executando_trajetoria else 0.1
                self.diagnosticos["correntes"][i] = round(max(0.0, random.random() * fator_esforco), 2)
                self.diagnosticos["temperaturas"][i] = round(32.0 + (fator_esforco * 2) + random.uniform(-0.2, 0.2), 1)
            return

        if not self.robot:
            return

        # Se a porta 10000 atualizou recentemente, ela é a fonte preferencial.
        last_real = self.diagnosticos.get("ultima_telemetria_real_ts")
        if self.diagnosticos.get("telemetria_origem") == "TCP 10000" and last_real and (time.time() - last_real) < 2.0:
            return

        try:
            status_res = self.robot.get_robot_status()
            if not (status_res and len(status_res) > 1 and status_res[0] == 0):
                return

            status = status_res[1]
            if isinstance(status, (list, tuple)):
                self._extrair_status_sdk_lista(status)
            elif isinstance(status, dict):
                self._extrair_status_sdk_dict(status)

        except Exception as e:
            print(f"[DIAG] Falha ao ler telemetria real da SDK: {e}")

    @staticmethod
    def _to_float(value, default=None):
        try:
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _to_bool(value):
        try:
            return bool(int(value))
        except Exception:
            return bool(value)

    def _extrair_status_sdk_lista(self, status):
        """Extrai campos do get_robot_status() no formato oficial de lista."""
        try:
            # Índices zero-based derivados da tabela da SDK:
            # 0 errcode, 1 inpos, 2 power_on, 3 enabled, 5 protective_stop,
            # 18 cart_position, 19 joint_position, 20 robot_monitor_data, 23 emergency_stop.
            if len(status) > 0:
                self.diagnosticos["codigo_erro"] = status[0]
            if len(status) > 1:
                self.diagnosticos["inpos"] = self._to_bool(status[1])
            if len(status) > 2:
                self.diagnosticos["power_on"] = self._to_bool(status[2])
            if len(status) > 3:
                self.diagnosticos["enabled"] = self._to_bool(status[3])
            if len(status) > 5:
                self.diagnosticos["protective_stop"] = self._to_bool(status[5])
            if len(status) > 18 and isinstance(status[18], (list, tuple)) and len(status[18]) >= 6:
                # Aproveita o status para atualizar TCP se vier no pacote.
                self._set_tcp(status[18])
            if len(status) > 20:
                self._extrair_robot_monitor_data(status[20])
            if len(status) > 23:
                self.diagnosticos["status_emergencia"] = self._to_bool(status[23])
        except Exception as e:
            print(f"[DIAG] Erro interpretando get_robot_status(lista): {e}")

    def _extrair_status_sdk_dict(self, status):
        """Fallback para bindings/versões que retornem dicionário."""
        try:
            self.diagnosticos["codigo_erro"] = status.get("errcode", status.get("err_code", self.diagnosticos["codigo_erro"]))
            self.diagnosticos["inpos"] = self._to_bool(status.get("inpos", self.diagnosticos["inpos"]))
            self.diagnosticos["power_on"] = self._to_bool(status.get("power_on", self.diagnosticos["power_on"]))
            self.diagnosticos["enabled"] = self._to_bool(status.get("enabled", self.diagnosticos["enabled"]))
            self.diagnosticos["protective_stop"] = self._to_bool(status.get("protective_stop", self.diagnosticos["protective_stop"]))
            self.diagnosticos["status_emergencia"] = self._to_bool(status.get("emergency_stop", status.get("estop", self.diagnosticos["status_emergencia"])))

            cart = status.get("cart_position") or status.get("actual_position") or status.get("tcp")
            if isinstance(cart, (list, tuple)) and len(cart) >= 6:
                self._set_tcp(cart)

            monitor = status.get("robot_monitor_data") or status.get("monitor_data")
            self._extrair_robot_monitor_data(monitor)
        except Exception as e:
            print(f"[DIAG] Erro interpretando get_robot_status(dict): {e}")

    def _extrair_robot_monitor_data(self, monitor, origem="SDK get_robot_status"):
        """Extrai corrente/temperatura/torque/tensão de robot_monitor_data.

        Formatos aceitos:
        - formato oficial: [major, minor, cab_temp, avg_voltage, avg_current, joints]
          onde joints = [[cur, volt, temp, power, fluct, cum_cycles, cum_time,
                          boot_cycles, boot_time, torque], ... x6]
        - alguns fallbacks defensivos para listas/dicts de versões diferentes.
        """
        if monitor is None:
            return False

        try:
            # Formato oficial do TCP/SDK.
            if isinstance(monitor, (list, tuple)) and len(monitor) >= 6:
                cab_temp = self._to_float(monitor[2])
                avg_voltage = self._to_float(monitor[3])
                avg_current = self._to_float(monitor[4])
                if cab_temp is not None:
                    self.diagnosticos["controller_temperature"] = round(cab_temp, 1)
                if avg_voltage is not None:
                    self.diagnosticos["robot_average_voltage"] = round(avg_voltage, 2)
                if avg_current is not None:
                    self.diagnosticos["robot_average_current"] = round(avg_current, 2)

                joints = monitor[5]
                if self._extrair_joints_monitor(joints):
                    self.diagnosticos["telemetria_real"] = True
                    self.diagnosticos["telemetria_origem"] = origem
                    self.diagnosticos["telemetria_status"] = f"Telemetria real via {origem}"
                    self.diagnosticos["ultima_telemetria_real_ts"] = time.time()
                    return True
                return False

            # Fallback: dict com vetores.
            if isinstance(monitor, dict):
                self._extrair_status_sdk_dict({"monitor_data": monitor})
                return bool(self.diagnosticos.get("telemetria_real"))
        except Exception as e:
            print(f"[DIAG] Erro extraindo robot_monitor_data: {e}")
        return False

    def _extrair_joints_monitor(self, joints):
        if joints is None:
            return False

        parsed = False
        robot_boot_times = []

        # Caso esperado: lista com 6 listas, cada uma com ao menos [current, voltage, temperature].
        if isinstance(joints, (list, tuple)) and len(joints) >= 6:
            # Se vier flat com 60 valores, divide em 6 blocos de 10.
            if all(not isinstance(x, (list, tuple, dict)) for x in joints) and len(joints) >= 60:
                joint_rows = [joints[i * 10:(i + 1) * 10] for i in range(6)]
            else:
                joint_rows = list(joints[:6])

            for idx, row in enumerate(joint_rows[:6]):
                if isinstance(row, dict):
                    cur = self._to_float(row.get("current", row.get("cur")))
                    volt = self._to_float(row.get("voltage", row.get("volt")))
                    temp = self._to_float(row.get("temperature", row.get("temp")))
                    torque = self._to_float(row.get("torque"))
                    boot_time = self._to_float(row.get("running_time_after_boot", row.get("boot_time")))
                elif isinstance(row, (list, tuple)):
                    cur = self._to_float(row[0] if len(row) > 0 else None)
                    volt = self._to_float(row[1] if len(row) > 1 else None)
                    temp = self._to_float(row[2] if len(row) > 2 else None)
                    torque = self._to_float(row[9] if len(row) > 9 else None)
                    boot_time = self._to_float(row[8] if len(row) > 8 else None)
                else:
                    continue

                if cur is not None:
                    self.diagnosticos["correntes"][idx] = round(cur, 2)
                    parsed = True
                if volt is not None:
                    self.diagnosticos["tensoes"][idx] = round(volt, 2)
                    parsed = True
                if temp is not None:
                    self.diagnosticos["temperaturas"][idx] = round(temp, 1)
                    parsed = True
                if torque is not None:
                    self.diagnosticos["torques"][idx] = round(torque, 2)
                    parsed = True
                if boot_time is not None and boot_time >= 0:
                    robot_boot_times.append(boot_time)

        if robot_boot_times:
            # Usa o maior tempo entre juntas como proxy de tempo interno após boot.
            # A documentação nomeia o campo como running time after this boot, mas não garante unidade;
            # em uso normal espera-se segundos. Se vier absurdo, o frontend ainda recebe a fonte marcada.
            robot_uptime = max(robot_boot_times)
            self.diagnosticos["robot_uptime_segundos"] = int(robot_uptime)
            self.diagnosticos["uptime_segundos"] = int(robot_uptime)
            self.diagnosticos["uptime_fonte"] = "robot_monitor_data.joint_running_time_after_boot"

        return parsed

    def _processar_joystick(self, joy):
        # Troca de orientação (L3 e R3)
        b7 = joy.get_button(7)
        if b7 == 1 and self.last_btns[7] == 0:
            self.angulo_operador -= math.pi / 2
            print(f"[PERSPECTIVA] Visão à Esquerda. Ângulo: {math.degrees(self.angulo_operador)}°")
        self.last_btns[7] = b7

        b8 = joy.get_button(8)
        if b8 == 1 and self.last_btns[8] == 0:
            self.angulo_operador += math.pi / 2
            print(f"[PERSPECTIVA] Visão à Direita. Ângulo: {math.degrees(self.angulo_operador)}°")
        self.last_btns[8] = b8

        cos_theta = math.cos(self.angulo_operador)
        sin_theta = math.sin(self.angulo_operador)

        # Leitura bruta
        btn_14 = joy.get_button(14)
        btn_13 = joy.get_button(13)
        btn_11 = joy.get_button(11)
        btn_12 = joy.get_button(12)

        btn_0 = joy.get_button(0)
        btn_3 = joy.get_button(3)
        btn_1 = joy.get_button(1)
        btn_2 = joy.get_button(2)

        axis_4 = joy.get_axis(4)
        axis_5 = joy.get_axis(5)

        btn_9 = joy.get_button(9)
        btn_10 = joy.get_button(10) if joy.get_init() and joy.get_numbuttons() > 10 else 0

        pressionando_linear = (btn_14 or btn_13 or btn_11 or btn_12)
        pressionando_rotat = (btn_0 or btn_3 or btn_1 or btn_2)
        pressionando_z = (axis_4 > -0.9 or axis_5 > -0.9)
        pressionando_rz = (btn_9 or btn_10)

        if self.grupo_ativo is None:
            if pressionando_linear:
                self.grupo_ativo = 'LINEAR'
            elif pressionando_rotat:
                self.grupo_ativo = 'ROTAT_TCP'
            elif pressionando_z:
                self.grupo_ativo = 'EIXO_Z'
            elif pressionando_rz:
                self.grupo_ativo = 'EIXO_RZ'

        if self.grupo_ativo == 'LINEAR' and not pressionando_linear:
            self.parar_grupo([0, 1])
            self.grupo_ativo = None
        elif self.grupo_ativo == 'ROTAT_TCP' and not pressionando_rotat:
            self.parar_grupo([3, 4])
            self.grupo_ativo = None
        elif self.grupo_ativo == 'EIXO_Z' and not pressionando_z:
            self.parar_grupo([2])
            self.grupo_ativo = None
        elif self.grupo_ativo == 'EIXO_RZ' and not pressionando_rz:
            self.parar_grupo([5])
            self.grupo_ativo = None

        if not (pressionando_linear or pressionando_rotat or pressionando_z or pressionando_rz):
            self.grupo_ativo = None

        if self.grupo_ativo == 'LINEAR':
            v_x_bruta = float(btn_14 - btn_13) * self.MAX_SPD_LINEAR
            v_y_bruta = float(btn_11 - btn_12) * self.MAX_SPD_LINEAR
            v_x_rotacionada = v_x_bruta * cos_theta - v_y_bruta * sin_theta
            v_y_rotacionada = v_x_bruta * sin_theta + v_y_bruta * cos_theta
            self.enviar_jog(0, v_x_rotacionada, 0)
            self.enviar_jog(1, v_y_rotacionada, 0)

        elif self.grupo_ativo == 'ROTAT_TCP':
            v_rx_bruta = float(btn_0 - btn_3) * self.MAX_SPD_ROTAT
            v_ry_bruta = float(btn_1 - btn_2) * self.MAX_SPD_ROTAT
            v_rx_rotacionada = v_rx_bruta * cos_theta - v_ry_bruta * sin_theta
            v_ry_rotacionada = v_rx_bruta * sin_theta + v_ry_bruta * cos_theta
            self.enviar_jog(3, v_rx_rotacionada, 2)
            self.enviar_jog(4, v_ry_rotacionada, 2)

        elif self.grupo_ativo == 'EIXO_Z':
            g_down = (axis_4 + 1.0) / 2.0 if axis_4 > -0.9 else 0.0
            g_up = (axis_5 + 1.0) / 2.0 if axis_5 > -0.9 else 0.0
            v_z = (g_up - g_down) * self.MAX_SPD_LINEAR
            self.enviar_jog(2, v_z, 0)

        elif self.grupo_ativo == 'EIXO_RZ':
            v_rz = float(btn_9 - btn_10) * self.MAX_SPD_ROTAT
            self.enviar_jog(5, v_rz, 1)

        # Salvar ponto linear
        b6 = joy.get_button(6)
        if b6 == 1 and self.last_btns[6] == 0:
            self.salvar_ponto_atual('L')
        self.last_btns[6] = b6

        # Salvar ponto circular
        b4 = joy.get_button(4)
        if b4 == 1 and self.last_btns[4] == 0:
            self.salvar_ponto_atual('C')
        self.last_btns[4] = b4

        # Botão 5: toque remove último; segurar 2s limpa tudo
        b5 = joy.get_button(5)
        if b5 == 1:
            if self.last_btns[5] == 0:
                self.b5_press_time = time.time()
                self.b5_triggered_long_press = False
            elif self.b5_press_time and not self.b5_triggered_long_press:
                if time.time() - self.b5_press_time >= 2.0:
                    self._set_pontos([])
                    print("[JOYSTICK] Botão 5 segurado por 2s: trajetória limpa.")
                    self.b5_triggered_long_press = True
        elif b5 == 0 and self.last_btns[5] == 1:
            if not self.b5_triggered_long_press:
                pts = self._get_pontos_snapshot()
                if pts:
                    pts.pop()
                    self._set_pontos(pts)
                    print("[JOYSTICK] Botão 5: último ponto removido.")
            self.b5_press_time = None
            self.b5_triggered_long_press = False
        self.last_btns[5] = b5

        # Botão 15: executar com backup dos pontos para manter a tela
        b15 = joy.get_button(15)
        if b15 == 1 and self.last_btns[15] == 0:
            def intertravamento_execucao_joystick():
                backup_pts = self._get_pontos_snapshot()
                self.executar_trajetoria()
                self._set_pontos(backup_pts)

            threading.Thread(target=intertravamento_execucao_joystick, daemon=True).start()
        self.last_btns[15] = b15

    def salvar_ponto_atual(self, tipo):
        pose = self._copy_tcp()
        with self._state_lock:
            self.lista_pontos.append((tipo, pose))
            pts = copy.deepcopy(self.lista_pontos)
        if self.on_point_saved:
            self.on_point_saved(pts)


adapter = RobotAdapter()
