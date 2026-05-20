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
            "temperaturas": [32.5, 33.1, 31.8, 34.2, 32.9, 33.5],
            "correntes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "uptime_segundos": 0,
            "status_emergencia": False,
            "codigo_erro": 0,
            "executando_trajetoria": False,
            "saida_digital_ativa": False,
        }
        self.contador_ciclos_telemetria = 0

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

            self.ip_atual = ip
            self.modo_simulacao = False
            print(f"[ROBÔ] Conectado e habilitado em {ip}.")
            return True

        except Exception as e:
            print(f"[ERRO] Falha ao conectar no robô: {e}")
            self.modo_simulacao = True
            return False

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

    def _fase_trajetoria_principal(self, pontos):
        """Trajetória comum para MoveL e MoveC, sem I/O no meio."""
        i = 1  # P1 já foi consumido pela fase de entrada
        while i < len(pontos):
            tipo, pose = pontos[i]

            # Watchdog leve antes de cada novo comando: não inicia próximo segmento se DO caiu.
            do_val = self.ler_saida_digital()
            if do_val is not None and not do_val:
                self.parar_movimento_processo()
                return False, "Saída digital desligada antes/durante a trajetória principal. Movimento abortado."

            if tipo == "C" and i + 1 < len(pontos) and pontos[i + 1][0] == "C":
                mid_pose = list(pontos[i][1])
                end_pose = list(pontos[i + 1][1])
                print(f"[SEQ] Trajetória: MoveC mid={mid_pose[:3]} end={end_pose[:3]}.")
                if not self._circular_move_nao_bloqueante(end_pose=end_pose, mid_pose=mid_pose, vel=self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, "Falha ao enviar MoveC. Movimento abortado."
                i += 2
            else:
                print(f"[SEQ] Trajetória: MoveL {list(pose)[:3]}.")
                if not self._linear_move_nao_bloqueante(list(pose), self.vel_reproducao):
                    self.parar_movimento_processo()
                    return False, "Falha ao enviar MoveL. Movimento abortado."
                i += 1

            # Pequeno yield para não empurrar todos os comandos no mesmo instante e dar tempo
            # ao controlador/telemetria de atualizar estado de processo.
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
            if len(pontos) < 2:
                return "Necessário ao menos 2 pontos para iniciar."

            h_clearance = [0, 0, 50, 0, 0, 0]
            p1_real = list(pontos[0][1])
            ultimo_real = list(pontos[-1][1])

            if self.modo_simulacao:
                print("[SIMULAÇÃO] Executando trajetória híbrida V10.")

            ok, msg = self._fase_entrada_processo(p1_real, h_clearance)
            if not ok:
                return msg

            ok, msg = self._fase_trajetoria_principal(pontos)
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
                    # 1. Uptime / divisores
                    self.diagnosticos["uptime_segundos"] = int(time.time() - self.tempo_inicio_sistema)
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

                    # 5. Socket payload
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
        if self.modo_simulacao:
            for i in range(6):
                fator_esforco = 1.8 if self.grupo_ativo is not None or self.executando_trajetoria else 0.1
                self.diagnosticos["correntes"][i] = round(max(0.0, random.random() * fator_esforco), 2)
                self.diagnosticos["temperaturas"][i] = round(32.0 + (fator_esforco * 2) + random.uniform(-0.2, 0.2), 1)
            return

        if not self.robot:
            return

        try:
            # Mantido defensivo porque a forma exata do retorno varia por SDK/versão.
            status_res = self.robot.get_robot_status()
            if status_res and len(status_res) > 1 and status_res[0] == 0:
                data = status_res[1]

                if isinstance(data, dict):
                    self.diagnosticos["status_emergencia"] = bool(data.get("estop", 0) or data.get("emergency_stop", 0))
                    self.diagnosticos["codigo_erro"] = data.get("err_code", self.diagnosticos["codigo_erro"])

                    # Caso algum SDK retorne monitor_data como dict/lista dentro do dict.
                    monitor = data.get("robot_monitor_data") or data.get("monitor_data")
                    self._extrair_monitor_data(monitor)

                elif isinstance(data, (list, tuple)):
                    # Manual SDK descreve get_robot_status como lista com múltiplos campos.
                    # Tentamos extrair qualquer subestrutura plausível sem travar o loop.
                    for item in data:
                        self._extrair_monitor_data(item)

            # Fallback opcional: se existir get_joint_status na binding, tenta ler.
            if hasattr(self.robot, "get_joint_status"):
                joint_res = self.robot.get_joint_status()
                if joint_res and len(joint_res) > 1 and joint_res[0] == 0:
                    self._extrair_monitor_data(joint_res[1])

        except Exception as e:
            print(f"[DIAG] Falha oculta na telemetria da SDK: {e}")

    def _extrair_monitor_data(self, monitor):
        if monitor is None:
            return

        # Caso lista de dicts por junta
        if isinstance(monitor, list):
            for idx in range(min(6, len(monitor))):
                item = monitor[idx]
                if isinstance(item, dict):
                    temp = item.get("temperature", item.get("temp", None))
                    cur = item.get("current", item.get("cur", None))
                    if temp is not None:
                        self.diagnosticos["temperaturas"][idx] = round(float(temp), 1)
                    if cur is not None:
                        self.diagnosticos["correntes"][idx] = round(float(cur), 2)
            return

        # Caso dict com vetores
        if isinstance(monitor, dict):
            temps = (
                monitor.get("temperatures") or
                monitor.get("joint_temperatures") or
                monitor.get("temperature") or
                monitor.get("temps")
            )
            currents = (
                monitor.get("currents") or
                monitor.get("joint_currents") or
                monitor.get("current") or
                monitor.get("curs")
            )

            if isinstance(temps, (list, tuple)):
                for i in range(min(6, len(temps))):
                    self.diagnosticos["temperaturas"][i] = round(float(temps[i]), 1)
            if isinstance(currents, (list, tuple)):
                for i in range(min(6, len(currents))):
                    self.diagnosticos["correntes"][i] = round(float(currents[i]), 2)

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
