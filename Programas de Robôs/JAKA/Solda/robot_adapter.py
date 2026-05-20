# robot_adapter.py
import sys
import os
import time
import math
import pygame
import threading
import copy  # 💡 Importado para garantir o backup de pontos na execução via Joystick

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
        
        # Parâmetros de Controle
        self.MAX_SPD_LINEAR = 120.0
        self.MAX_SPD_ROTAT = 6.0
        self.DEADZONE = 0.2
        
        # Estado dinâmico do robô
        self.posicao_atual_tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.lista_pontos = []
        self.vel_reproducao = 15.0
        self.vel_aproximacao = 150.0
        
        # --- NOVO: ESTRUTURA DE TELEMETRIA E DIAGNÓSTICO PARA A FEIRA ---
        self.tempo_inicio_sistema = time.time()
        self.diagnosticos = {
            "temperaturas": [32.5, 33.1, 31.8, 34.2, 32.9, 33.5],  # 6 Juntas (°C)
            "correntes": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # 6 Motores (A)
            "uptime_segundos": 0,                                  # Tempo de atividade
            "status_emergencia": False,                           # Estado do E-Stop
            "codigo_erro": 0                                      # Código de falha ativo
        }
        self.contador_ciclos_telemetria = 0  # Divisor de frequência p/ leitura assíncrona
        
        # 💡 Variáveis para o debounce e retenção do Botão 5
        self.b5_press_time = None
        self.b5_triggered_long_press = False
        
        # --- SISTEMA DE COORDENADAS DO OPERADOR ---
        self.angulo_operador = 0.0  
        
        # Histórico de controle para debounce/edge detection
        self.last_sent_vels = {0: 0.0, 1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0}
        self.last_btns = {4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 15: 0}
        
        # --- VARIÁVEL DE TRAVA DE EXCLUSIVIDADE ---
        self.grupo_ativo = None

        # Callbacks para notificar o app.py via Socket.IO
        self.on_state_update = None
        self.on_point_saved = None

    def conectar(self, ip="192.168.0.200"):
        if self.modo_simulacao:
            print("[SIMULAÇÃO] Robô conectado ficticiamente.")
            return True
        try:
            self.robot = jkrc.RC(ip)
            self.robot.login()
            self.robot.power_on()
            self.robot.enable_robot()
            print(f"[ROBÔ] Conectado com sucesso em {ip}!")
            return True
        except Exception as e:
            print(f"[ERRO] Falha ao conectar no robô: {e}")
            self.modo_simulacao = True
            return False

    def enviar_jog(self, eixo, vel, coord):
        if self.modo_simulacao:
            if abs(vel) > 0.01:
                self.posicao_atual_tcp[eixo] += (vel * 0.05)
            return

        vel = round(float(vel), 2)
        if abs(vel) < self.DEADZONE:
            vel = 0.0

        if vel != self.last_sent_vels[eixo]:
            try:
                if vel != 0.0:
                    self.robot.jog(eixo, 2, coord, vel, 0)
                else:
                    self.robot.jog_stop(eixo)
                self.last_sent_vels[eixo] = vel
            except Exception as e:
                print(f"[ERRO JOG] Eixo {eixo}: {e}")

    def parar_grupo(self, eixos_grupo):
        """Força a parada imediata dos eixos de um grupo específico e limpa o histórico."""
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

    def _esperar_chegada_fisica(self):
        """Bloqueia a linha de código Python até o robô físico estar 100% parado no destino."""
        if self.modo_simulacao:
            return
        while True:
            ret = self.robot.is_in_pos()
            if ret[0] == 0 and ret[1] == 1:  # 0 = OK no SDK, 1 = Robô parado na posição de destino
                break
            time.sleep(0.005)  # Alivia processamento da CPU

    def executar_trajetoria(self):
        if len(self.lista_pontos) < 2:
            return "Necessário ao menos 2 pontos para iniciar."
        
        if self.modo_simulacao:
            print("[SIMULAÇÃO] Executando trajetória mista na bancada virtual...")
            for tipo, pose in self.lista_pontos:
                self.posicao_atual_tcp = list(pose)
                time.sleep(0.5)
            return "Trajetória simulada concluída!"

        try:
            # =================================================================
            # FASE 1: LIFECYCLE - ENTRADA / APROXIMAÇÃO (Sequencial Dedicado)
            # =================================================================
            h_in = [0, 0, 50, 0, 0, 0]  # Z-Clearance de segurança (50mm)
            p1_real = self.lista_pontos[0][1]
            p1_aprox = [a + b for a, b in zip(p1_real, h_in)]
            
            # 1. Garante que a ferramenta comece DESLIGADA na aproximação
            self.robot.set_digital_output(0, 1, False)
            
            # 2. Desloca até a altura de segurança acima do Ponto 1
            self.robot.linear_move(p1_aprox, 0, False, self.vel_aproximacao)
            self._esperar_chegada_fisica()
            
            # 3. Desce verticalmente até tocar o Ponto 1 real
            self.robot.linear_move(p1_real, 0, False, self.vel_aproximacao)
            self._esperar_chegada_fisica()  # Sincronismo: Garante parada mecânica no P1
            
            # 4. ATIVAÇÃO DA SAÍDA DIGITAL (Exatamente em cima do primeiro ponto)
            self.robot.set_digital_output(0, 1, True)
            
            # =================================================================
            # FASE 2: TRAJETÓRIA PRINCIPAL (Fluida/Non-blocking p/ Blending de Movimento)
            # =================================================================
            i = 0
            while i < len(self.lista_pontos):
                if i + 2 < len(self.lista_pontos) and self.lista_pontos[i+1][0] == 'C' and self.lista_pontos[i+2][0] == 'C':
                    mid_pose = self.lista_pontos[i+1][1]
                    end_pose = self.lista_pontos[i+2][1]
                    
                    self.robot.circular_move(end_pose, mid_pose, 0, False, self.vel_reproducao, 800, 0)
                    i += 2
                else:
                    tipo, pose = self.lista_pontos[i]
                    self.robot.linear_move(pose, 0, False, self.vel_reproducao)
                    i += 1
            
            # AGUARDA MARCO DE CONCLUSÃO: Espera toda a trajetória principal ser concluída fisicamente
            self._esperar_chegada_fisica()
            
            # =================================================================
            # FASE 3: LIFECYCLE - DESATIVAÇÃO E SAÍDA / RECUO (Sequencial Dedicado)
            # =================================================================
            # 1. DESATIVAÇÃO DA SAÍDA DIGITAL (Exatamente no término do último ponto)
            self.robot.set_digital_output(0, 1, False)
            
            # 2. Captura a posição física atual (que agora sim é garantidamente o ÚLTIMO ponto)
            res_cur = self.robot.get_tcp_position()
            if res_cur[0] == 0:
                p_ultimo_real = res_cur[1]
                p_saida_aprox = [a + b for a, b in zip(p_ultimo_real, h_in)]
                
                # 3. Sobe verticalmente aplicando o clearance baseado no ÚLTIMO ponto
                self.robot.linear_move(p_saida_aprox, 0, False, self.vel_aproximacao)
                self._esperar_chegada_fisica()
            
            # Sincroniza HMI para reter os pontos na tela perfeitamente
            if self.on_point_saved:
                self.on_point_saved(self.lista_pontos)
                
            return "Trajetória física executada com sucesso!"
            
        except Exception as e:
            try:
                self.robot.set_digital_output(0, 1, False)
            except:
                pass
            return f"Erro na execução da trajetória: {e}"

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
                # 1. ATUALIZAÇÃO DO UPTIME DO SISTEMA (Roda em cada ciclo)
                self.diagnosticos["uptime_segundos"] = int(time.time() - self.tempo_inicio_sistema)
                self.contador_ciclos_telemetria += 1

                if joy is None and pygame.joystick.get_count() > 0:
                    try:
                        joy = pygame.joystick.Joystick(0)
                        joy.init()
                    except Exception:
                        joy = None

                # 2. LEITURA DE ALTA FREQUÊNCIA DA POSIÇÃO DO ROBÔ (33ms)
                if not self.modo_simulacao and self.robot:
                    res = self.robot.get_tcp_position()
                    if res[0] == 0:
                        self.posicao_atual_tcp = res[1]

                # 3. LEITURA DE BAIXA FREQUÊNCIA PARA DIAGNÓSTICOS (A cada 15 ciclos ~= 500ms)
                if self.contador_ciclos_telemetria >= 15:
                    self.contador_ciclos_telemetria = 0  # Reseta o divisor
                    
                    if not self.modo_simulacao and self.robot:
                        try:
                            # A. Captura Estado de Emergência e Erros Gerais
                            status_res = self.robot.get_robot_status()
                            if status_res[0] == 0 and isinstance(status_res[1], dict):
                                st_data = status_res[1]
                                # Procura chaves comuns da SDK da JAKA para parada de emergência
                                self.diagnosticos["status_emergencia"] = st_data.get("estop", 0) == 1 or st_data.get("emergency_stop", 0) == 1
                                self.diagnosticos["codigo_erro"] = st_data.get("err_code", 0)

                            # B. Captura Temperatura e Corrente de cada junta
                            joint_res = self.robot.get_joint_status()
                            if joint_res[0] == 0 and isinstance(joint_res[1], list):
                                j_list = joint_res[1]
                                for idx in range(min(6, len(j_list))):
                                    if isinstance(j_list[idx], dict):
                                        self.diagnosticos["temperaturas"][idx] = round(j_list[idx].get("temperature", j_list[idx].get("temp", 35.0)), 1)
                                        self.diagnosticos["correntes"][idx] = round(j_list[idx].get("current", j_list[idx].get("cur", 0.0)), 2)
                        except Exception as e:
                            print(f"[DIAG] Falha oculta na telemetria da SDK: {e}")
                    
                    elif self.modo_simulacao:
                        # 💡 Simulação Dinâmica para visualização em bancada/feira
                        import random
                        for i in range(6):
                            # Se houver comando de jog ativo, simula acréscimo de corrente e calor
                            fator_esforco = 1.8 if self.grupo_ativo is not None else 0.1
                            self.diagnosticos["correntes"][i] = round(max(0.0, (random.random() * fator_esforco)), 2)
                            self.diagnosticos["temperaturas"][i] = round(32.0 + (fator_esforco * 2) + random.uniform(-0.2, 0.2), 1)

                if joy:
                    pygame.event.pump()
                    
                    # --- TROCA DE ORIENTAÇÃO (L3 e R3) ---
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

                    # --- LEITURA BRUTA E CALIBRAÇÃO DE EIXOS ---
                    btn_14 = joy.get_button(14)
                    btn_13 = joy.get_button(13)
                    btn_11 = joy.get_button(11)
                    btn_12 = joy.get_button(12)
                    
                    btn_0 = joy.get_button(0)
                    btn_3 = joy.get_button(3)
                    btn_1 = joy.get_button(1)
                    btn_2 = joy.get_button(2)
                    
                    # Calibração estrita dos Gatilhos (Solto = -1.0, Pressionado = 1.0)
                    axis_4 = joy.get_axis(4)
                    axis_5 = joy.get_axis(5)
                    
                    btn_9 = joy.get_button(9)
                    btn_10 = joy.get_button(10) if joy.get_init() and joy.get_numbuttons() > 10 else 0

                    # --- DETECÇÃO REAL DE PRESSIONAMENTO (Filtro de Ruído/Estado Inicial) ---
                    pressionando_linear = (btn_14 or btn_13 or btn_11 or btn_12)
                    pressionando_rotat = (btn_0 or btn_3 or btn_1 or btn_2)
                    
                    # SÓ considera pressionado se o gatilho saiu do repouso de -1.0 e passou de -0.9
                    pressionando_z = (axis_4 > -0.9 or axis_5 > -0.9)
                    pressionando_rz = (btn_9 or btn_10)

                    # --- MÁQUINA DE ESTADOS COM TRAVA DE EXCLUSIVIDADE ---
                    if self.grupo_ativo is None:
                        if pressionando_linear:
                            self.grupo_ativo = 'LINEAR'
                        elif pressionando_rotat:
                            self.grupo_ativo = 'ROTAT_TCP'
                        elif pressionando_z:
                            self.grupo_ativo = 'EIXO_Z'
                        elif pressionando_rz:
                            self.grupo_ativo = 'EIXO_RZ'

                    # Condições de liberação de trava e parada forçada
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

                    # --- REDUNDÂNCIA ABSOLUTA DE SEGURANÇA ---
                    if not (pressionando_linear or pressionando_rotat or pressionando_z or pressionando_rz):
                        self.grupo_ativo = None

                    # --- EXECUÇÃO DOS MOVIMENTOS SELECIONADOS ---
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

                    # --- DETECÇÃO DE BORDAS (BOTÕES DE SALVAMENTO) ---
                    b6 = joy.get_button(6)
                    if b6 == 1 and self.last_btns[6] == 0:
                        self.salvar_ponto_atual('L')
                    self.last_btns[6] = b6

                    b4 = joy.get_button(4)
                    if b4 == 1 and self.last_btns[4] == 0:
                        self.salvar_ponto_atual('C')
                    self.last_btns[4] = b4

                    # --- BOTÃO 5 MODIFICADO COM RETENÇÃO DE 2 SEGUNDOS ---
                    b5 = joy.get_button(5)
                    if b5 == 1:
                        if self.last_btns[5] == 0:
                            self.b5_press_time = time.time()
                            self.b5_triggered_long_press = False
                        else:
                            if self.b5_press_time and not self.b5_triggered_long_press:
                                if time.time() - self.b5_press_time >= 2.0:
                                    self.lista_pontos = []  
                                    if self.on_point_saved: 
                                        self.on_point_saved(self.lista_pontos)
                                    print("[JOYSTICK] Botão 5 segurado por 2s: Toda a trajetória foi limpa!")
                                    self.b5_triggered_long_press = True
                    elif b5 == 0 and self.last_btns[5] == 1:
                        if not self.b5_triggered_long_press:
                            if self.lista_pontos:
                                self.lista_pontos.pop()
                                if self.on_point_saved: self.on_point_saved(self.lista_pontos)
                                print("[JOYSTICK] Botão 5 pressionado: Último ponto removido.")
                        self.b5_press_time = None
                        self.b5_triggered_long_press = False
                    self.last_btns[5] = b5

                    # --- GATILHO SEGURO DO BOTÃO 15 PELO JOYSTICK ---
                    b15 = joy.get_button(15)
                    if b15 == 1 and self.last_btns[15] == 0:
                        # 💡 Integrada a lógica de backup idêntica à HMI para evitar a perda dos pontos da tela
                        def intertravamento_execucao_joystick():
                            backup_pts = copy.deepcopy(self.lista_pontos)
                            self.executar_trajetoria()
                            self.lista_pontos = backup_pts
                            if self.on_point_saved:
                                self.on_point_saved(backup_pts)
                        
                        threading.Thread(target=intertravamento_execucao_joystick, daemon=True).start()
                    self.last_btns[15] = b15

                # 4. ENVIO AUTOMÁTICO VIA SOCKET PARA O FRONT-END
                if self.on_state_update:
                    self.on_state_update({
                        "tcp": self.posicao_atual_tcp,
                        "pontos": self.lista_pontos,
                        "modo_sim": self.modo_simulacao,
                        "angulo_operador": math.degrees(self.angulo_operador),
                        # Injectando o novo payload de diagnósticos estruturado
                        "diagnosticos": self.diagnosticos
                    })

                time.sleep(0.03)

        t = threading.Thread(target=loop, daemon=True)
        t.start()

    def salvar_ponto_atual(self, tipo):
        pose = list(self.posicao_atual_tcp)
        self.lista_pontos.append((tipo, pose))
        if self.on_point_saved:
            self.on_point_saved(self.lista_pontos)

adapter = RobotAdapter()