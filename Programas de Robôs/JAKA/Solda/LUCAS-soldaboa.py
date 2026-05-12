import sys
import os
import math
import time
import pygame
import tkinter as tk
from tkinter import ttk

SDK_DIR = r"C:\jakaAPI_V2.1.7stable\SDK2.1.7\Windows\python3\x64"
sys.path.insert(0, SDK_DIR)
os.add_dll_directory(SDK_DIR)
import jkrc 

# --- CONFIGURAÇÕES DO ROBÔ ---
MAX_SPD_LINEAR = 120.0  
MAX_SPD_ROTAT  = 6.0    
PRECISION_SPD  = 20.0   
DEADZONE = 0.2

# Variáveis globais
lista_pontos = [] # Agora armazena: [('L', pose), ('C', pose1), ('C', pose2)...]
posicao_atual_tcp = [0, 0, 0, 0, 0, 0] 
vel_reproducao_global = 15.0
vel_aproximacao_global = 150.0

# --- INTERFACE TKINTER ---
root = tk.Tk()
root.title("Monitor e Trajetória JAKA")
root.geometry("450x750") 
root.attributes("-topmost", True)

# Widgets de Velocidade e Lista
ttk.Label(root, text="Velocidades (Execução / Aproximação):").pack(pady=5)
val_repro = tk.DoubleVar(value=vel_reproducao_global)
scale_repro = ttk.Scale(root, from_=1, to=100, variable=val_repro, orient='horizontal', length=200)
scale_repro.pack()

val_aprox = tk.DoubleVar(value=vel_aproximacao_global)
scale_aprox = ttk.Scale(root, from_=1, to=300, variable=val_aprox, orient='horizontal', length=200)
scale_aprox.pack()

lb_pontos = tk.Listbox(root, width=50, height=8)
lb_pontos.pack(pady=5)

# --- CANVAS PARA DESENHO ---
ttk.Label(root, text="Visualização em Tempo Real:", font=('Arial', 10, 'bold')).pack()
canvas = tk.Canvas(root, width=400, height=400, bg="white", highlightthickness=1, highlightbackground="black")
canvas.pack(pady=10)

def desenhar_tudo():
    canvas.delete("all") 
    escala = 0.4 
    offset_x = 250         
    offset_y = 200

    # 1. CURSOR (TCP)
    tx = offset_x + (posicao_atual_tcp[0] * escala)
    ty = offset_y - (posicao_atual_tcp[1] * escala)
    canvas.create_line(tx-15, ty, tx+15, ty, fill="#00AA00", width=1)
    canvas.create_line(tx, ty-15, tx, ty+15, fill="#00AA00", width=1)
    canvas.create_oval(tx-4, ty-4, tx+4, ty+4, outline="#00AA00", width=2)

    # 2. TRAJETÓRIA
    if len(lista_pontos) > 0:
        i = 0
        while i < len(lista_pontos):
            tipo, p_atual = lista_pontos[i]
            
            # Se for o primeiro ponto, não há linha para trás
            if i == 0:
                cx = offset_x + (p_atual[0] * escala)
                cy = offset_y - (p_atual[1] * escala)
                canvas.create_oval(cx-5, cy-5, cx+5, cy+5, fill="blue")
                canvas.create_text(cx+12, cy-12, text="P1", font=("Arial", 8, "bold"))
                i += 1
                continue

            # Ponto anterior (para começar a linha)
            _, p_anterior = lista_pontos[i-1]
            x0, y0 = offset_x + (p_anterior[0] * escala), offset_y - (p_anterior[1] * escala)

            # LÓGICA DE DESENHO DE ARCO (MoveC)
            if tipo == 'C' and i + 1 < len(lista_pontos) and lista_pontos[i+1][0] == 'C':
                # Temos os 3 pontos: p_anterior (Início), p_atual (Meio), p_proximo (Fim)
                p_meio = p_atual
                p_fim = lista_pontos[i+1][1]
                
                x1, y1 = offset_x + (p_meio[0] * escala), offset_y - (p_meio[1] * escala)
                x2, y2 = offset_x + (p_fim[0] * escala), offset_y - (p_fim[1] * escala)

                # Desenha o arco aproximado usando uma Quadratic Bezier (curva simples)
                # O ponto de controle é ajustado para que a curva passe perto do ponto do meio
                canvas.create_line(x0, y0, x1, y1, x2, y2, fill="orange", smooth=True, width=2)
                
                # Bolinhas dos pontos do arco
                canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="orange")
                canvas.create_oval(x2-5, y2-5, x2+5, y2+5, fill="orange")
                canvas.create_text(x1+12, cy-12, text=f"C{i+1}", font=("Arial", 8))
                canvas.create_text(x2+12, cy-12, text=f"C{i+2}", font=("Arial", 8))
                
                i += 2 # Pula os dois pontos do arco já desenhados
            
            else:
                # LÓGICA DE LINHA RETA (MoveL ou C isolado)
                x1, y1 = offset_x + (p_atual[0] * escala), offset_y - (p_atual[1] * escala)
                canvas.create_line(x0, y0, x1, y1, fill="red", width=2)
                canvas.create_oval(x1-5, y1-5, x1+5, y1+5, fill="black")
                canvas.create_text(x1+12, y1-12, text=f"L{i+1}", font=("Arial", 8))
                i += 1

def atualizar_ui():
    global vel_reproducao_global, vel_aproximacao_global
    vel_reproducao_global = val_repro.get()
    vel_aproximacao_global = val_aprox.get()
    
    if lb_pontos.size() != len(lista_pontos):
        lb_pontos.delete(0, tk.END)
        for i, (tipo, p) in enumerate(lista_pontos):
            msg = "Linear" if tipo == 'L' else "Circular"
            lb_pontos.insert(tk.END, f"{i+1}: {msg} - X:{p[0]:.1f} Y:{p[1]:.1f}")
    
    desenhar_tudo()
    root.update()

# --- INICIALIZAÇÃO ROBÔ ---
robot = jkrc.RC("192.168.0.200")
robot.login()
robot.enable_robot()

pygame.init()
pygame.joystick.init()
joy = pygame.joystick.Joystick(0)
joy.init()

last_sent_vels = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0}
# Estados para detectar clique único (Edge Detection)
last_btns = {4: 0, 5: 0, 6: 0, 15: 0} 

def enviar_jog(eixo, vel, coord):
    vel = round(float(vel), 2)
    if vel != last_sent_vels[eixo]:
        if abs(vel) > 0.01:
            robot.jog(eixo, 2, coord, vel, 0)
        else:
            robot.jog_stop(eixo)
        last_sent_vels[eixo] = vel

try:
    while True:
        res_realtime = robot.get_tcp_position()
        if res_realtime[0] == 0:
            posicao_atual_tcp = res_realtime[1]

        atualizar_ui() 
        pygame.event.pump()

        # --- JOG (X, Y, Z, RX, RY, J6) - MANTIDO INTACTO ---
        v_x = float(joy.get_button(14)- joy.get_button(13)) * MAX_SPD_LINEAR
        v_y = float(joy.get_button(11)- joy.get_button(12)) * MAX_SPD_LINEAR
        enviar_jog(0, v_x, 0) 
        enviar_jog(1, v_y, 0) 
        g_down = (joy.get_axis(4) + 1) / 2 if joy.get_axis(4) != 0 else 0
        g_up = (joy.get_axis(5) + 1) / 2 if joy.get_axis(5) != 0 else 0
        enviar_jog(2, (g_up - g_down) * MAX_SPD_LINEAR, 0) 
        enviar_jog(3, (joy.get_button(1) - joy.get_button(2)) * MAX_SPD_ROTAT, 2) 
        enviar_jog(4, (joy.get_button(0) - joy.get_button(3)) * MAX_SPD_ROTAT, 2) 
        enviar_jog(5, (joy.get_button(9) - joy.get_button(10)) * MAX_SPD_ROTAT, 1)

        # --- LOGICA DE BOTÕES ---
        
        # Botão 6: Salvar Ponto Linear
        b6 = joy.get_button(6)
        if b6 == 1 and last_btns[6] == 0:
            res = robot.get_tcp_position()
            if res[0] == 0:
                lista_pontos.append(('L', res[1]))
                print("Ponto Linear salvo!")
        last_btns[6] = b6

        # Botão 4: Salvar Ponto Circular
        b4 = joy.get_button(4)
        if b4 == 1 and last_btns[4] == 0:
            res = robot.get_tcp_position()
            if res[0] == 0:
                lista_pontos.append(('C', res[1]))
                print("Ponto Circular salvo!")
        last_btns[4] = b4

        # Botão 5: Apagar Último Ponto (Undo)
        b5 = joy.get_button(5)
        if b5 == 1 and last_btns[5] == 0:
            if lista_pontos:
                removido = lista_pontos.pop()
                print(f"Último ponto ({removido[0]}) removido!")
        last_btns[5] = b5

        # Botão 15: Executar Trajetória (Start)
        b15 = joy.get_button(15)
        if b15 == 1 and last_btns[15] == 0:
            if len(lista_pontos) >= 2:
                v_repr = vel_reproducao_global
                v_aprox = vel_aproximacao_global
                h_in = [0,0,50,0,0,0]
                
                print("Iniciando trajetória mista...")
                # Aproximação ao primeiro ponto
                robot.linear_move([a+b for a,b in zip(lista_pontos[0][1], h_in)], 0, 1, v_aprox)
                robot.linear_move(lista_pontos[0][1], 0, 1, v_aprox)
                
                i = 0
                while i < len(lista_pontos):
                    tipo, pose = lista_pontos[i]
                    
                    if tipo == 'L':
                        robot.set_digital_output(0, 9, True)
                        robot.linear_move(pose, 0, 1, v_repr)
                        i += 1
                    elif tipo == 'C':
                        # Verifica se existe um próximo ponto para completar o arco
                        if i + 1 < len(lista_pontos) and lista_pontos[i+1][0] == 'C':
                            robot.set_digital_output(0, 9, True)
                            # Ponto auxiliar (i) e Ponto final (i+1)
                            robot.circular_move(lista_pontos[i+1][1], lista_pontos[i][1], 0, 1, v_repr, 800, 0)
                            i += 2
                        else:
                            # Se for um 'C' isolado, faz linear para não travar
                            robot.linear_move(pose, 0, 1, v_repr)
                            i += 1
                
                robot.set_digital_output(0, 9, False)
                # Saída
                cur = robot.get_tcp_position()[1]
                robot.linear_move([a+b for a,b in zip(cur, h_in)], 0, 1, v_aprox)
                lista_pontos = []
            else:
                print("Necessário ao menos 2 pontos para iniciar.")
        last_btns[15] = b15

        time.sleep(0.01)

except KeyboardInterrupt:
    for i in range(6): robot.jog_stop(i)
    robot.logout()
    root.destroy()
