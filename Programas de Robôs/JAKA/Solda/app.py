# ⚠️ CRÍTICO: Estas duas linhas têm de ser as primeiras absolutas do arquivo!
import eventlet
eventlet.monkey_patch()

from eventlet import tpool  # 💡 Mantido para isolar o C++ do SDK
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO
from robot_adapter import adapter
import copy  # 💡 Adicionado para fazer a cópia real da lista de pontos em memória

app = Flask(__name__)
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'

# ⚠️ Forçamos o async_mode para 'eventlet' garantir compatibilidade com as threads do adapter
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

@app.route('/')
def index():
    return render_template('index.html')

# --- API REST PARA COMANDOS MANUAIS DO FRONT ---
@app.route('/api/config/velocidades', methods=['POST'])
def configurar_velocidades():
    dados = request.json
    adapter.vel_reproducao = float(dados.get('reproducao', adapter.vel_reproducao))
    adapter.vel_aproximacao = float(dados.get('aproximacao', adapter.vel_aproximacao))
    return jsonify({"status": "success"})

@app.route('/api/robo/conectar', methods=['POST'])
def conectar_robo():
    dados = request.json
    ip = dados.get('ip', '192.168.0.200')
    
    # Executa a chamada de conexão forçando validação real do hardware
    sucesso = adapter.conectar(ip)
    
    if sucesso and not adapter.modo_simulacao:
        return jsonify({"status": "success", "message": f"Conectado ao robô físico no IP {ip}!"})
    elif adapter.modo_simulacao and not adapter.sdk_carregado():
        return jsonify({"status": "error", "message": "Falha: Arquivos binários do SDK JAKA (.dll/.pyd) não encontrados."})
    else:
        return jsonify({"status": "error", "message": f"Falha ao conectar no IP {ip}. Verifique os cabos e a sub-rede."})

# 💡 Nova função auxiliar em background para salvar e restaurar os pontos
def gerenciar_execucao_segura():
    # 1. Faz um backup profundo da lista antes que o adaptador consuma os pontos
    backup_pontos = copy.deepcopy(adapter.lista_pontos)
    
    # 2. Executa o movimento real isolado na thread nativa do sistema
    tpool.execute(adapter.executar_trajetoria)
    
    # 3. Quando o movimento termina, restaura a lista original para o adaptador
    adapter.lista_pontos = backup_pontos
    
    # 4. Força o Socket.IO a atualizar a lista na tela do front-end
    socketio.emit('atualizar_pontos', {"pontos": backup_pontos})

@app.route('/api/trajetoria/executar', methods=['POST'])
def executar_trajetoria():
    # 💡 MUDANÇA: Agora chamamos a nossa função de gerenciamento seguro em background
    eventlet.spawn(gerenciar_execucao_segura)
    
    eventlet.sleep(0.01)
    return jsonify({"status": "success", "message": "Trajetória iniciada em background."})

@app.route('/api/trajetoria/limpar', methods=['POST'])
def limpar_trajetoria():
    adapter.lista_pontos = []
    return jsonify({"status": "success", "message": "Trajetória limpa"})

@app.route('/api/ponto/adicionar', methods=['POST'])
def adicionar_ponto_manual():
    tipo = request.json.get('tipo', 'L')
    adapter.salvar_ponto_atual(tipo)
    return jsonify({"status": "success"})

# Envio assíncrono do estado do robô via Socket.IO
def disparar_update_via_websocket(dados):
    socketio.emit('atualizar_estado', dados)
    eventlet.sleep(0)

adapter.on_state_update = disparar_update_via_websocket
adapter.on_point_saved = lambda pts: socketio.emit('atualizar_pontos', {"pontos": pts})

if __name__ == '__main__':
    # Inicializa o hardware padrão do robô e o joystick antes do servidor subir
    adapter.conectar("192.168.0.200")
    adapter.iniciar_loop_controle()
    
    print("\n[PAYBACK HMI] Servidor online. Acesse http://localhost:5000 no navegador.")
    # Executamos o app diretamente via socketio usando o servidor modificado
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)