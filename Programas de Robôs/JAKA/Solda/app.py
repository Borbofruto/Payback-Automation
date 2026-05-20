# app.py
# Backend Flask/Socket.IO — IHM Solda Payback — V12
# Mantém o HTML atual e conecta com robot_adapter.py.

# CRÍTICO: eventlet precisa ser importado e aplicar monkey_patch antes dos demais imports de rede/thread.
import eventlet
eventlet.monkey_patch()

from eventlet import tpool
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO
from robot_adapter import adapter
import copy

app = Flask(__name__, template_folder='.')
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'

socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')


@app.route('/')
def index():
    # O HTML fica na raiz do projeto, como combinado.
    return render_template('index.html')


@app.route('/api/config/velocidades', methods=['POST'])
def configurar_velocidades():
    dados = request.get_json(silent=True) or {}

    try:
        adapter.vel_reproducao = float(dados.get('reproducao', adapter.vel_reproducao))
        adapter.vel_aproximacao = float(dados.get('aproximacao', adapter.vel_aproximacao))
        return jsonify({
            'status': 'success',
            'reproducao': adapter.vel_reproducao,
            'aproximacao': adapter.vel_aproximacao,
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Velocidade inválida: {e}'}), 400


@app.route('/api/robo/conectar', methods=['POST'])
def conectar_robo():
    dados = request.get_json(silent=True) or {}
    ip = dados.get('ip', '192.168.0.200')

    sucesso = adapter.conectar(ip)

    if sucesso and not adapter.modo_simulacao:
        return jsonify({'status': 'success', 'message': f'Conectado ao robô físico no IP {ip}.'})

    if not adapter.sdk_carregado():
        return jsonify({
            'status': 'error',
            'message': 'Falha: SDK JAKA indisponível. Verifique .dll/.pyd e SDK_DIR.',
        }), 500

    return jsonify({
        'status': 'error',
        'message': f'Falha ao conectar no IP {ip}. Verifique cabos, sub-rede e estado do controlador.',
    }), 500


def gerenciar_execucao_segura():
    """Executa a trajetória fora do handler HTTP e preserva a lista de pontos na HMI."""
    backup_pontos = adapter._get_pontos_snapshot()

    try:
        resultado = tpool.execute(adapter.executar_trajetoria)
    except Exception as e:
        resultado = f'Erro crítico ao iniciar/executar trajetória: {e}'

    adapter._set_pontos(backup_pontos)
    socketio.emit('atualizar_pontos', {'pontos': backup_pontos})
    socketio.emit('execucao_status', {'message': resultado})


@app.route('/api/trajetoria/executar', methods=['POST'])
def executar_trajetoria():
    if adapter.executando_trajetoria:
        return jsonify({'status': 'error', 'message': 'Já existe uma trajetória em execução.'}), 409

    # Validação síncrona antes de enviar qualquer comando ao robô.
    # Isso evita casos perigosos de bloco circular incompleto ou mistura L/C ambígua.
    try:
        pontos = adapter._get_pontos_snapshot()
        segmentos, msg = adapter._validar_e_planejar_trajetoria(pontos)
        if segmentos is None:
            return jsonify({'status': 'error', 'message': msg}), 400
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Falha validando trajetória: {e}'}), 400

    eventlet.spawn(gerenciar_execucao_segura)
    eventlet.sleep(0.01)
    return jsonify({'status': 'success', 'message': 'Trajetória iniciada em background.'})


@app.route('/api/trajetoria/limpar', methods=['POST'])
def limpar_trajetoria():
    adapter._set_pontos([])
    socketio.emit('atualizar_pontos', {'pontos': []})
    return jsonify({'status': 'success', 'message': 'Trajetória limpa.'})


@app.route('/api/ponto/adicionar', methods=['POST'])
def adicionar_ponto_manual():
    dados = request.get_json(silent=True) or {}
    tipo = dados.get('tipo', 'L')
    if tipo not in ('L', 'C'):
        return jsonify({'status': 'error', 'message': 'Tipo de ponto inválido. Use L ou C.'}), 400

    adapter.salvar_ponto_atual(tipo)
    return jsonify({'status': 'success', 'message': f'Ponto {tipo} adicionado.'})


# Envio assíncrono do estado do robô via Socket.IO
def disparar_update_via_websocket(dados):
    socketio.emit('atualizar_estado', dados)
    eventlet.sleep(0)


adapter.on_state_update = disparar_update_via_websocket
adapter.on_point_saved = lambda pts: socketio.emit('atualizar_pontos', {'pontos': pts})


@socketio.on('connect')
def on_connect():
    # Empurra estado inicial para a tela assim que o cliente conecta.
    socketio.emit('atualizar_pontos', {'pontos': adapter._get_pontos_snapshot()})
    socketio.emit('atualizar_estado', {
        'tcp': adapter._copy_tcp(),
        'pontos': adapter._get_pontos_snapshot(),
        'modo_sim': adapter.modo_simulacao,
        'angulo_operador': adapter.angulo_operador,
        'diagnosticos': copy.deepcopy(adapter.diagnosticos),
    })


if __name__ == '__main__':
    adapter.conectar('192.168.0.200')
    adapter.iniciar_loop_controle()

    print('\n[PAYBACK HMI] Servidor online. Acesse http://localhost:5000 no navegador.')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
