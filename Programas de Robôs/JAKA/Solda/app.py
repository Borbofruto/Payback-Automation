# app.py
# Backend Flask/Socket.IO — IHM Solda Payback — V15 absoluto + modal de erro.
# HTML fica na raiz do projeto.

import eventlet

eventlet.monkey_patch()

from eventlet import tpool
from flask import Flask, jsonify, request, send_file
from flask_socketio import SocketIO

from robot_adapter import adapter

app = Flask(__name__, template_folder='.')
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')


@app.route('/')
def index():
    # Aceita o HTML tanto na raiz quanto em /templates e tolera Index.html maiúsculo.
    # Isso evita TemplateNotFound por diferença de maiúscula/minúscula ou pasta.
    from pathlib import Path
    base = Path(__file__).resolve().parent
    candidates = [
        base / 'index.html',
        base / 'Index.html',
        base / 'templates' / 'index.html',
        base / 'templates' / 'Index.html',
    ]
    for candidate in candidates:
        if candidate.exists():
            return send_file(str(candidate))
    return (
        'index.html não encontrado. Coloque o arquivo como index.html na raiz ou em templates/Index.html.',
        500,
    )


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
            'message': 'SDK JAKA indisponível. Verifique SDK_DIR, .dll e .pyd.',
        }), 500
    return jsonify({
        'status': 'error',
        'message': f'Falha ao conectar no IP {ip}. Verifique cabos, sub-rede e controlador.',
    }), 500


@app.route('/api/trajetoria/validar', methods=['GET', 'POST'])
def validar_trajetoria():
    plan = adapter.validar_trajetoria_atual()
    status_code = 200 if plan.ok else 400
    return jsonify(plan.to_dict()), status_code


def gerenciar_execucao_segura():
    backup_pontos = adapter._get_pontos_snapshot()
    try:
        resultado = tpool.execute(adapter.executar_trajetoria)
        status = 'success' if 'sucesso' in str(resultado).lower() else 'error'
    except Exception as e:
        resultado = f'Erro crítico ao executar trajetória: {e}'
        status = 'error'
    adapter._set_pontos(backup_pontos)
    socketio.emit('atualizar_pontos', {'pontos': backup_pontos})
    socketio.emit('execucao_status', {'message': resultado, 'status': status})


@app.route('/api/trajetoria/executar', methods=['POST'])
def executar_trajetoria():
    if adapter.executando_trajetoria:
        return jsonify({'status': 'error', 'message': 'Já existe uma trajetória em execução.'}), 409

    plan = adapter.validar_trajetoria_atual()
    if not plan.ok:
        # Erro de validação em tentativa de execução é estado bloqueante:
        # o joystick fica ignorado até o operador reconhecer o modal na IHM.
        adapter.definir_erro_trajetoria(plan.errors)
        payload = {
            'status': 'trajectory_error',
            'message': plan.message,
            'errors': plan.errors,
            'plan': plan.to_dict(),
        }
        socketio.emit('trajectory_error', payload)
        socketio.emit('execucao_status', {'message': plan.message, 'status': 'error'})
        return jsonify(payload), 400

    eventlet.spawn(gerenciar_execucao_segura)
    eventlet.sleep(0.01)
    return jsonify({'status': 'success', 'message': 'Trajetória validada e iniciada em background.', 'plan': plan.to_dict()})


@app.route('/api/trajetoria/limpar', methods=['POST'])
def limpar_trajetoria():
    adapter._set_pontos([])
    socketio.emit('atualizar_pontos', {'pontos': []})
    socketio.emit('execucao_status', {'message': 'Trajetória limpa.', 'status': 'info'})
    return jsonify({'status': 'success', 'message': 'Trajetória limpa.'})


@app.route('/api/ponto/adicionar', methods=['POST'])
def adicionar_ponto_manual():
    dados = request.get_json(silent=True) or {}
    tipo = str(dados.get('tipo', 'L')).upper()
    if tipo not in ('L', 'C'):
        return jsonify({'status': 'error', 'message': 'Tipo de ponto inválido. Use L ou C.'}), 400
    adapter.salvar_ponto_atual(tipo)
    plan = adapter.validar_trajetoria_atual()
    return jsonify({'status': 'success', 'message': f'Ponto {tipo} adicionado.', 'plan': plan.to_dict()})



@app.route('/api/trajetoria/erro/ack', methods=['POST'])
def ack_erro_trajetoria():
    adapter.limpar_erro_trajetoria()
    socketio.emit('execucao_status', {'message': 'Erro de trajetória reconhecido. Controle liberado.', 'status': 'info'})
    return jsonify({'status': 'success', 'message': 'Erro reconhecido. Controle liberado.'})

@app.route('/api/debug/telemetria', methods=['GET'])
def debug_telemetria():
    return jsonify(adapter.get_telemetry_debug())


@app.route('/api/debug/trajetoria', methods=['GET'])
def debug_trajetoria():
    return jsonify(adapter.validar_trajetoria_atual().to_dict())


def disparar_update_via_websocket(dados):
    socketio.emit('atualizar_estado', dados)
    eventlet.sleep(0)


adapter.on_state_update = disparar_update_via_websocket
adapter.on_point_saved = lambda pts: socketio.emit('atualizar_pontos', {'pontos': pts})
adapter.on_execution_status = lambda dados: socketio.emit('execucao_status', dados)


@socketio.on('connect')
def on_connect():
    socketio.emit('atualizar_pontos', {'pontos': adapter._get_pontos_snapshot()})
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    if getattr(adapter, 'trajetoria_em_erro', False):
        socketio.emit('trajectory_error', {
            'status': 'trajectory_error',
            'message': 'Erro de trajetória pendente.',
            'errors': adapter.obter_erros_trajetoria(),
        })
    socketio.emit('execucao_status', {'message': 'Backend conectado à IHM.', 'status': 'info'})


if __name__ == '__main__':
    adapter.conectar('192.168.0.200')
    adapter.iniciar_loop_controle()
    print('\n[PAYBACK HMI] Servidor online. Acesse http://localhost:5000 no navegador.')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
