# app.py
# Backend Flask/Socket.IO — IHM Solda Payback — V17.2 parâmetros operacionais.
# HTML fica na raiz do projeto.
# Patch: fallback HTTP /api/estado para manter HTML sincronizado mesmo se Socket.IO falhar.

import os
import eventlet

eventlet.monkey_patch()

from eventlet import tpool
from flask import Flask, jsonify, request, send_file, make_response
from flask_socketio import SocketIO

from robot_adapter import adapter

app = Flask(__name__, template_folder='.')
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

DEFAULT_ROBOT_IP = os.environ.get('JAKA_IP', '10.5.5.100')


HMI_POLLING_PATCH = r'''
<script id="hmi-polling-fallback">
(function(){
  if (window.__paybackPollingFallbackInstalled) return;
  window.__paybackPollingFallbackInstalled = true;

  function safeCall(fnName, arg) {
    try {
      if (typeof window[fnName] === 'function') window[fnName](arg);
    } catch (e) {
      console.warn('[HMI fallback] falha em', fnName, e);
    }
  }

  function setBadgeOnline(modoSim) {
    try {
      var badge = document.getElementById('status-badge');
      if (!badge) return;
      if (modoSim) {
        badge.className = 'pill pill-sim';
        badge.innerHTML = '<span class="dot pulse"></span> Modo simulação';
      } else {
        badge.className = 'pill pill-on';
        badge.innerHTML = '<span class="dot"></span> Robô conectado';
      }
    } catch(e) {}
  }

  function aplicarEstado(dados) {
    if (!dados) return;

    try {
      if (typeof estadoLocal !== 'undefined') {
        if (Array.isArray(dados.tcp)) estadoLocal.tcp = dados.tcp;
        if (Array.isArray(dados.pontos)) estadoLocal.pontos = dados.pontos;
      }
    } catch(e) {}

    if (Array.isArray(dados.tcp)) safeCall('atualizarTcpNumerico', dados.tcp);
    if (Array.isArray(dados.pontos)) safeCall('renderizarListaPontos', dados.pontos);

    setBadgeOnline(!!dados.modo_sim);

    try {
      if (dados.diagnosticos && typeof atualizarDiagnosticosReais === 'function') {
        atualizarDiagnosticosReais(dados.diagnosticos);
      }
    } catch(e) {}

    try {
      if (dados.parametros && typeof atualizarParametrosUI === 'function') {
        atualizarParametrosUI(dados.parametros, false);
      }
    } catch(e) {}

    try {
      if (typeof dados.controle_manual_pausado !== 'undefined' && typeof atualizarStatusControleManual === 'function') {
        atualizarStatusControleManual(!!dados.controle_manual_pausado);
      }
    } catch(e) {}

    try {
      if (typeof desenharVisualizacao === 'function') desenharVisualizacao();
    } catch(e) {}
  }

  window.__paybackAplicarEstado = aplicarEstado;

  async function pollEstado() {
    try {
      var r = await fetch('/api/estado?_=' + Date.now(), { cache: 'no-store' });
      if (!r.ok) return;
      var dados = await r.json();
      aplicarEstado(dados);

      try {
        var el = document.getElementById('socket-state');
        if (el) { el.className = 'trend-badge ok'; el.innerText = 'HTTP OK'; }
      } catch(e) {}
    } catch(e) {
      try {
        var el = document.getElementById('socket-state');
        if (el) { el.className = 'trend-badge attention'; el.innerText = 'Offline'; }
      } catch(_) {}
    }
  }

  setTimeout(pollEstado, 120);
  setInterval(pollEstado, 180);
})();
</script>
'''


def _response_no_cache(body: str):
    resp = make_response(body)
    resp.headers['Content-Type'] = 'text/html; charset=utf-8'
    resp.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    return resp


@app.route('/')
def index():
    # Aceita o HTML tanto na raiz quanto em /templates e tolera Index.html maiúsculo.
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
            html = candidate.read_text(encoding='utf-8', errors='ignore')
            # Injeta fallback depois do JS principal, antes de fechar body.
            if 'id="hmi-polling-fallback"' not in html:
                if '</body>' in html:
                    html = html.replace('</body>', HMI_POLLING_PATCH + '\n</body>')
                else:
                    html += HMI_POLLING_PATCH
            return _response_no_cache(html)
    return (
        'index.html não encontrado. Coloque o arquivo como index.html na raiz ou em templates/Index.html.',
        500,
    )


@app.route('/api/estado', methods=['GET'])
def obter_estado():
    """Snapshot único usado por fallback HTTP e debug.

    Se o Socket.IO não chegar ao navegador, esta rota mantém TCP/pontos/diagnósticos
    sincronizados no localhost.
    """
    return jsonify(adapter.snapshot_state())


@app.route('/api/pontos', methods=['GET'])
def obter_pontos():
    return jsonify({'status': 'success', 'pontos': adapter._get_pontos_snapshot()})


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


@app.route('/api/parametros', methods=['GET'])
def obter_parametros():
    return jsonify({'status': 'success', 'parametros': adapter.get_parametros_operacionais()})


@app.route('/api/parametros', methods=['POST'])
def atualizar_parametros():
    dados = request.get_json(silent=True) or {}
    try:
        params = adapter.set_parametros_operacionais(dados)
        socketio.emit('atualizar_estado', adapter.snapshot_state())
        socketio.emit('execucao_status', {'message': 'Parâmetros atualizados. Controle manual permanece pausado até liberação.', 'status': 'warn'})
        return jsonify({'status': 'success', 'parametros': params, 'message': 'Parâmetros atualizados.'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e), 'parametros': adapter.get_parametros_operacionais()}), 400


@app.route('/api/controle/pausar', methods=['POST'])
def pausar_controle():
    dados = request.get_json(silent=True) or {}
    motivo = dados.get('motivo', 'edição de parâmetros')
    params = adapter.pausar_controle_manual(motivo)
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    socketio.emit('execucao_status', {'message': 'Controle manual pausado para edição de parâmetros.', 'status': 'warn'})
    return jsonify({'status': 'success', 'parametros': params, 'message': 'Controle manual pausado.'})


@app.route('/api/controle/retomar', methods=['POST'])
def retomar_controle():
    params = adapter.retomar_controle_manual()
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    socketio.emit('execucao_status', {'message': 'Controle manual liberado.', 'status': 'info'})
    return jsonify({'status': 'success', 'parametros': params, 'message': 'Controle manual liberado.'})


@app.route('/api/robo/conectar', methods=['POST'])
def conectar_robo():
    dados = request.get_json(silent=True) or {}
    ip = dados.get('ip', DEFAULT_ROBOT_IP)
    sucesso = adapter.conectar(ip)
    socketio.emit('atualizar_estado', adapter.snapshot_state())
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
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    socketio.emit('execucao_status', {'message': resultado, 'status': status})


@app.route('/api/trajetoria/executar', methods=['POST'])
def executar_trajetoria():
    if adapter.executando_trajetoria:
        return jsonify({'status': 'error', 'message': 'Já existe uma trajetória em execução.'}), 409

    plan = adapter.validar_trajetoria_atual()
    if not plan.ok:
        adapter.definir_erro_trajetoria(plan.errors)
        payload = {
            'status': 'trajectory_error',
            'message': plan.message,
            'errors': plan.errors,
            'plan': plan.to_dict(),
        }
        socketio.emit('trajectory_error', payload)
        socketio.emit('execucao_status', {'message': plan.message, 'status': 'error'})
        socketio.emit('atualizar_estado', adapter.snapshot_state())
        return jsonify(payload), 400

    eventlet.spawn(gerenciar_execucao_segura)
    eventlet.sleep(0.01)
    return jsonify({'status': 'success', 'message': 'Trajetória validada e iniciada em background.', 'plan': plan.to_dict()})


@app.route('/api/trajetoria/limpar', methods=['POST'])
def limpar_trajetoria():
    adapter.limpar_pontos()
    socketio.emit('atualizar_pontos', {'pontos': []})
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    socketio.emit('execucao_status', {'message': 'Trajetória limpa e erro de trajetória liberado.', 'status': 'info'})
    return jsonify({'status': 'success', 'message': 'Trajetória limpa.'})


@app.route('/api/trajetoria/remover_ultimo', methods=['POST'])
def remover_ultimo_ponto():
    removed = adapter.remover_ultimo_ponto()
    pts = adapter._get_pontos_snapshot()
    socketio.emit('atualizar_pontos', {'pontos': pts})
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    msg = 'Último ponto removido.' if removed else 'Não há pontos para remover.'
    socketio.emit('execucao_status', {'message': msg, 'status': 'info' if removed else 'warn'})
    return jsonify({'status': 'success' if removed else 'empty', 'message': msg, 'pontos': pts})


@app.route('/api/ponto/adicionar', methods=['POST'])
def adicionar_ponto_manual():
    dados = request.get_json(silent=True) or {}
    tipo = str(dados.get('tipo', 'L')).upper()
    if tipo not in ('L', 'C'):
        return jsonify({'status': 'error', 'message': 'Tipo de ponto inválido. Use L ou C.'}), 400
    adapter.salvar_ponto_atual(tipo)
    plan = adapter.validar_trajetoria_atual()
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    return jsonify({'status': 'success', 'message': f'Ponto {tipo} adicionado.', 'plan': plan.to_dict()})


@app.route('/api/trajetoria/erro/ack', methods=['POST'])
def ack_erro_trajetoria():
    adapter.limpar_erro_trajetoria()
    socketio.emit('atualizar_estado', adapter.snapshot_state())
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


def disparar_pontos_via_websocket(pts):
    payload = {'pontos': pts}
    socketio.emit('atualizar_pontos', payload)
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    eventlet.sleep(0)


adapter.on_state_update = disparar_update_via_websocket
adapter.on_point_saved = disparar_pontos_via_websocket
adapter.on_execution_status = lambda dados: socketio.emit('execucao_status', dados)
adapter.on_trajectory_error = lambda dados: socketio.emit('trajectory_error', dados)


@socketio.on('connect')
def on_connect():
    print('[SOCKET] Cliente conectado à IHM.')
    socketio.emit('atualizar_pontos', {'pontos': adapter._get_pontos_snapshot()})
    socketio.emit('atualizar_estado', adapter.snapshot_state())
    if getattr(adapter, 'trajetoria_em_erro', False):
        socketio.emit('trajectory_error', {
            'status': 'trajectory_error',
            'message': 'Erro de trajetória pendente.',
            'errors': adapter.obter_erros_trajetoria(),
        })
    socketio.emit('execucao_status', {'message': 'Backend conectado à IHM.', 'status': 'info'})


@socketio.on('disconnect')
def on_disconnect():
    print('[SOCKET] Cliente desconectado da IHM.')


if __name__ == '__main__':
    print(f'[PAYBACK HMI] Tentando conectar robô em {DEFAULT_ROBOT_IP}...')
    adapter.conectar(DEFAULT_ROBOT_IP)
    adapter.iniciar_loop_controle()
    print('\n[PAYBACK HMI] Servidor online. Acesse http://localhost:5000 no navegador.')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
