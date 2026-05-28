# app.py
# Backend Flask/Socket.IO — IHM Solda Payback — V17.2 parâmetros operacionais.
# HTML fica na raiz do projeto.
# Patch: fallback HTTP /api/estado + interface de workspace/mesa injetada na tela Parâmetros.

import os
import eventlet

eventlet.monkey_patch()

from eventlet import tpool
from flask import Flask, jsonify, request, make_response
from flask_socketio import SocketIO

from robot_adapter import adapter

app = Flask(__name__, template_folder='.')
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

DEFAULT_ROBOT_IP = os.environ.get('JAKA_IP', '10.5.5.100')


HMI_POLLING_PATCH = r'''
<style id="hmi-workspace-style">
  .workspace-card .workspace-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
  .workspace-card .workspace-field{display:flex;flex-direction:column;gap:4px}
  .workspace-card label{font-size:11px;font-weight:800;color:var(--navy);text-transform:uppercase;letter-spacing:.05em}
  .workspace-card input{height:34px;border:1px solid var(--line);border-radius:8px;padding:0 10px;font-weight:800;color:var(--navy);background:#fff}
  .workspace-card .workspace-line{display:flex;align-items:center;justify-content:space-between;gap:8px;margin:8px 0}
  .workspace-card .workspace-point{font-size:11px;background:#f7fafc;border:1px solid var(--line);border-radius:8px;padding:8px;color:var(--muted);line-height:1.35}
  .workspace-card .workspace-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .workspace-card .workspace-full{grid-column:1/-1}
  .workspace-card .workspace-status{border-radius:10px;padding:10px;margin-top:10px;font-size:12px;font-weight:800;line-height:1.35}
  .workspace-card .workspace-status.ok{background:#e8fff5;border:1px solid #8ee0be;color:#0b6b45}
  .workspace-card .workspace-status.warn{background:#fff5dc;border:1px solid #ffc966;color:#7a4a00}
  .workspace-card .workspace-status.err{background:#ffe8e8;border:1px solid #ff9d9d;color:#8a1111}
  .workspace-card .workspace-toggle{display:flex;align-items:center;gap:8px;font-size:12px;font-weight:900;color:var(--navy)}
  .workspace-card .workspace-toggle input{height:auto;width:auto}
  #workspace-lock-modal{position:fixed;inset:0;z-index:9999;display:none;align-items:center;justify-content:center;background:rgba(0,20,40,.42);backdrop-filter:blur(2px)}
  #workspace-lock-modal.show{display:flex}
  #workspace-lock-modal .box{width:min(560px,92vw);background:#fff;border-radius:18px;border:2px solid #ff9d9d;box-shadow:0 24px 80px rgba(0,0,0,.24);padding:22px;text-align:left}
  #workspace-lock-modal h2{margin:0 0 8px;color:#8a1111;font-size:20px;letter-spacing:.02em}
  #workspace-lock-modal p{margin:0 0 14px;color:#243b53;font-size:14px;line-height:1.45;font-weight:700}
  #workspace-lock-modal .meta{background:#fff5f5;border:1px solid #ffd0d0;border-radius:12px;padding:10px;margin-top:10px;color:#8a1111;font-size:12px;font-weight:900}
  #workspace-lock-modal button{margin-top:14px;border:0;border-radius:999px;padding:10px 18px;font-weight:900;background:#0aaed0;color:white;cursor:pointer}
</style>
<script id="hmi-polling-fallback">
(function(){
  if (window.__paybackPollingFallbackInstalled) return;
  window.__paybackPollingFallbackInstalled = true;

  let ultimoWorkspaceLocked = false;

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

  function fmtPose3(p) {
    if (!Array.isArray(p)) return 'não definido';
    return 'X ' + Number(p[0]||0).toFixed(1) + ' · Y ' + Number(p[1]||0).toFixed(1) + ' · Z ' + Number(p[2]||0).toFixed(1);
  }

  function ensureWorkspaceModal() {
    if (document.getElementById('workspace-lock-modal')) return;
    var m = document.createElement('div');
    m.id = 'workspace-lock-modal';
    m.innerHTML = '<div class="box"><h2>TCP fora da área de trabalho</h2><p>O controle por joystick foi bloqueado. Use Drag Mode / Free Drive para recolocar manualmente o TCP dentro da área da mesa. Quando o TCP voltar para dentro, o controle será liberado automaticamente.</p><div id="workspace-lock-meta" class="meta"></div><button onclick="document.getElementById(\'workspace-lock-modal\').classList.remove(\'show\')">Entendi</button></div>';
    document.body.appendChild(m);
  }

  function atualizarWorkspaceModal(ws) {
    ensureWorkspaceModal();
    var status = ws && ws.status ? ws.status : null;
    var locked = !!(status && status.enabled && status.configured && status.jog_locked);
    var modal = document.getElementById('workspace-lock-modal');
    var meta = document.getElementById('workspace-lock-meta');
    if (locked) {
      if (meta) meta.textContent = (status.message || 'TCP fora da área.') + ' Eixos: ' + ((status.outside_axes || []).join(', ') || 'fora dos limites');
      if (!ultimoWorkspaceLocked && modal) modal.classList.add('show');
    } else if (modal) {
      modal.classList.remove('show');
    }
    ultimoWorkspaceLocked = locked;
  }

  function atualizarWorkspaceUI(ws) {
    try {
      criarWorkspaceUI();
      if (!ws) return;
      var cfg = ws.config || {};
      var status = ws.status || {};
      var limits = status.limits || null;
      var enabled = document.getElementById('ws-enabled');
      var zmargin = document.getElementById('ws-z-margin');
      var xymargin = document.getElementById('ws-xy-margin');
      var slow = document.getElementById('ws-slow-zone');
      if (enabled) enabled.checked = !!cfg.enabled;
      if (zmargin && cfg.z_margin_mm != null) zmargin.value = cfg.z_margin_mm;
      if (xymargin && cfg.xy_margin_mm != null) xymargin.value = cfg.xy_margin_mm;
      if (slow && cfg.slow_zone_mm != null) slow.value = cfg.slow_zone_mm;

      var p1 = document.getElementById('ws-p1-val');
      var p2 = document.getElementById('ws-p2-val');
      if (p1) p1.textContent = fmtPose3(cfg.p1);
      if (p2) p2.textContent = fmtPose3(cfg.p2);

      var st = document.getElementById('ws-status');
      if (st) {
        st.className = 'workspace-status ' + (!cfg.enabled ? 'warn' : status.inside ? 'ok' : 'err');
        var txt = status.message || 'Workspace não configurado.';
        if (limits) {
          txt += '\nX: ' + limits.x_min_safe.toFixed(1) + ' até ' + limits.x_max_safe.toFixed(1) +
                 ' · Y: ' + limits.y_min_safe.toFixed(1) + ' até ' + limits.y_max_safe.toFixed(1) +
                 ' · Z mínimo TCP: ' + limits.z_min_tcp.toFixed(1);
        }
        st.textContent = txt;
      }
      atualizarWorkspaceModal(ws);
    } catch(e) { console.warn('[workspace] falha UI', e); }
  }

  window.atualizarWorkspaceUI = atualizarWorkspaceUI;

  async function apiJson(url, opts) {
    var r = await fetch(url, Object.assign({headers:{'Content-Type':'application/json'}, cache:'no-store'}, opts || {}));
    var data = await r.json().catch(function(){ return {}; });
    if (!r.ok) throw new Error(data.message || data.error || ('HTTP ' + r.status));
    return data;
  }

  async function carregarWorkspace() {
    try {
      var data = await apiJson('/api/workspace');
      atualizarWorkspaceUI(data.workspace || data);
    } catch(e) { console.warn('[workspace] carregar falhou', e); }
  }
  window.carregarWorkspace = carregarWorkspace;

  async function salvarWorkspace() {
    try {
      var payload = {
        enabled: !!document.getElementById('ws-enabled')?.checked,
        z_margin_mm: Number(document.getElementById('ws-z-margin')?.value || 10),
        xy_margin_mm: Number(document.getElementById('ws-xy-margin')?.value || 5),
        slow_zone_mm: Number(document.getElementById('ws-slow-zone')?.value || 30)
      };
      var data = await apiJson('/api/workspace', {method:'POST', body:JSON.stringify(payload)});
      atualizarWorkspaceUI(data.workspace || data);
      if (typeof toast === 'function') toast('Workspace aplicado.');
    } catch(e) { alert('Erro ao salvar workspace: ' + e.message); }
  }
  window.salvarWorkspace = salvarWorkspace;

  async function capturarWorkspacePonto(ponto) {
    try {
      var data = await apiJson('/api/workspace/ponto/' + ponto, {method:'POST'});
      atualizarWorkspaceUI(data.workspace || data);
      if (typeof toast === 'function') toast(String(ponto).toUpperCase() + ' capturado pelo TCP atual.');
    } catch(e) { alert('Erro ao capturar ' + ponto + ': ' + e.message); }
  }
  window.capturarWorkspacePonto = capturarWorkspacePonto;

  function criarWorkspaceUI() {
    if (document.getElementById('workspace-card')) return;
    var side = document.querySelector('#page-parametros .param-side') || document.querySelector('#page-parametros .param-main');
    if (!side) return;
    var card = document.createElement('div');
    card.className = 'card workspace-card';
    card.id = 'workspace-card';
    card.innerHTML = `
      <div class="cttl"><i class="fa-solid fa-border-all"></i> Área de trabalho / mesa</div>
      <div class="workspace-line">
        <label class="workspace-toggle"><input id="ws-enabled" type="checkbox"> Habilitar limites da mesa</label>
      </div>
      <div class="workspace-point"><strong>P1</strong><br><span id="ws-p1-val">não definido</span></div>
      <div class="workspace-point"><strong>P2</strong><br><span id="ws-p2-val">não definido</span></div>
      <div class="workspace-actions">
        <button class="btn btn-o" onclick="capturarWorkspacePonto('p1')"><i class="fa-solid fa-location-crosshairs"></i> Capturar P1</button>
        <button class="btn btn-o" onclick="capturarWorkspacePonto('p2')"><i class="fa-solid fa-location-crosshairs"></i> Capturar P2</button>
      </div>
      <div class="workspace-grid">
        <div class="workspace-field"><label>Margem Z <span>mm</span></label><input id="ws-z-margin" type="number" min="0" max="300" step="1" value="10"></div>
        <div class="workspace-field"><label>Margem XY <span>mm</span></label><input id="ws-xy-margin" type="number" min="0" max="300" step="1" value="5"></div>
        <div class="workspace-field workspace-full"><label>Zona lenta perto do limite <span>mm</span></label><input id="ws-slow-zone" type="number" min="1" max="500" step="1" value="30"></div>
      </div>
      <div id="ws-status" class="workspace-status warn">Workspace ainda não configurado.</div>
      <div class="workspace-actions">
        <button class="btn btn-p workspace-full" onclick="salvarWorkspace()"><i class="fa-solid fa-check"></i> Aplicar workspace</button>
      </div>
      <div class="param-help">P1 e P2 são cantos opostos da mesa no frame da base do robô. Z da superfície = média do Z dos dois pontos. Se o TCP sair da área, o joystick é bloqueado e o retorno deve ser por Drag Mode / Free Drive.</div>`;
    side.appendChild(card);
    carregarWorkspace();
  }
  window.criarWorkspaceUI = criarWorkspaceUI;

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
      if (dados.workspace) atualizarWorkspaceUI(dados.workspace);
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

  setTimeout(function(){ criarWorkspaceUI(); pollEstado(); }, 120);
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
            if 'id="hmi-polling-fallback"' not in html:
                if '</body>' in html:
                    html = html.replace('</body>', HMI_POLLING_PATCH + '\n</body>')
                else:
                    html += HMI_POLLING_PATCH
            return _response_no_cache(html)
    return ('index.html não encontrado. Coloque o arquivo como index.html na raiz ou em templates/Index.html.', 500)


@app.route('/api/estado', methods=['GET'])
def obter_estado():
    return jsonify(adapter.snapshot_state())


@app.route('/api/pontos', methods=['GET'])
def obter_pontos():
    return jsonify({'status': 'success', 'pontos': adapter._get_pontos_snapshot()})


@app.route('/api/workspace', methods=['GET'])
def obter_workspace():
    if not hasattr(adapter, 'get_workspace_config'):
        return jsonify({'status': 'error', 'message': 'Workspace não disponível no adapter atual.'}), 500
    return jsonify({'status': 'success', 'workspace': adapter.get_workspace_config()})


@app.route('/api/workspace', methods=['POST'])
def atualizar_workspace():
    if not hasattr(adapter, 'set_workspace_config'):
        return jsonify({'status': 'error', 'message': 'Workspace não disponível no adapter atual.'}), 500
    dados = request.get_json(silent=True) or {}
    try:
        workspace = adapter.set_workspace_config(dados)
        socketio.emit('atualizar_estado', adapter.snapshot_state())
        socketio.emit('execucao_status', {'message': 'Workspace da mesa atualizado.', 'status': 'info'})
        return jsonify({'status': 'success', 'workspace': workspace})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e), 'workspace': adapter.get_workspace_config()}), 400


@app.route('/api/workspace/ponto/<ponto>', methods=['POST'])
def capturar_workspace_ponto(ponto):
    if not hasattr(adapter, 'set_workspace_point_from_tcp'):
        return jsonify({'status': 'error', 'message': 'Workspace não disponível no adapter atual.'}), 500
    try:
        workspace = adapter.set_workspace_point_from_tcp(ponto)
        socketio.emit('atualizar_estado', adapter.snapshot_state())
        socketio.emit('execucao_status', {'message': f'{ponto.upper()} da mesa capturado pelo TCP atual.', 'status': 'info'})
        return jsonify({'status': 'success', 'workspace': workspace})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e), 'workspace': adapter.get_workspace_config()}), 400


@app.route('/api/config/velocidades', methods=['POST'])
def configurar_velocidades():
    dados = request.get_json(silent=True) or {}
    try:
        adapter.vel_reproducao = float(dados.get('reproducao', adapter.vel_reproducao))
        adapter.vel_aproximacao = float(dados.get('aproximacao', adapter.vel_aproximacao))
        return jsonify({'status': 'success', 'reproducao': adapter.vel_reproducao, 'aproximacao': adapter.vel_aproximacao})
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
        return jsonify({'status': 'error', 'message': 'SDK JAKA indisponível. Verifique SDK_DIR, .dll e .pyd.'}), 500
    return jsonify({'status': 'error', 'message': f'Falha ao conectar no IP {ip}. Verifique cabos, sub-rede e controlador.'}), 500


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
        payload = {'status': 'trajectory_error', 'message': plan.message, 'errors': plan.errors, 'plan': plan.to_dict()}
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
        socketio.emit('trajectory_error', {'status': 'trajectory_error', 'message': 'Erro de trajetória pendente.', 'errors': adapter.obter_erros_trajetoria()})
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
