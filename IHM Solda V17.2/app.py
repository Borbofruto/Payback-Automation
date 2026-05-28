# app.py
# Backend Flask/Socket.IO — IHM Solda Payback — V17.2 parâmetros operacionais.
# HTML fica na raiz do projeto.
# Patch: fallback HTTP /api/estado + UI de workspace + desenho do workspace na trajetória.

import os
import time
import eventlet

eventlet.monkey_patch()

from eventlet import tpool
from flask import Flask, jsonify, request, make_response
from flask_socketio import SocketIO
from types import MethodType

from robot_adapter import adapter
import robot_adapter as robot_adapter_mod

app = Flask(__name__, template_folder='.')
app.config['SECRET_KEY'] = 'payback_industrial_secret_2026'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

DEFAULT_ROBOT_IP = os.environ.get('JAKA_IP', '10.5.5.100')


def _apply_directional_workspace_jog_patch():
    """Permite jog de retorno quando o TCP já está fora do workspace.

    Regra operacional:
    - dentro da área: usa o limitador normal;
    - fora da área: permite apenas comandos cujo vetor reduza a distância até a área válida;
    - comandos que afastam mais ou tangenciam sem ajudar continuam bloqueados.
    """
    if getattr(adapter, '_workspace_directional_jog_patch_applied', False):
        return

    original_limiter = getattr(adapter, '_limitar_velocidade_workspace', None)
    if not callable(original_limiter):
        return

    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def safe_limits(self):
        cfg = getattr(self, 'workspace', {}) or {}
        if not cfg.get('enabled'):
            return None, cfg
        try:
            return robot_adapter_mod._workspace_limits_from_cfg(cfg), cfg
        except Exception:
            return None, cfg

    def local_error_vector(limits, tcp):
        s, t = robot_adapter_mod._workspace_coords_xy(limits, float(tcp[0]), float(tcp[1]))
        z = float(tcp[2])
        target_s = clamp(s, limits['s_min_safe'], limits['s_max_safe'])
        target_t = clamp(t, limits['t_min_safe'], limits['t_max_safe'])
        target_z = max(z, limits['z_min_tcp'])
        return {
            's': s,
            't': t,
            'z': z,
            'err_s': target_s - s,
            'err_t': target_t - t,
            'err_z': target_z - z,
        }

    def is_inside_error(err):
        return abs(err['err_s']) < 1e-9 and abs(err['err_t']) < 1e-9 and abs(err['err_z']) < 1e-9

    def local_motion(limits, eixo, vel):
        eixo = int(eixo)
        vel = float(vel)
        ds = dt = dz = 0.0
        if eixo == 0:
            ds = robot_adapter_mod._dot2([vel, 0.0], limits['u_edge'])
            dt = robot_adapter_mod._dot2([vel, 0.0], limits['n_depth'])
        elif eixo == 1:
            ds = robot_adapter_mod._dot2([0.0, vel], limits['u_edge'])
            dt = robot_adapter_mod._dot2([0.0, vel], limits['n_depth'])
        elif eixo == 2:
            dz = vel
        else:
            return None
        return ds, dt, dz

    def record_directional_block(self, motivo, eixo, vel, err=None):
        try:
            with self._state_lock:
                self.diagnosticos['workspace_bloqueio'] = {
                    'motivo': motivo,
                    'eixo': int(eixo),
                    'vel_original': float(vel),
                    'erro_retorno': err,
                    'ts': time.time(),
                }
        except Exception:
            pass
        return 0.0

    def directional_limiter(self, eixo, vel):
        limits, cfg = safe_limits(self)
        if not cfg.get('enabled'):
            return float(vel)

        try:
            self._refresh_tcp_from_robot('workspace_limit')
        except Exception:
            pass

        limits, cfg = safe_limits(self)
        if limits is None:
            return record_directional_block(self, 'workspace inválido ou não configurado', eixo, vel)

        tcp = self._copy_tcp()
        err = local_error_vector(limits, tcp)
        if is_inside_error(err):
            return original_limiter(int(eixo), float(vel))

        vel = float(vel)
        if abs(vel) < 1e-9:
            return 0.0

        motion = local_motion(limits, eixo, vel)
        if motion is None:
            return record_directional_block(self, 'TCP fora da área; rotação bloqueada até retornar', eixo, vel, err)

        ds, dt, dz = motion
        retorno = ds * err['err_s'] + dt * err['err_t'] + dz * err['err_z']
        if retorno > 1e-9:
            try:
                self._workspace_stop_emitido = False
                with self._state_lock:
                    self.diagnosticos['workspace_retorno_por_jog'] = {
                        'eixo': int(eixo),
                        'vel': vel,
                        'retorno_score': retorno,
                        'erro_retorno': err,
                        'ts': time.time(),
                    }
            except Exception:
                pass
            return vel

        return record_directional_block(self, 'TCP fora da área; movimento não aponta para dentro', eixo, vel, err)

    adapter._limitar_velocidade_workspace = MethodType(directional_limiter, adapter)
    adapter._workspace_directional_jog_patch_applied = True


_apply_directional_workspace_jog_patch()


HMI_POLLING_PATCH = r'''
<style id="hmi-workspace-style">
  .workspace-card .workspace-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
  .workspace-card .workspace-field{display:flex;flex-direction:column;gap:4px}
  .workspace-card label{font-size:11px;font-weight:800;color:var(--navy);text-transform:uppercase;letter-spacing:.05em}
  .workspace-card input{height:34px;border:1px solid var(--line);border-radius:8px;padding:0 10px;font-weight:800;color:var(--navy);background:#fff}
  .workspace-card .workspace-point{font-size:11px;background:#f7fafc;border:1px solid var(--line);border-radius:8px;padding:8px;color:var(--muted);line-height:1.35}
  .workspace-card .workspace-actions{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .workspace-card .workspace-full{grid-column:1/-1}
  .workspace-card .workspace-status{white-space:pre-line;border-radius:10px;padding:10px;margin-top:10px;font-size:12px;font-weight:800;line-height:1.35}
  .workspace-card .workspace-status.ok{background:#e8fff5;border:1px solid #8ee0be;color:#0b6b45}
  .workspace-card .workspace-status.warn{background:#fff5dc;border:1px solid #ffc966;color:#7a4a00}
  .workspace-card .workspace-status.err{background:#ffe8e8;border:1px solid #ff9d9d;color:#8a1111}
  #workspace-lock-modal{position:fixed;inset:0;z-index:9999;display:none;align-items:center;justify-content:center;background:rgba(0,20,40,.42);backdrop-filter:blur(2px)}
  #workspace-lock-modal.show{display:flex}
  #workspace-lock-modal .box{width:min(460px,92vw);background:#fff;border-radius:18px;border:2px solid #ff9d9d;box-shadow:0 24px 80px rgba(0,0,0,.24);padding:20px;text-align:left}
  #workspace-lock-modal h2{margin:0 0 8px;color:#8a1111;font-size:19px;letter-spacing:.02em}
  #workspace-lock-modal p{margin:0 0 10px;color:#243b53;font-size:14px;line-height:1.35;font-weight:800}
  #workspace-lock-modal .meta{background:#fff5f5;border:1px solid #ffd0d0;border-radius:12px;padding:8px;margin-top:8px;color:#8a1111;font-size:12px;font-weight:900}
  #workspace-lock-modal button{margin-top:12px;border:0;border-radius:999px;padding:9px 16px;font-weight:900;background:#0aaed0;color:white;cursor:pointer}
</style>
<script id="hmi-polling-fallback">
(function(){
  if (window.__paybackPollingFallbackInstalled) return;
  window.__paybackPollingFallbackInstalled = true;

  let ultimoWorkspaceLocked = false;

  function safeCall(fnName, arg) {
    try { if (typeof window[fnName] === 'function') window[fnName](arg); }
    catch (e) { console.warn('[HMI fallback] falha em', fnName, e); }
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

  function fmt(n) { return Number(n || 0).toFixed(1); }

  function ensureWorkspaceModal() {
    if (document.getElementById('workspace-lock-modal')) return;
    var m = document.createElement('div');
    m.id = 'workspace-lock-modal';
    m.innerHTML = '<div class="box"><h2>TCP fora da área</h2><p>Volte para dentro com o controle ou Free Drive.</p><div id="workspace-lock-meta" class="meta"></div><button onclick="document.getElementById(\'workspace-lock-modal\').classList.remove(\'show\')">Entendi</button></div>';
    document.body.appendChild(m);
  }

  function atualizarWorkspaceModal(ws) {
    ensureWorkspaceModal();
    var status = ws && ws.status ? ws.status : null;
    var locked = !!(status && status.enabled && status.configured && status.jog_locked);
    var modal = document.getElementById('workspace-lock-modal');
    var meta = document.getElementById('workspace-lock-meta');
    if (locked) {
      if (meta) meta.textContent = 'Permitido: mover em direção à área. Eixos: ' + ((status.outside_axes || []).join(', ') || 'fora dos limites');
      if (!ultimoWorkspaceLocked && modal) modal.classList.add('show');
    } else if (modal) {
      modal.classList.remove('show');
    }
    ultimoWorkspaceLocked = locked;
  }

  function textoWorkspace(status, cfg) {
    cfg = cfg || {};
    status = status || {};
    var limits = status.limits || null;
    var txt = status.jog_locked ? 'TCP fora da área. Mova de volta para dentro.' : (status.message || 'Workspace não configurado.');
    if (limits) {
      txt += '\nLargura P1–P2: ' + fmt(limits.edge_length_mm) + ' mm';
      txt += ' · Profundidade até base: ' + fmt(limits.depth_mm) + ' mm';
      txt += '\nZ superfície mesa: ' + fmt(limits.z_surface);
      txt += ' · Margem Z: ' + fmt(limits.z_margin_mm);
      txt += ' · Z limite TCP: ' + fmt(limits.z_min_tcp);
      if (status.workspace_coords) {
        txt += '\nTCP local na mesa: S ' + fmt(status.workspace_coords.s_mm) + ' mm · T ' + fmt(status.workspace_coords.t_mm) + ' mm';
      }
    }
    if (!cfg.enabled) txt += '\nClique em Aplicar workspace para habilitar os limites.';
    return txt;
  }

  function getWorkspaceLimits() {
    var ws = window.__paybackWorkspace;
    return ws && ws.status && ws.status.limits ? ws.status.limits : null;
  }

  function instalarWorkspaceTrajectoryPatch() {
    if (window.__paybackWsTrajectoryPatchInstalled) return;
    if (typeof canvas === 'undefined' || typeof ctx === 'undefined') return;
    if (typeof desenharVisualizacao !== 'function' || typeof projectPoseToCanvas !== 'function') return;
    window.__paybackWsTrajectoryPatchInstalled = true;

    var originalDesenhar = desenharVisualizacao;
    var originalProject = projectPoseToCanvas;
    var ROBOT_DIAMETER_MM = 160;
    var ROBOT_RADIUS_MM = ROBOT_DIAMETER_MM / 2;
    var RED = 'rgba(220, 38, 38, 1)';

    function centerOfVertices(vertices) {
      var sx = 0, sy = 0;
      vertices.forEach(function(p){ sx += Number(p[0] || 0); sy += Number(p[1] || 0); });
      return [sx / vertices.length, sy / vertices.length];
    }

    function rotateAroundWorkspace(p, center) {
      var r = ((viewRot % 4) + 4) % 4;
      var x = Number(p[0] || 0), y = Number(p[1] || 0);
      var dx = x - center[0], dy = y - center[1];
      if (r === 0) return [center[0] + dx, center[1] + dy];
      if (r === 1) return [center[0] + dy, center[1] - dx];
      if (r === 2) return [center[0] - dx, center[1] - dy];
      return [center[0] - dy, center[1] + dx];
    }

    function buildWorkspaceTransform() {
      var limits = getWorkspaceLimits();
      if (!limits || !Array.isArray(limits.vertices_xy) || limits.vertices_xy.length !== 4) return null;
      var rect = canvas.getBoundingClientRect();
      var w = rect.width || 700;
      var h = rect.height || 520;
      var wsCenter = centerOfVertices(limits.vertices_xy);
      var pts = limits.vertices_xy.slice();
      pts.push([0, 0], [ROBOT_RADIUS_MM, 0], [-ROBOT_RADIUS_MM, 0], [0, ROBOT_RADIUS_MM], [0, -ROBOT_RADIUS_MM]);
      var rot = pts.map(function(p){ return rotateAroundWorkspace(p, wsCenter); });
      var xs = rot.map(function(p){ return p[0]; });
      var ys = rot.map(function(p){ return p[1]; });
      var minX = Math.min.apply(null, xs), maxX = Math.max.apply(null, xs);
      var minY = Math.min.apply(null, ys), maxY = Math.max.apply(null, ys);
      var pad = 54;
      var worldW = Math.max(1, maxX - minX);
      var worldH = Math.max(1, maxY - minY);
      var scale = Math.max(0.02, Math.min((w - pad*2) / worldW, (h - pad*2) / worldH));
      return { limits: limits, wsCenter: wsCenter, scale: scale, screenCenter: [w/2, h/2], worldCenter: [(minX + maxX)/2, (minY + maxY)/2], w: w, h: h };
    }

    function worldToScreenXY(p, t) {
      var rp = rotateAroundWorkspace(p, t.wsCenter);
      return { x: t.screenCenter[0] + (rp[0] - t.worldCenter[0]) * t.scale, y: t.screenCenter[1] - (rp[1] - t.worldCenter[1]) * t.scale };
    }

    projectPoseToCanvas = function(p) {
      var t = buildWorkspaceTransform();
      if (!t) return originalProject(p);
      escala = t.scale;
      return worldToScreenXY([Number(p?.[0] || 0), Number(p?.[1] || 0)], t);
    };
    window.projectPoseToCanvas = projectPoseToCanvas;

    rotacionarVistaTrajetoria = function() {
      viewRot = (viewRot + 1) % 4;
      var el = document.getElementById('view-angle-label');
      if (el) el.innerText = (viewRot * 90) + '°';
      desenharVisualizacao();
    };
    window.rotacionarVistaTrajetoria = rotacionarVistaTrajetoria;

    function drawWorkspaceMesa(t) {
      var vertices = t.limits.vertices_xy;
      var screenPts = vertices.map(function(p){ return worldToScreenXY(p, t); });
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(screenPts[0].x, screenPts[0].y);
      for (var i = 1; i < screenPts.length; i++) ctx.lineTo(screenPts[i].x, screenPts[i].y);
      ctx.closePath();
      ctx.fillStyle = 'rgba(0, 174, 214, 0.07)';
      ctx.fill();
      ctx.strokeStyle = 'rgba(0, 174, 214, 0.95)';
      ctx.lineWidth = 2;
      ctx.stroke();

      var robot = worldToScreenXY([0, 0], t);
      var rr = ROBOT_RADIUS_MM * t.scale;
      ctx.beginPath();
      ctx.arc(robot.x, robot.y, rr, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(220, 38, 38, 0.12)';
      ctx.fill();
      ctx.strokeStyle = RED;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = RED;
      ctx.font = '700 ' + Math.max(9, Math.min(18, rr * 0.38)) + 'px Segoe UI';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText('Robô', robot.x, robot.y);
      ctx.restore();
    }

    function drawGrid(w, h, t) {
      ctx.strokeStyle = '#EEF3F8';
      ctx.lineWidth = 1;
      for (var x = 0; x < w; x += 40) { ctx.beginPath(); ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke(); }
      for (var y = 0; y < h; y += 40) { ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke(); }
      var zero = worldToScreenXY([0,0], t);
      ctx.strokeStyle = '#DDE4EC';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(0, zero.y); ctx.lineTo(w, zero.y); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(zero.x, 0); ctx.lineTo(zero.x, h); ctx.stroke();
    }

    desenharVisualizacao = function() {
      if (!ctx || !canvas) return;
      var t = buildWorkspaceTransform();
      if (!t) return originalDesenhar();

      escala = t.scale;
      offset_x = t.screenCenter[0] - t.worldCenter[0] * t.scale;
      offset_y = t.screenCenter[1] + t.worldCenter[1] * t.scale;

      ctx.clearRect(0, 0, t.w, t.h);
      drawGrid(t.w, t.h, t);
      drawWorkspaceMesa(t);

      var pts = estadoLocal.pontos || [];
      if (pts.length > 0) {
        var currentPose = pose6(pts[0]?.[1]);
        desenharMarcadorPose(currentPose, 'P1 (entrada)', '#003865', 6);
        var i = 1;
        if (String(pts[0]?.[0] || '').toUpperCase() === 'C') {
          if (pts.length >= 3 && String(pts[1]?.[0] || '').toUpperCase() === 'C' && String(pts[2]?.[0] || '').toUpperCase() === 'C') {
            ctx.strokeStyle = '#E0A100'; ctx.lineWidth = 3;
            drawJakaMoveCArc(pose6(pts[0][1]), pose6(pts[1][1]), pose6(pts[2][1]));
            desenharMarcadorPose(pose6(pts[1][1]), 'C2', '#E0A100', 5);
            desenharMarcadorPose(pose6(pts[2][1]), 'C3', '#E0A100', 5);
            currentPose = pose6(pts[2][1]);
            i = 3;
          }
        }
        while (i < pts.length) {
          var tipo = String(pts[i]?.[0] || 'L').toUpperCase();
          var pose = pose6(pts[i]?.[1]);
          if (tipo === 'C') {
            if (i + 2 < pts.length && String(pts[i+1]?.[0] || '').toUpperCase() === 'C' && String(pts[i+2]?.[0] || '').toUpperCase() === 'C') {
              desenharLinhaPose(currentPose, pose, '#CC3340', 2.5);
              desenharMarcadorPose(pose, 'C' + (i+1) + ' início', '#E0A100', 5);
              ctx.strokeStyle = '#E0A100'; ctx.lineWidth = 3;
              drawJakaMoveCArc(pose, pose6(pts[i+1][1]), pose6(pts[i+2][1]));
              desenharMarcadorPose(pose6(pts[i+1][1]), 'C' + (i+2) + ' passagem', '#E0A100', 5);
              desenharMarcadorPose(pose6(pts[i+2][1]), 'C' + (i+3) + ' fim', '#E0A100', 5);
              currentPose = pose6(pts[i+2][1]);
              i += 3;
            } else {
              desenharLinhaPose(currentPose, pose, '#CC3340', 2);
              desenharMarcadorPose(pose, 'C' + (i+1) + ' incompleto', '#CC3340', 5);
              currentPose = pose;
              i += 1;
            }
          } else {
            desenharLinhaPose(currentPose, pose, '#CC3340', 3);
            desenharMarcadorPose(pose, 'L' + (i+1), '#28323D', 5);
            currentPose = pose;
            i += 1;
          }
        }
      }

      var tcp = pose6(estadoLocal.tcp);
      var tcpP = projectPoseToCanvas(tcp);
      ctx.strokeStyle = '#00AED6';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tcpP.x - 18, tcpP.y); ctx.lineTo(tcpP.x + 18, tcpP.y); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(tcpP.x, tcpP.y - 18); ctx.lineTo(tcpP.x, tcpP.y + 18); ctx.stroke();
      ctx.beginPath(); ctx.arc(tcpP.x, tcpP.y, 5, 0, 2*Math.PI); ctx.fillStyle = '#003865'; ctx.fill(); ctx.stroke();
    };
    window.desenharVisualizacao = desenharVisualizacao;
  }
  window.instalarWorkspaceTrajectoryPatch = instalarWorkspaceTrajectoryPatch;

  function atualizarWorkspaceUI(ws) {
    try {
      window.__paybackWorkspace = ws;
      criarWorkspaceUI();
      instalarWorkspaceTrajectoryPatch();
      if (!ws) return;
      var cfg = ws.config || {};
      var status = ws.status || {};
      var zmargin = document.getElementById('ws-z-margin');
      var xymargin = document.getElementById('ws-xy-margin');
      var slow = document.getElementById('ws-slow-zone');
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
        st.textContent = textoWorkspace(status, cfg);
      }
      atualizarWorkspaceModal(ws);
      if (typeof desenharVisualizacao === 'function' && typeof telaAtual !== 'undefined' && telaAtual === 'trajetoria') desenharVisualizacao();
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
        enabled: true,
        z_margin_mm: Number(document.getElementById('ws-z-margin')?.value || 0.5),
        xy_margin_mm: Number(document.getElementById('ws-xy-margin')?.value || 5),
        slow_zone_mm: Number(document.getElementById('ws-slow-zone')?.value || 30)
      };
      var data = await apiJson('/api/workspace', {method:'POST', body:JSON.stringify(payload)});
      atualizarWorkspaceUI(data.workspace || data);
      if (typeof toast === 'function') toast('Workspace aplicado e limites habilitados.');
    } catch(e) { alert('Erro ao aplicar workspace: ' + e.message); }
  }
  window.salvarWorkspace = salvarWorkspace;

  async function desabilitarWorkspace() {
    try {
      var payload = {
        enabled: false,
        z_margin_mm: Number(document.getElementById('ws-z-margin')?.value || 0.5),
        xy_margin_mm: Number(document.getElementById('ws-xy-margin')?.value || 5),
        slow_zone_mm: Number(document.getElementById('ws-slow-zone')?.value || 30)
      };
      var data = await apiJson('/api/workspace', {method:'POST', body:JSON.stringify(payload)});
      atualizarWorkspaceUI(data.workspace || data);
      if (typeof toast === 'function') toast('Limites da mesa desabilitados.');
    } catch(e) { alert('Erro ao desabilitar workspace: ' + e.message); }
  }
  window.desabilitarWorkspace = desabilitarWorkspace;

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
      <div class="workspace-point"><strong>P1</strong><br><span id="ws-p1-val">não definido</span></div>
      <div class="workspace-point"><strong>P2</strong><br><span id="ws-p2-val">não definido</span></div>
      <div class="workspace-actions">
        <button class="btn btn-o" onclick="capturarWorkspacePonto('p1')"><i class="fa-solid fa-location-crosshairs"></i> Capturar P1</button>
        <button class="btn btn-o" onclick="capturarWorkspacePonto('p2')"><i class="fa-solid fa-location-crosshairs"></i> Capturar P2</button>
      </div>
      <div class="workspace-grid">
        <div class="workspace-field"><label>Margem Z <span>mm</span></label><input id="ws-z-margin" type="number" min="0" max="300" step="0.1" value="0.5"></div>
        <div class="workspace-field"><label>Margem XY <span>mm</span></label><input id="ws-xy-margin" type="number" min="0" max="300" step="1" value="5"></div>
        <div class="workspace-field workspace-full"><label>Zona lenta perto do limite <span>mm</span></label><input id="ws-slow-zone" type="number" min="1" max="500" step="1" value="30"></div>
      </div>
      <div id="ws-status" class="workspace-status warn">Workspace ainda não configurado.</div>
      <div class="workspace-actions">
        <button class="btn btn-p workspace-full" onclick="salvarWorkspace()"><i class="fa-solid fa-check"></i> Aplicar workspace e habilitar limites</button>
        <button class="btn btn-o workspace-full" onclick="desabilitarWorkspace()"><i class="fa-solid fa-ban"></i> Desabilitar limites</button>
      </div>
      <div class="param-help">P1 e P2 definem a borda distante da mesa. A base do robô (0,0) define a profundidade por projeção perpendicular. Z da superfície = menor Z entre P1 e P2. Fora da área, o controle só aceita movimentos de retorno.</div>`;
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

    try { if (dados.diagnosticos && typeof atualizarDiagnosticosReais === 'function') atualizarDiagnosticosReais(dados.diagnosticos); } catch(e) {}
    try { if (dados.parametros && typeof atualizarParametrosUI === 'function') atualizarParametrosUI(dados.parametros, false); } catch(e) {}
    try { if (typeof dados.controle_manual_pausado !== 'undefined' && typeof atualizarStatusControleManual === 'function') atualizarStatusControleManual(!!dados.controle_manual_pausado); } catch(e) {}
    try { if (dados.workspace) atualizarWorkspaceUI(dados.workspace); } catch(e) {}
    try { if (typeof desenharVisualizacao === 'function') desenharVisualizacao(); } catch(e) {}
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

  setTimeout(function(){ criarWorkspaceUI(); instalarWorkspaceTrajectoryPatch(); pollEstado(); }, 120);
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
    candidates = [base / 'index.html', base / 'Index.html', base / 'templates' / 'index.html', base / 'templates' / 'Index.html']
    for candidate in candidates:
        if candidate.exists():
            html = candidate.read_text(encoding='utf-8', errors='ignore')
            if 'id="hmi-polling-fallback"' not in html:
                html = html.replace('</body>', HMI_POLLING_PATCH + '\n</body>') if '</body>' in html else html + HMI_POLLING_PATCH
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
        msg = 'Workspace da mesa habilitado.' if workspace.get('config', {}).get('enabled') else 'Workspace da mesa desabilitado.'
        socketio.emit('execucao_status', {'message': msg, 'status': 'info'})
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
