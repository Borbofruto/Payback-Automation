# sitecustomize.py — hotfix V17.2
#
# Python carrega este arquivo automaticamente quando app.py é executado nesta pasta.
# O objetivo aqui é aplicar um patch pós-import no robot_adapter sem reescrever o
# wrapper inteiro: remove completamente a heurística de abortar trajetória porque
# o TCP ficou parado. Parada do robô NÃO é erro por si só; programas reais podem
# pausar, esperar entrada digital ou executar movimentos lentos.
#
# Regra final:
# - joystick segue bloqueado durante trajetória;
# - solda segue desligando em falha real/timeout/finalização;
# - trajetória só aborta por status real de falha/parada do robô ou timeout total;
# - nunca aborta só porque o TCP ficou parado.

import importlib.abc
import importlib.machinery
import sys
import time
from types import MethodType


_PATCH_FLAG = "_trajectory_wait_no_stationary_abort_patch_applied"


def _patch_robot_adapter_module(module):
    adapter = getattr(module, "adapter", None)
    if adapter is None or getattr(adapter, _PATCH_FLAG, False):
        return

    def _aguardar_chegada_sem_abort_por_parada(self, alvo, tol_mm=2.0, timeout_s=120.0, ciclos_estaveis=5, exigir_inpos=False):
        if getattr(self, "modo_simulacao", True):
            return True

        t0 = time.time()
        stable = 0
        last = None

        while time.time() - t0 < timeout_s:
            try:
                self._refresh_tcp_from_robot("traj_wait")
            except Exception:
                pass

            fault_reasons = []
            try:
                fault_reasons = self._probe_robot_fault()
            except Exception as e:
                try:
                    with self._state_lock:
                        self.diagnosticos["robot_status_probe_error"] = str(e)
                except Exception:
                    pass

            if fault_reasons:
                print(f"[TRAJ] Interrompida por status do robô: {fault_reasons}")
                try:
                    self._marcar_interrupcao_trajetoria(
                        "status de falha/parada do robô durante trajetória",
                        fault_reasons,
                    )
                except Exception:
                    pass
                return False

            atual = self._copy_tcp()
            dist = self._dist_xyz(atual, alvo)
            delta = self._dist_xyz(atual, last) if last is not None else 999999.0
            last = atual

            inpos_ok = True
            if exigir_inpos:
                try:
                    inpos = self._is_in_pos()
                    inpos_ok = True if inpos is None else bool(inpos)
                except Exception:
                    inpos_ok = True

            if dist <= tol_mm and delta <= 0.35 and inpos_ok:
                stable += 1
                if stable >= ciclos_estaveis:
                    return True
            else:
                stable = 0

            # Intencionalmente NÃO existe critério de "robô parado fora do alvo" aqui.
            # Parado pode ser pausa legítima, espera de IO, transição lenta ou leitura de TCP repetida.
            time.sleep(0.03)

        print(f"[TRAJ] Timeout aguardando chegada ao último ponto após {timeout_s:.1f}s.")
        try:
            self._marcar_interrupcao_trajetoria(
                "timeout aguardando chegada ao último ponto",
                [f"timeout={timeout_s}s"],
            )
        except Exception:
            pass
        return False

    adapter.aguardar_chegada_por_tcp = MethodType(_aguardar_chegada_sem_abort_por_parada, adapter)
    setattr(adapter, _PATCH_FLAG, True)
    print("[HOTFIX] Abort por 'robô parado' removido; trajetória aborta só por status de falha ou timeout.")


class _PatchLoader(importlib.abc.Loader):
    def __init__(self, wrapped_loader):
        self.wrapped_loader = wrapped_loader

    def create_module(self, spec):
        if hasattr(self.wrapped_loader, "create_module"):
            return self.wrapped_loader.create_module(spec)
        return None

    def exec_module(self, module):
        self.wrapped_loader.exec_module(module)
        _patch_robot_adapter_module(module)


class _RobotAdapterPatchFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != "robot_adapter":
            return None

        # Evita recursão: pergunta aos outros finders.
        for finder in sys.meta_path:
            if finder is self:
                continue
            spec = finder.find_spec(fullname, path, target) if hasattr(finder, "find_spec") else None
            if spec and spec.loader:
                spec.loader = _PatchLoader(spec.loader)
                return spec
        return None


# Se robot_adapter já estiver carregado por algum cenário raro, aplica agora.
if "robot_adapter" in sys.modules:
    _patch_robot_adapter_module(sys.modules["robot_adapter"])
else:
    sys.meta_path.insert(0, _RobotAdapterPatchFinder())
