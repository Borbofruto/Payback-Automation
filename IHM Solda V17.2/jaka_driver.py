# Proxy da V17.2 para o modulo real em Programas de Robos/JAKA/Solda.
# Mantem esta pasta leve sem duplicar o driver.

from pathlib import Path
import importlib.util

_REAL = Path(__file__).resolve().parents[1] / "Programas de Robôs" / "JAKA" / "Solda" / "jaka_driver.py"
_SPEC = importlib.util.spec_from_file_location("_payback_real_jaka_driver", _REAL)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)

JakaDriver = _MOD.JakaDriver
SDK_DIR = getattr(_MOD, "SDK_DIR", None)
SDK_AVAILABLE = getattr(_MOD, "SDK_AVAILABLE", False)
SDK_ERROR = getattr(_MOD, "SDK_ERROR", None)
