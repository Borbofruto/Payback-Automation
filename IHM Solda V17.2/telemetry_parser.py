# Proxy da V17.2 para o modulo real em Programas de Robos/JAKA/Solda.

from pathlib import Path
import importlib.util

_REAL = Path(__file__).resolve().parents[1] / "Programas de Robôs" / "JAKA" / "Solda" / "telemetry_parser.py"
_SPEC = importlib.util.spec_from_file_location("_payback_real_telemetry_parser", _REAL)
_MOD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MOD)

globals().update({k: v for k, v in vars(_MOD).items() if not k.startswith("__")})
