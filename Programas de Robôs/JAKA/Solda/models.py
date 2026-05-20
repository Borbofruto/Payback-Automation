# models.py
# Modelos puros da IHM de Solda Payback.
# Não conhece Flask, Socket.IO, pygame nem SDK JAKA.

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple

Pose = List[float]
RawPoint = Tuple[str, Pose]


class PointKind(str, Enum):
    LINEAR = "L"
    CIRCULAR = "C"


@dataclass(frozen=True)
class Waypoint:
    kind: PointKind
    pose: Pose
    index: int  # zero-based

    @property
    def label(self) -> str:
        return f"#{self.index + 1}"

    def as_frontend(self) -> List[Any]:
        return [self.kind.value, list(self.pose)]


class SegmentKind(str, Enum):
    LINEAR = "L"
    CIRCULAR = "C"


@dataclass(frozen=True)
class TrajectorySegment:
    kind: SegmentKind
    target: Optional[Waypoint] = None
    start: Optional[Waypoint] = None
    mid: Optional[Waypoint] = None
    end: Optional[Waypoint] = None
    note: str = ""


@dataclass
class TrajectoryPlan:
    ok: bool
    points: List[Waypoint] = field(default_factory=list)
    segments: List[TrajectorySegment] = field(default_factory=list)
    entry: Optional[Waypoint] = None
    exit: Optional[Waypoint] = None
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def message(self) -> str:
        if self.ok:
            return "Trajetória válida."
        return self.errors[0] if self.errors else "Trajetória inválida."

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "message": self.message,
            "errors": list(self.errors),
            "warnings": list(self.warnings),
            "segments": [segment_to_dict(s) for s in self.segments],
            "point_count": len(self.points),
        }


def segment_to_dict(seg: TrajectorySegment) -> Dict[str, Any]:
    if seg.kind == SegmentKind.LINEAR:
        return {
            "kind": "L",
            "target": seg.target.index + 1 if seg.target else None,
            "note": seg.note,
        }
    return {
        "kind": "C",
        "start": seg.start.index + 1 if seg.start else None,
        "mid": seg.mid.index + 1 if seg.mid else None,
        "end": seg.end.index + 1 if seg.end else None,
        "note": seg.note,
    }


def normalize_raw_points(raw_points: Sequence[Any]) -> Tuple[List[Waypoint], List[str]]:
    """Converte lista vinda do frontend/adapter em Waypoints tipados.

    Formato aceito: [("L", [x,y,z,rx,ry,rz]), ("C", [...])]
    Também tolera listas JSON: [["L", [...]], ["C", [...]]].
    """
    points: List[Waypoint] = []
    errors: List[str] = []

    for idx, raw in enumerate(raw_points or []):
        if not isinstance(raw, (list, tuple)) or len(raw) < 2:
            errors.append(f"Ponto #{idx + 1} inválido: esperado [tipo, pose].")
            continue

        tipo_raw = str(raw[0]).upper().strip()
        try:
            kind = PointKind(tipo_raw)
        except Exception:
            errors.append(f"Ponto #{idx + 1} tem tipo inválido '{tipo_raw}'. Use L ou C.")
            continue

        pose_raw = raw[1]
        if not isinstance(pose_raw, (list, tuple)) or len(pose_raw) < 6:
            errors.append(f"Ponto #{idx + 1} tem pose inválida. Esperado [x,y,z,rx,ry,rz] absolutos.")
            continue

        try:
            pose = [float(v) for v in pose_raw[:6]]
        except Exception:
            errors.append(f"Ponto #{idx + 1} contém valores não numéricos na pose.")
            continue

        points.append(Waypoint(kind=kind, pose=pose, index=idx))

    return points, errors
