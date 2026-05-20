# trajectory_planner.py
# Validação e planejamento sequencial de trajetórias L/C.
# Regra de segurança: valida tudo antes de qualquer comando real de robô.

from typing import Any, Sequence, List

from models import PointKind, SegmentKind, TrajectoryPlan, TrajectorySegment, normalize_raw_points


class TrajectoryPlanner:
    """Planejador puro para trajetórias mistas, com pontos ABSOLUTOS.

    Semântica operacional:
    - Todo ponto salvo é pose absoluta [x,y,z,rx,ry,rz].
    - A trajetória completa precisa de pelo menos 2 pontos para haver deslocamento.
    - O primeiro ponto é consumido pela fase de entrada: aproximação + descida.
    - Ponto L consome 1 ponto e gera MoveL absoluto até aquele ponto.
    - Ponto C sempre define arco circular em grupos de 3 pontos C consecutivos:
        C_start, C_mid, C_end.
      Se o arco começa no ponto #1, a fase de entrada já colocou o TCP em C_start.
      Se o arco aparece no meio, o executor primeiro faz MoveL absoluto até C_start
      e depois MoveC usando C_mid e C_end.
    - Qualquer C incompleto bloqueia a execução antes de enviar movimento ao robô.
    """

    @staticmethod
    def plan(raw_points: Sequence[Any]) -> TrajectoryPlan:
        points, parse_errors = normalize_raw_points(raw_points)
        errors: List[str] = list(parse_errors)

        if len(points) < 2:
            errors.append("Necessário ao menos 2 pontos para executar uma trajetória.")

        # Validação global de todos os blocos C, sem retornar no primeiro erro.
        i = 0
        while i < len(points):
            p = points[i]
            if p.kind != PointKind.CIRCULAR:
                i += 1
                continue

            # Um bloco circular SEMPRE começa no C atual e exige mais dois C.
            faltantes = []
            if i + 1 >= len(points) or points[i + 1].kind != PointKind.CIRCULAR:
                faltantes.append(i + 2)
            if i + 2 >= len(points) or points[i + 2].kind != PointKind.CIRCULAR:
                faltantes.append(i + 3)

            if faltantes:
                errors.append(
                    f"Ponto #{i + 1} é circular, mas não há 3 pontos C consecutivos "
                    "para formar uma trajetória circular completa: início, passagem e fim."
                )
                i += 1
            else:
                i += 3

        if errors:
            return TrajectoryPlan(
                ok=False,
                points=points,
                entry=points[0] if points else None,
                exit=points[-1] if points else None,
                errors=errors,
            )

        plan = TrajectoryPlan(ok=True, points=points, entry=points[0], exit=points[-1])

        # Construção sequencial do plano. Neste ponto a validade já foi garantida.
        if points[0].kind == PointKind.CIRCULAR:
            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.CIRCULAR,
                    start=points[0],
                    mid=points[1],
                    end=points[2],
                    note="Arco inicial; fase de entrada posiciona no ponto inicial absoluto.",
                )
            )
            i = 3
        else:
            i = 1

        while i < len(points):
            p = points[i]
            if p.kind == PointKind.LINEAR:
                plan.segments.append(
                    TrajectorySegment(
                        kind=SegmentKind.LINEAR,
                        target=p,
                        note="MoveL absoluto sequencial.",
                    )
                )
                i += 1
                continue

            # p é C e já sabemos que há p, p+1, p+2 como C.
            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.LINEAR,
                    target=p,
                    note="MoveL absoluto até o início do arco circular.",
                )
            )
            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.CIRCULAR,
                    start=p,
                    mid=points[i + 1],
                    end=points[i + 2],
                    note="MoveC absoluto sequencial de 3 pontos C.",
                )
            )
            i += 3

        if not plan.segments:
            return TrajectoryPlan(
                ok=False,
                points=points,
                entry=points[0],
                exit=points[-1],
                errors=["Trajetória sem segmento executável após o ponto inicial."],
            )

        return plan
