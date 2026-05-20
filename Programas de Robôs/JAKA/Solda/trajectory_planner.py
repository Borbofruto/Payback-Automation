# trajectory_planner.py
# Validação e planejamento sequencial de trajetórias L/C.
# Regra de segurança: valida tudo antes de qualquer comando real de robô.

from typing import Any, Sequence

from models import PointKind, SegmentKind, TrajectoryPlan, TrajectorySegment, normalize_raw_points


class TrajectoryPlanner:
    """Planejador puro para trajetórias mistas.

    Semântica adotada:
    - A trajetória completa precisa ter pelo menos 2 pontos.
    - O primeiro ponto é consumido pela fase de entrada: aproximação + descida.
    - Ponto L consome 1 ponto e gera MoveL até aquele ponto.
    - Ponto C gera sempre um arco com exatamente 3 pontos C consecutivos:
        início, passagem, fim.
      Se o arco começa no primeiro ponto da trajetória, a fase de entrada já colocou
      o TCP no início. Caso contrário, o executor primeiro faz MoveL até o C inicial.
    - Blocos C incompletos bloqueiam a execução antes de enviar movimento ao robô.
    """

    @staticmethod
    def plan(raw_points: Sequence[Any]) -> TrajectoryPlan:
        points, errors = normalize_raw_points(raw_points)
        if errors:
            return TrajectoryPlan(ok=False, points=points, errors=errors)

        if len(points) < 2:
            return TrajectoryPlan(
                ok=False,
                points=points,
                errors=["Necessário ao menos 2 pontos para executar uma trajetória."],
            )

        plan = TrajectoryPlan(ok=True, points=points, entry=points[0], exit=points[-1])

        # Caso especial: a trajetória começa com um arco circular.
        if points[0].kind == PointKind.CIRCULAR:
            if len(points) < 3 or points[1].kind != PointKind.CIRCULAR or points[2].kind != PointKind.CIRCULAR:
                return TrajectoryPlan(
                    ok=False,
                    points=points,
                    entry=points[0],
                    exit=points[-1],
                    errors=[
                        "Trajetória circular iniciada no ponto #1 precisa de 3 pontos C consecutivos: "
                        "início, passagem e fim."
                    ],
                )
            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.CIRCULAR,
                    start=points[0],
                    mid=points[1],
                    end=points[2],
                    note="Arco inicial; fase de entrada posiciona no ponto inicial.",
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
                        note="Movimento linear sequencial.",
                    )
                )
                i += 1
                continue

            # Ponto C no meio da trajetória: C_i é o início do arco.
            if i + 2 >= len(points):
                faltam = (i + 2) - (len(points) - 1)
                return TrajectoryPlan(
                    ok=False,
                    points=points,
                    entry=points[0],
                    exit=points[-1],
                    errors=[
                        f"Ponto #{i + 1} é circular, mas faltam {faltam} ponto(s) C consecutivo(s) "
                        "para formar início, passagem e fim. Nenhum movimento foi enviado ao robô."
                    ],
                )

            p_mid = points[i + 1]
            p_end = points[i + 2]
            if p_mid.kind != PointKind.CIRCULAR or p_end.kind != PointKind.CIRCULAR:
                return TrajectoryPlan(
                    ok=False,
                    points=points,
                    entry=points[0],
                    exit=points[-1],
                    errors=[
                        f"Ponto #{i + 1} é circular. Os pontos #{i + 2} e #{i + 3} também precisam ser C "
                        "para formar um arco: início, passagem e fim. Nenhum movimento foi enviado ao robô."
                    ],
                )

            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.LINEAR,
                    target=p,
                    note="Entrada linear até o início do arco circular.",
                )
            )
            plan.segments.append(
                TrajectorySegment(
                    kind=SegmentKind.CIRCULAR,
                    start=p,
                    mid=p_mid,
                    end=p_end,
                    note="Arco circular sequencial de 3 pontos.",
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
