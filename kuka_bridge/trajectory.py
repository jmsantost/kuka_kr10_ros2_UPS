"""Utilidades de trayectoria sin dependencias de ROS (testeables con pytest)."""

import math


def simplify_path(points, tol):
    """Ramer-Douglas-Peucker en espacio articular (grados).

    Conserva primer y ultimo punto. Un punto intermedio se elimina si esta a
    menos de tol (max por eje) de donde estaria sobre la recta entre los puntos
    conservados, recorrida en proporcion a la distancia acumulada del camino
    original. Asi una ida y vuelta sobre la misma recta no se fusiona.
    """
    cum = [0.0]
    for p, q in zip(points, points[1:]):
        cum.append(cum[-1] + math.dist(p, q))

    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        i0, i1 = stack.pop()
        if i1 - i0 < 2:
            continue
        a, b = points[i0], points[i1]
        length = cum[i1] - cum[i0]
        worst, worst_i = -1.0, None
        for i in range(i0 + 1, i1):
            t = (cum[i] - cum[i0]) / length if length > 0 else 0.0
            dev = max(abs(pi - (ai + t * (bi - ai))) for pi, ai, bi in zip(points[i], a, b))
            if dev > worst:
                worst, worst_i = dev, i
        if worst > tol:
            keep.add(worst_i)
            stack += [(i0, worst_i), (worst_i, i1)]
    return [points[i] for i in sorted(keep)]
