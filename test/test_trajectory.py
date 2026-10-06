import math

from kuka_bridge.trajectory import simplify_path

HOME = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]


def axes(a1=0.0, a2=-90.0, a3=90.0, a4=0.0, a5=0.0, a6=0.0):
    return [a1, a2, a3, a4, a5, a6]


def test_keeps_first_and_last():
    pts = [HOME, axes(a1=10)]
    assert simplify_path(pts, 0.5) == pts


def test_straight_line_collapses_to_endpoints():
    pts = [HOME] + [axes(a1=0.5 * k) for k in range(1, 21)]
    assert simplify_path(pts, 0.5) == [HOME, axes(a1=10)]


def test_irregular_spacing_on_a_line_collapses():
    pts = [HOME] + [axes(a1=0.1 * k ** 1.5) for k in range(1, 20)]
    out = simplify_path(pts, 0.5)
    assert len(out) == 2 and out[-1] == pts[-1]


def test_l_shape_keeps_the_corner():
    pts = ([HOME] + [axes(a1=0.5 * k) for k in range(1, 17)]
           + [axes(a1=8, a2=-90 + 0.5 * k) for k in range(1, 17)])
    assert simplify_path(pts, 0.5) == [HOME, axes(a1=8), axes(a1=8, a2=-82)]


def test_zigzag_on_same_line_is_not_merged():
    # Ida y vuelta sobre la misma recta: con proyeccion simple quedaba en 2 puntos
    pts, a = [HOME], 0.0
    corners = []
    for c in range(8):
        target = 4.0 if c % 2 == 0 else -4.0
        pts += [axes(a1=a + (target - a) * k / 8) for k in range(1, 9)]
        corners.append(target)
        a = target
    out = simplify_path(pts, 0.5)
    assert [p[0] for p in out[1:]] == corners


def test_arc_keeps_more_points_with_lower_tolerance():
    arc = [HOME] + [axes(a1=10 * math.sin(t / 20 * math.pi / 2),
                         a2=-90 + 10 * (1 - math.cos(t / 20 * math.pi / 2)))
                    for t in range(1, 21)]
    coarse, fine = simplify_path(arc, 0.5), simplify_path(arc, 0.1)
    assert 2 < len(coarse) < len(fine) < len(arc)


def test_every_removed_point_is_within_tolerance():
    arc = [HOME] + [axes(a1=30 * math.sin(t / 40 * math.pi),
                         a5=20 * (1 - math.cos(t / 40 * math.pi))) for t in range(1, 41)]
    tol = 0.5
    kept = simplify_path(arc, tol)
    # Cada punto original debe quedar a < tol de algun segmento conservado
    for p in arc:
        best = min(
            max(abs(pi - (ai + t * (bi - ai))) for pi, ai, bi in zip(p, a, b))
            for a, b in zip(kept, kept[1:])
            for t in [i / 200 for i in range(201)])
        assert best <= tol + 0.05


def test_stationary_points():
    assert simplify_path([HOME, HOME, HOME], 0.5) == [HOME, HOME]
