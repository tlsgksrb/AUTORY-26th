"""해석기 검증: 외팔보 처짐/비틀림을 이론식과 비교.  실행: python -m pytest tests  또는  python tests/test_fea.py"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from frame.config import GROUPS, MATERIAL, TUBES, Params  # noqa: E402
from frame.fea import Model  # noqa: E402
from frame.geometry import Frame, Member  # noqa: E402

L = 1000.0
TUBE = TUBES[GROUPS["SIS"].tube]


def _cantilever(direction):
    fr = Frame(params=Params())
    fr.nodes = {"A": np.zeros(3), "B": np.array(direction, float) * L}
    fr.members = [Member("A-B", "A", "B", "SIS")]
    return Model(fr)


def test_tip_deflection():
    P = 100.0
    for d in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [0.6, 0.0, 0.8]):
        m = _cantilever(d)
        load = np.array([0, 0, P]) if d != [0, 0, 1] else np.array([P, 0, 0])
        u, _ = m.solve({"B": load}, {"A": (0, 1, 2, 3, 4, 5)})
        # 하중의 부재 수직 성분만 굽힘을 만든다
        e = np.array(d, float) / np.linalg.norm(d)
        p_perp = np.linalg.norm(load - np.dot(load, e) * e)
        p_ax = np.dot(load, e)
        exp_perp = p_perp * L ** 3 / (3 * MATERIAL.E * TUBE.inertia)
        exp_ax = p_ax * L / (MATERIAL.E * TUBE.area)
        got = m.disp(u, "B")
        got_perp = np.linalg.norm(got - np.dot(got, e) * e)
        got_ax = np.dot(got, e)
        assert math.isclose(got_perp, exp_perp, rel_tol=1e-9), (d, got_perp, exp_perp)
        assert math.isclose(got_ax, exp_ax, rel_tol=1e-9, abs_tol=1e-12)


def test_torsion_and_stress():
    T = 50_000.0  # N·mm
    m = _cantilever([1, 0, 0])
    u, _ = m.solve({"B": [0, 0, 0, T, 0, 0]}, {"A": (0, 1, 2, 3, 4, 5)})
    phi = u[6 * m.idx["B"] + 3]
    assert math.isclose(phi, T * L / (MATERIAL.G * TUBE.polar), rel_tol=1e-9)
    r = m.element_results(u)[0]
    tau = T * (TUBE.od / 2) / TUBE.polar
    assert math.isclose(r.vm, math.sqrt(3) * tau, rel_tol=1e-9)


def test_frame_runs_and_is_symmetric():
    from frame.geometry import build
    from frame.loadcases import build_cases
    fr = build(Params())
    m = Model(fr)
    lc = next(c for c in build_cases(m) if c.key == "bending_1g")
    u, reac = m.solve(lc.loads, lc.supports)
    # 좌우 대칭 하중 → 좌우 바퀴 반력 같음
    rl = reac[6 * m.idx["WC_F_L"] + 2]
    rr = reac[6 * m.idx["WC_F_R"] + 2]
    assert math.isclose(rl, rr, rel_tol=0.02)
    # 반력 합 = 총 하중
    total = sum(v[2] for v in lc.loads.values())
    assert math.isclose(sum(reac[6 * m.idx[w] + 2] for w in
                            ("WC_F_L", "WC_F_R", "WC_R_L", "WC_R_R")), -total, rel_tol=1e-6)


if __name__ == "__main__":
    test_tip_deflection(); test_torsion_and_stress(); test_frame_runs_and_is_symmetric()
    print("all tests passed")
