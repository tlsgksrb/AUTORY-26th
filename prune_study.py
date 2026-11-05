"""부재 제거 검토: 체크를 모두 유지하면서 뺄 수 있는 부재를 탐욕적으로 찾는다.

좌우 대칭 쌍은 함께 뺀다. 규정상 필수(후프, 지지대, 측면충격 상·하부재, 벌크헤드,
벨트 바 등)와 기능상 필요한 부재(대시바, 방화벽 하단, 후방 범퍼 마운트면)는 후보에서 제외.
실행: python prune_study.py [비틀림 하한 N·m/deg=1800] [충돌 안전율 하한=1.05]
"""
from __future__ import annotations

import math
import sys

import numpy as np

from frame.config import GROUPS, MATERIAL, TUBES, Params
from frame.fea import Model
from frame.geometry import build, physical_tubes
from frame.loadcases import build_cases

PROTECT_GROUPS = {"MH", "FH", "SHB", "MHB", "FBH", "LAP"}
PROTECT = {
    "FH_up_L-FH_up_R", "MH_bot_L-MH_bot_R", "FH_bot_L-FH_bot_R",
    "RL_b_L-RL_b_R", "RU_b_L-RU_b_R", "RL_b_L-RU_b_L", "RL_b_R-RU_b_R",
    # 측면충격 상·하부재
    "FH_bot_L-LR_m_L", "LR_m_L-MH_bot_L", "FH_mid_L-SU_m_L", "SU_m_L-MH_mid_L",
    "FH_bot_R-LR_m_R", "LR_m_R-MH_bot_R", "FH_mid_R-SU_m_R", "SU_m_R-MH_mid_R",
    # 전방 벌크헤드 지지 상·하 레일 + 필수 대각
    *(f"{a}_{s}-{b}_{s}" for s in "LR" for a, b in (
        ("FB_bot", "FL_a"), ("FL_a", "FL_b"), ("FL_b", "FH_bot"),
        ("FB_top", "FU_a"), ("FU_a", "FU_b"), ("FU_b", "FH_mid"), ("FB_bot", "FU_a"))),
}


def mirror(name):
    sw = lambda n: n[:-2] + ("_R" if n.endswith("_L") else "_L") if n[-2:] in ("_L", "_R") else n
    a, b = name.split("-")
    return f"{sw(a)}-{sw(b)}"


def evaluate(fr):
    m = Model(fr)
    worst, K = {}, None
    for lc in build_cases(m):
        u, _ = m.solve(lc.loads, lc.supports)
        res = [r for r in m.element_results(u) if GROUPS[r.member.group].rule_class != "rigid"]
        sf = min(min(r.sf, 1 / r.buckling_util if r.buckling_util > 0 else 99) for r in res)
        dmax = max(np.linalg.norm(m.disp(u, n)) for n in fr.nodes if not n.startswith(("WC_", "LBJ_", "UBJ_")))
        if lc.key == "torsion":
            dz = m.disp(u, "WC_F_L")[2] - m.disp(u, "WC_F_R")[2]
            K = 1000 * fr.params.track_f / 1000 / math.degrees(math.atan(dz / fr.params.track_f))
        worst[lc.key] = (sf, dmax, lc.kind)
    mass = sum(TUBES[GROUPS[x.group].tube].area * fr.length(x) * MATERIAL.rho for x in fr.frame_members()) * 1.1
    return dict(K=K, worst=worst, mass=mass)


def feasible(ev, kmin, crash):
    if ev["K"] < kmin:
        return False
    for key, (sf, d, kind) in ev["worst"].items():
        if (kind == "operating" and sf < 1.5) or (kind == "crash" and sf < crash):
            return False
        if key in ("main_hoop", "front_hoop", "side_impact") and d > 25:
            return False
    return True


def main(params: Params, kmin=1800.0, crash=1.05):
    fr = build(params)
    cur = evaluate(fr)
    print(f"시작: 구간 {len(fr.frame_members())}개 / 튜브 {physical_tubes(fr)}개, {cur['mass']:.1f} kg, "
          f"{cur['K']:.0f} N·m/deg")
    while True:
        names = {x.name for x in fr.members}
        best, seen = None, set()
        for mm in fr.frame_members():
            if mm.group in PROTECT_GROUPS or mm.name in PROTECT or mm.name in seen:
                continue
            unit = {mm.name} | ({mirror(mm.name)} & names)
            seen |= unit
            trial = build(params)
            trial.members = [x for x in trial.members if x.name in names - unit]
            try:
                ev = evaluate(trial)
            except (RuntimeError, np.linalg.LinAlgError):
                continue
            if not feasible(ev, kmin, crash):
                continue
            score = (cur["K"] - ev["K"]) / max(cur["mass"] - ev["mass"], 1e-3)
            if best is None or score < best[0]:
                best = (score, unit, ev)
        if best is None:
            break
        fr.members = [x for x in fr.members if x.name not in best[1]]
        cur = best[2]
        print(f"  제거 {' + '.join(sorted(best[1])):40s} → {cur['K']:.0f} N·m/deg, {cur['mass']:.1f} kg")
    print(f"결과: 구간 {len(fr.frame_members())}개 / 튜브 {physical_tubes(fr)}개, {cur['mass']:.1f} kg, "
          f"{cur['K']:.0f} N·m/deg")
    print("  하중 조건별 최소 안전율:", {k: round(v[0], 2) for k, v in cur["worst"].items()})


if __name__ == "__main__":
    a = sys.argv[1:]
    main(Params(), float(a[0]) if a else 1800.0, float(a[1]) if len(a) > 1 else 1.05)
