"""형상·규정 체크. 각 항목은 (항목, 값, 기준, 판정, 규정키) 형태로 반환."""
from __future__ import annotations

import math

import numpy as np

from .config import GROUPS, MATERIAL, RULES, TUBES
from .geometry import Frame


def _xz(v):
    return np.array([v[0], v[2]])


def _signed_dist_below(p, a, b):
    """측면도에서 점 p 가 선분 a→b 아래로 얼마나 떨어져 있는지(+면 아래)."""
    d = b - a
    n = np.array([-d[1], d[0]]) / np.linalg.norm(d)   # 위쪽 법선(a→b 가 +x 방향일 때)
    if n[1] < 0:
        n = -n
    return -float(np.dot(p - a, n))


def _seg_point_dist(a, b, q):
    ab = b - a
    t = np.clip(np.dot(q - a, ab) / np.dot(ab, ab), 0, 1)
    return float(np.linalg.norm(a + t * ab - q))


def clearance(frame: Frame, center, radius):
    """구(center, radius)와 프레임 튜브 표면 사이 최소 거리와 해당 부재."""
    best = (math.inf, None)
    for m in frame.frame_members():
        od = TUBES[GROUPS[m.group].tube].od
        d = _seg_point_dist(frame.nodes[m.a], frame.nodes[m.b], center) - radius - od / 2
        if d < best[0]:
            best = (d, m.name)
    return best


def run(frame: Frame) -> list[dict]:
    p = frame.params
    N = frame.nodes
    d = frame.driver
    R = RULES
    out: list[dict] = []

    def item(name, value, crit, ok, rule=None, unit=""):
        out.append(dict(item=name, value=value, criterion=crit, ok=bool(ok), rule=rule, unit=unit))

    # 차량 제원
    item("휠베이스", p.wheelbase, f"≥ {R['wheelbase_min'].value:g}",
         p.wheelbase >= R["wheelbase_min"].value, "wheelbase_min", "mm")
    item("타이어 폭(가정 타이어)", p.tire_w, f"≥ {R['tire_width_min'].value:g}",
         p.tire_w >= R["tire_width_min"].value, "tire_width_min", "mm")

    # 튜브 규격
    for gk, g in GROUPS.items():
        if g.rule_class == "rigid":
            continue
        t = TUBES[g.tube]
        if g.rule_class == "hoop":
            ok = t.od >= R["hoop_od_min"].value and t.t >= R["hoop_t_min"].value
            crit = f"≥ Ø{R['hoop_od_min'].value:g}×{R['hoop_t_min'].value:g}"
            rk = "hoop_t_min"
        elif g.rule_class == "primary":
            ok = t.od >= R["hoop_od_min"].value and t.t >= R["primary_t_min"].value
            crit = f"≥ Ø{R['hoop_od_min'].value:g}×{R['primary_t_min'].value:g}"
            rk = "primary_t_min"
        else:
            ok = t.od >= R["secondary_od_min"].value and t.t >= R["secondary_t_min"].value
            crit = f"≥ Ø{R['secondary_od_min'].value:g}×{R['secondary_t_min'].value:g}"
            rk = "secondary_t_min"
        item(f"튜브 규격 · {g.name}", t.label, crit, ok, rk)

    item("재료 항복강도", MATERIAL.yield_, f"≥ {R['yield_min'].value:g}",
         MATERIAL.yield_ >= R["yield_min"].value, "yield_min", "MPa")
    item("재료 인장강도", MATERIAL.tensile, f"≥ {R['tensile_min'].value:g}",
         MATERIAL.tensile >= R["tensile_min"].value, "tensile_min", "MPa")

    # 헬멧 여유: 전방 후프 상단 ~ 메인 후프 상단 선
    head = d["head"]
    fh_top, mh_top = _xz(N["FH_top_C"]), _xz(N["MH_top_C"])
    clear = _signed_dist_below(head, fh_top, mh_top) - d["r_head"]
    item("헬멧 ~ 롤후프 연결선 여유", round(clear, 1), f"≥ {R['helmet_clearance'].value:g}",
         clear >= R["helmet_clearance"].value, "helmet_clearance", "mm")
    # 헬멧이 메인 후프 앞에 있는지
    gap = (p.x_mh - 12.7) - (head[0] + d["r_head"])
    item("헬멧 뒤 ~ 메인 후프 앞면 (헤드레스트 공간)", round(gap, 1), "≥ 40 (패드+간격, 자체)",
         gap >= 40, None, "mm")
    # 헬멧 상단이 메인 후프보다 낮은지
    item("메인 후프 상단 - 헬멧 상단", round(N["MH_top_C"][2] - (head[1] + d["r_head"]), 1),
         "> 0", N["MH_top_C"][2] > head[1] + d["r_head"], None, "mm")

    # 메인 후프 지지대 각도/부착 위치
    br, rb = _xz(N[frame.brace_node]), _xz(N["RU_b_L"])
    v = rb - br
    ang = math.degrees(math.atan2(abs(v[0]), abs(v[1])))   # 수직(메인 후프)과 이루는 각
    item("메인 후프-지지대 각도(측면도)", round(ang, 1), f"≥ {R['mhb_angle_min'].value:g}",
         ang >= R["mhb_angle_min"].value, "mhb_angle_min", "deg")
    att = N["MH_top_C"][2] + TUBES["T1"].od / 2 - N[frame.brace_node][2]
    item("지지대 부착점 ~ 후프 최상단", round(att, 1), f"≤ {R['mhb_attach_max'].value:g}",
         att <= R["mhb_attach_max"].value, "mhb_attach_max", "mm")

    # 전방 후프 vs 스티어링 휠
    sw_top = p.sw_z + p.sw_r
    item("전방 후프 상단 - 스티어링 휠 상단", round(N["FH_top_C"][2] - sw_top, 1), "≥ 0",
         N["FH_top_C"][2] >= sw_top, None, "mm")
    fwd = p.sw_x - p.x_fh
    item("전방 후프가 휠보다 앞선 거리", round(fwd, 1), f"≤ {R['fh_to_sw_max'].value:g}",
         0 <= fwd <= R["fh_to_sw_max"].value, "fh_to_sw_max", "mm")
    # 무릎 ~ 휠 하단
    knee = d["knee"]
    item("무릎 상단(+60) ~ 전방 후프 상단", round(N["FH_top_C"][2] - (knee[1] + 60), 1), "> 0",
         N["FH_top_C"][2] > knee[1] + 60, None, "mm")

    # 드라이버 신체 ~ 프레임 튜브 간섭(3D)
    from .config import TEMPLATE
    kr, ky = TEMPLATE["knee_r"], TEMPLATE["knee_y"]
    worst = min((clearance(frame, np.array([knee[0], s * ky, knee[1]]), kr) for s in (1, -1)),
                key=lambda r: r[0])
    item(f"무릎 ~ 프레임 튜브 최소 간격 [{worst[1]}]", round(worst[0], 1), "≥ 25 (자체)",
         worst[0] >= 25, None, "mm")
    hc = clearance(frame, np.array([head[0], 0.0, head[1]]), d["r_head"])
    item(f"헬멧 ~ 프레임 튜브 최소 간격 [{hc[1]}]", round(hc[0], 1), "≥ 25 (자체)",
         hc[0] >= 25, None, "mm")

    # 어깨벨트
    sh = d["shoulder"]
    dz = p.z_shb - sh[1]
    dx = p.x_mh - sh[0]
    sang = math.degrees(math.atan2(dz, dx))
    item("어깨벨트 각도(수평 기준, +위)", round(sang, 1),
         f"-{R['shoulder_angle_down'].value:g} ~ +{R['shoulder_angle_up'].value:g}",
         -R["shoulder_angle_down"].value <= sang <= R["shoulder_angle_up"].value, "shoulder_angle_up", "deg")
    spacing = 2 * p.shoulder_mount_half
    item("어깨벨트 마운트 간격", spacing,
         f"{R['shoulder_spacing_min'].value:g} ~ {R['shoulder_spacing_max'].value:g}",
         R["shoulder_spacing_min"].value <= spacing <= R["shoulder_spacing_max"].value,
         "shoulder_spacing_min", "mm")
    # 랩벨트
    hip = d["hip"]
    lap = _xz(N["LR_m_L"])
    lang = math.degrees(math.atan2(hip[1] - lap[1], abs(lap[0] - hip[0])))
    item("랩벨트 각도(측면도)", round(lang, 1),
         f"{R['lap_angle_min'].value:g} ~ {R['lap_angle_max'].value:g}",
         R["lap_angle_min"].value <= lang <= R["lap_angle_max"].value and lap[0] > hip[0],
         "lap_angle_min", "deg")

    # 최저지상고 (가장 낮은 프레임 튜브 하단)
    zmin = min(N[m.a][2] for m in frame.frame_members()) - TUBES["T2"].od / 2
    item("프레임 최저지상고", round(zmin, 1), f"≥ {R['ground_clearance_min'].value:g}",
         zmin >= R["ground_clearance_min"].value, "ground_clearance_min", "mm")

    # 6점식/범퍼 마운트 존재
    have = all(n in N for n in ("SH_L", "SH_R", "LR_m_L", "LR_m_R", "LRX_C"))
    item("6점식 벨트 마운트 노드(어깨2·랩2·안티서브)", "있음" if have else "없음", "필수", have, "harness_points")
    have = all(n in N for n in ("FB_bot_L", "FB_top_L", "RL_b_L", "RU_b_L"))
    item("전·후면 범퍼(충격완화장치) 마운트면", "전방/후방 벌크헤드" if have else "없음", "필수", have,
         "bumper_absorber")
    return out
