"""프레임 생성 → 규정 체크 → 하중조건별 해석 → 결과/그림/CAD 출력.

사용법:  python run_frame.py            (결과는 output/ 에 저장)
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from frame import checks, export, plots
from frame.config import GROUPS, MASSES, MATERIAL, RULES, TUBES, Params
from frame.fea import Model
from frame.geometry import build
from frame.loadcases import build_cases, sprung_masses, vehicle_mass

OUT = Path(__file__).parent / "output"


def _jsonable(o):
    return o.item() if hasattr(o, "item") else str(o)


def frame_mass(model):
    m = model.node_masses(frame_extra_ratio=0.0)
    return sum(m.values())


def main(params: Params | None = None, out: Path = OUT, figures=True):
    out.mkdir(exist_ok=True)
    p = params or Params()
    fr = build(p)
    model = Model(fr)

    # --- 질량 ------------------------------------------------------------
    m_tubes = frame_mass(model)
    m_frame = m_tubes * (1 + MASSES["frame_extra_ratio"])
    m_vehicle = vehicle_mass(model)
    allm = model.node_masses(sprung_masses(), frame_extra_ratio=MASSES["frame_extra_ratio"])
    for w in ("WC_F_L", "WC_F_R", "WC_R_L", "WC_R_R"):
        allm[w] += MASSES["unsprung"] / 4
    tot = sum(allm.values())
    cg = sum(allm[n] * fr.nodes[n] for n in allm) / tot
    front_pct = 100 * (1 - cg[0] / p.wheelbase)

    # --- 해석 ------------------------------------------------------------
    results = {}
    case_rows = []
    torsion = None
    for lc in build_cases(model):
        u, reac = model.solve(lc.loads, lc.supports)
        res = model.element_results(u)
        frame_res = [r for r in res if GROUPS[r.member.group].rule_class != "rigid"]
        worst_y = min(frame_res, key=lambda r: r.sf)
        worst_b = max(frame_res, key=lambda r: r.buckling_util)
        dmax_node = max((n for n in fr.nodes if not n.startswith(("WC_", "LBJ_", "UBJ_"))),
                        key=lambda n: np.linalg.norm(model.disp(u, n)))
        dmax = float(np.linalg.norm(model.disp(u, dmax_node)))
        row = dict(key=lc.key, name=lc.name, kind=lc.kind, note=lc.note,
                   max_disp_mm=round(dmax, 2), max_disp_node=dmax_node,
                   min_sf=round(worst_y.sf, 2), min_sf_member=worst_y.member.name,
                   min_sf_group=GROUPS[worst_y.member.group].name,
                   max_vm_mpa=round(worst_y.vm, 1),
                   max_buckling_util=round(worst_b.buckling_util, 3),
                   max_buckling_member=worst_b.member.name)
        if lc.key == "torsion":
            dzl = model.disp(u, "WC_F_L")[2]; dzr = model.disp(u, "WC_F_R")[2]
            T = lc.loads["WC_F_L"][2] * p.track_f / 1000  # N·m
            theta = math.degrees(math.atan((dzl - dzr) / p.track_f))
            torsion = T / theta
            row["torsional_stiffness_Nm_per_deg"] = round(torsion, 0)
            # 프레임 구간별 비틀림각 분포(바닥 좌우 노드 Z 변위 차)
            dist = []
            for l, r_ in (("FB_bot_L", "FB_bot_R"), ("FL_a_L", "FL_a_R"), ("FL_b_L", "FL_b_R"),
                          ("FH_bot_L", "FH_bot_R"), ("LR_m_L", "LR_m_R"), ("MH_bot_L", "MH_bot_R"),
                          ("RL_a_L", "RL_a_R"), ("RL_b_L", "RL_b_R")):
                w = fr.nodes[l][1] - fr.nodes[r_][1]
                ang = math.degrees(math.atan((model.disp(u, l)[2] - model.disp(u, r_)[2]) / w))
                dist.append(dict(station=l[:-2], x=round(float(fr.nodes[l][0])), twist_deg=round(ang, 4)))
            row["twist_distribution"] = dist
            if figures:
                plots.deformed(fr, model, u, str(out / "torsion_deformed.png"), "비틀림 하중 변형")
        if lc.kind == "crash" and lc.key in ("main_hoop", "side_impact") and figures:
            plots.deformed(fr, model, u, str(out / f"{lc.key}_deformed.png"), lc.name)
        results[lc.key] = (lc.name, res)
        case_rows.append(row)

    # --- 체크 ------------------------------------------------------------
    chk = checks.run(fr)
    sf_op, sf_cr = RULES["sf_operating"].value, RULES["sf_crash"].value
    for row in case_rows:
        if row["kind"] == "operating":
            ok = row["min_sf"] >= sf_op and row["max_buckling_util"] <= 1 / sf_op
            chk.append(dict(item=f"해석 · {row['name']} 안전율", value=row["min_sf"],
                            criterion=f"≥ {sf_op:g} (좌굴 이용률 ≤ {1 / sf_op:.2f})", ok=ok,
                            rule="sf_operating", unit="-"))
        elif row["kind"] == "crash":
            ok = row["min_sf"] >= sf_cr and row["max_buckling_util"] <= 1 / sf_cr
            chk.append(dict(item=f"해석 · {row['name']} 안전율", value=row["min_sf"],
                            criterion=f"≥ {sf_cr:g} (좌굴 이용률 ≤ 1)", ok=ok, rule="sf_crash", unit="-"))
            if row["key"] in ("main_hoop", "front_hoop", "side_impact"):
                lim = RULES["hoop_deflection_max"].value
                chk.append(dict(item=f"해석 · {row['name']} 최대 변위", value=row["max_disp_mm"],
                                criterion=f"≤ {lim:g}", ok=row["max_disp_mm"] <= lim,
                                rule="hoop_deflection_max", unit="mm"))
    tgt = RULES["torsion_target"].value
    chk.append(dict(item="해석 · 비틀림 강성", value=round(torsion), criterion=f"≥ {tgt:g}",
                    ok=torsion >= tgt, rule="torsion_target", unit="N·m/deg"))
    mt = RULES["frame_mass_target"].value
    chk.append(dict(item="프레임 질량(용접·탭 +10 %)", value=round(m_frame, 1), criterion=f"≤ {mt:g}",
                    ok=m_frame <= mt, rule="frame_mass_target", unit="kg"))

    # --- 출력 ------------------------------------------------------------
    export.write_nodes(fr, out / "nodes.csv")
    rows = export.write_cutlist(fr, out / "cut_list.csv")
    bom = export.bom(rows)
    export.write_dxf(fr, out / "frame_centerlines.dxf")
    pts = np.array([fr.nodes[n] for m in fr.frame_members() for n in (m.a, m.b)])
    dims = dict(length=float(np.ptp(pts[:, 0])), width=float(np.ptp(pts[:, 1])),
                height=float(pts[:, 2].max()))
    summary = dict(
        material=MATERIAL.name,
        tubes={g.name: TUBES[g.tube].label for g in GROUPS.values() if g.rule_class != "rigid"},
        frame_mass_tubes_kg=round(m_tubes, 2), frame_mass_kg=round(m_frame, 2),
        vehicle_mass_kg=round(m_vehicle, 1),
        cg_mm=[round(float(c), 1) for c in cg], front_weight_pct=round(front_pct, 1),
        torsional_stiffness_Nm_per_deg=round(torsion),
        stiffness_per_mass=round(torsion / m_frame, 1),
        frame_envelope_mm={k: round(v) for k, v in dims.items()},
        members=len(rows), nodes=len([n for n in fr.nodes if not n.startswith(("WC_", "LBJ_", "UBJ_"))]),
        load_cases=case_rows, checks=chk, bom=bom,
        driver={k: [round(float(c), 1) for c in v] for k, v in fr.driver.items() if hasattr(v, "__len__")},
    )
    with open(out / "results.json", "w", encoding="utf-8") as fp:
        json.dump(summary, fp, ensure_ascii=False, indent=1, default=_jsonable)
    export.write_viewer_json(fr, out / "frame.json",
                             extra=dict(summary={k: summary[k] for k in (
                                 "frame_mass_kg", "torsional_stiffness_Nm_per_deg", "vehicle_mass_kg",
                                 "front_weight_pct", "frame_envelope_mm")}))
    write_summary_md(summary, out / "summary.md")
    if figures:
        plots.overview(fr, str(out / "frame_overview.png"),
                       f"AUTORY 2027 e-모빌리티 프레임 v0.1 — {m_frame:.1f} kg, {torsion:.0f} N·m/deg")
        plots.safety_grid(fr, {k: v for k, v in results.items() if k not in ("torsion", "bending_1g")},
                          str(out / "safety_factors.png"))
    return summary


def write_summary_md(s, path):
    L = ["# 프레임 해석 결과 요약 (자동 생성)", "",
         "`python run_frame.py` 실행 결과. 수치는 `frame/config.py` 입력값 기준.", "",
         f"- 재료: {s['material']}",
         f"- 프레임 질량: {s['frame_mass_kg']} kg (튜브만 {s['frame_mass_tubes_kg']} kg + 용접·탭 10 %)",
         f"- 비틀림 강성: **{s['torsional_stiffness_Nm_per_deg']} N·m/deg** "
         f"({s['stiffness_per_mass']} N·m/deg/kg)",
         f"- 차량 질량(드라이버 포함, 가정): {s['vehicle_mass_kg']} kg, 앞축 하중 {s['front_weight_pct']} %",
         f"- 무게중심 (X, Y, Z): {s['cg_mm']} mm",
         f"- 프레임 외곽: 길이 {s['frame_envelope_mm']['length']} × 폭 {s['frame_envelope_mm']['width']} × "
         f"높이 {s['frame_envelope_mm']['height']} mm", "",
         "## 하중 조건별 결과", "",
         "| 조건 | 최대 변위 (mm) | 최소 안전율 | 취약 부재 | 최대 좌굴 이용률 |",
         "|---|---:|---:|---|---:|"]
    for r in s["load_cases"]:
        L.append(f"| {r['name']} | {r['max_disp_mm']} | {r['min_sf']} | {r['min_sf_member']} "
                 f"({r['min_sf_group']}) | {r['max_buckling_util']} |")
    L += ["", "## 체크리스트", "", "| 항목 | 값 | 기준 | 판정 |", "|---|---:|---|:---:|"]
    for c in s["checks"]:
        L.append(f"| {c['item']} | {c['value']} {c.get('unit') or ''} | {c['criterion']} | "
                 f"{'OK' if c['ok'] else '**NG**'} |")
    L += ["", "## 강관 소요량", "", "| 규격 | 개수 | 총 길이 (m) | 질량 (kg) | 6 m 정척(손실 15 %) |",
          "|---|---:|---:|---:|---:|"]
    for b in s["bom"]:
        L.append(f"| {b['tube']} | {b['pieces']} | {b['length_m']} | {b['mass_kg']} | {b['stock_bars_6m']} |")
    with open(path, "w", encoding="utf-8") as fp:
        fp.write("\n".join(L) + "\n")


if __name__ == "__main__":
    s = main()
    print(f"프레임 {s['frame_mass_kg']} kg, 비틀림 {s['torsional_stiffness_Nm_per_deg']} N·m/deg, "
          f"차량 {s['vehicle_mass_kg']} kg, 앞축 {s['front_weight_pct']} %")
    for r in s["load_cases"]:
        print(f"  {r['name']:<28} 변위 {r['max_disp_mm']:>7} mm  SF {r['min_sf']:>6}  "
              f"[{r['min_sf_member']}]  좌굴 {r['max_buckling_util']}")
    bad = [c for c in s["checks"] if not c["ok"]]
    print(f"체크 {len(s['checks']) - len(bad)}/{len(s['checks'])} 통과")
    for c in bad:
        print(f"  NG: {c['item']} = {c['value']} (기준 {c['criterion']})")
