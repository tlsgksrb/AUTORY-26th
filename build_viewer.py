"""output 결과로 3D 뷰어(viewer/index.html)를 만든다.  실행: python build_viewer.py"""
from __future__ import annotations

import json
from pathlib import Path

from frame.config import GROUPS, PACKAGING, TUBES, Params
from frame.fea import Model
from frame.geometry import build
from frame.loadcases import build_cases

ROOT = Path(__file__).parent


def main():
    p = Params()
    fr = build(p)
    model = Model(fr)
    summary = json.loads((ROOT / "output" / "results.json").read_text(encoding="utf-8"))

    names = list(fr.nodes)
    idx = {n: i for i, n in enumerate(names)}
    members = []
    for m in fr.members:
        g = GROUPS[m.group]
        members.append([idx[m.a], idx[m.b], m.group, round(fr.length(m))])
    cases = []
    for lc in build_cases(model):
        if lc.kind == "stiffness" and lc.key != "torsion":
            continue
        u, _ = model.solve(lc.loads, lc.supports)
        sf = {}
        for r in model.element_results(u):
            if GROUPS[r.member.group].rule_class == "rigid":
                continue
            b = 1 / r.buckling_util if r.buckling_util > 0 else 99
            sf[r.member.name] = round(min(r.sf, b, 99), 2)
        cases.append(dict(key=lc.key, name=lc.name, kind=lc.kind, note=lc.note,
                          sf=[sf.get(f"{fr.members[i].a}-{fr.members[i].b}") for i in range(len(fr.members))]))
    data = dict(
        nodes=[[round(float(c), 1) for c in fr.nodes[n]] for n in names],
        names=names,
        members=members,
        groups={k: dict(name=g.name, color=g.color, tube=TUBES[g.tube].label, od=TUBES[g.tube].od,
                        cls=g.rule_class) for k, g in GROUPS.items()},
        driver={k: [round(float(c), 1) for c in v] for k, v in fr.driver.items() if hasattr(v, "__len__")},
        radii=dict(hip=fr.driver["r_hip"], shoulder=fr.driver["r_shoulder"], head=fr.driver["r_head"]),
        wheels=dict(wb=p.wheelbase, tf=p.track_f, tr=p.track_r, d=p.tire_d, w=p.tire_w),
        sw=[p.sw_x, p.sw_z, p.sw_r],
        packaging={k: list(v) for k, v in PACKAGING.items()},
        cases=cases,
        summary={k: summary[k] for k in ("frame_mass_kg", "torsional_stiffness_Nm_per_deg", "vehicle_mass_kg",
                                         "front_weight_pct", "frame_envelope_mm", "cg_mm", "bom")},
        checks=[dict(item=c["item"], value=c["value"], unit=c.get("unit") or "", criterion=c["criterion"],
                     ok=c["ok"]) for c in summary["checks"]],
        load_cases=[{k: r[k] for k in ("key", "name", "max_disp_mm", "min_sf", "min_sf_group")}
                    for r in summary["load_cases"]],
    )
    tpl = (ROOT / "viewer" / "template.html").read_text(encoding="utf-8")
    out = tpl.replace("/*__DATA__*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    (ROOT / "viewer" / "index.html").write_text(out, encoding="utf-8")
    print("viewer/index.html", len(out) // 1024, "KB")


if __name__ == "__main__":
    main()
