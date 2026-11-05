"""CSV(노드·절단목록·BOM), DXF(3D 중심선), JSON(뷰어용) 출력."""
from __future__ import annotations

import csv
import json
import math
from collections import defaultdict

from .config import GROUPS, MATERIAL, TUBES
from .geometry import Frame

STOCK_LEN = 6000.0  # 강관 정척 길이(mm) 가정


def write_nodes(frame: Frame, path):
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.writer(fp)
        w.writerow(["node", "x_mm", "y_mm", "z_mm"])
        for n, v in frame.nodes.items():
            w.writerow([n, *(round(float(c), 1) for c in v)])


def write_cutlist(frame: Frame, path, extra_ratio=0.0):
    rows = []
    for i, m in enumerate(frame.frame_members(), 1):
        g = GROUPS[m.group]
        t = TUBES[g.tube]
        L = frame.length(m)
        rows.append(dict(no=i, member=m.name, group=g.name, tube=t.label, length_mm=round(L, 1),
                         mass_kg=round(t.area * L * MATERIAL.rho, 3),
                         node_a=m.a, node_b=m.b,
                         a_xyz=" ".join(f"{c:.1f}" for c in frame.nodes[m.a]),
                         b_xyz=" ".join(f"{c:.1f}" for c in frame.nodes[m.b])))
    with open(path, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows


def bom(rows):
    agg = defaultdict(lambda: dict(pieces=0, length_mm=0.0, mass_kg=0.0))
    for r in rows:
        a = agg[r["tube"]]
        a["pieces"] += 1
        a["length_mm"] += r["length_mm"]
        a["mass_kg"] += r["mass_kg"]
    out = []
    for tube, a in agg.items():
        # 절단 손실·노칭 여유 15 %
        bars = math.ceil(a["length_mm"] * 1.15 / STOCK_LEN)
        out.append(dict(tube=tube, pieces=a["pieces"], length_m=round(a["length_mm"] / 1000, 2),
                        mass_kg=round(a["mass_kg"], 2), stock_bars_6m=bars))
    return sorted(out, key=lambda r: r["tube"], reverse=True)


def write_dxf(frame: Frame, path):
    """AutoCAD R12 ASCII DXF — 3D LINE 엔티티, 레이어 = 부재 그룹."""
    lines = ["0", "SECTION", "2", "ENTITIES"]
    for m in frame.frame_members():
        a, b = frame.nodes[m.a], frame.nodes[m.b]
        lines += ["0", "LINE", "8", m.group,
                  "10", f"{a[0]:.3f}", "20", f"{a[1]:.3f}", "30", f"{a[2]:.3f}",
                  "11", f"{b[0]:.3f}", "21", f"{b[1]:.3f}", "31", f"{b[2]:.3f}"]
    lines += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w", encoding="ascii") as fp:
        fp.write("\n".join(lines) + "\n")


def write_viewer_json(frame: Frame, path, extra: dict | None = None):
    data = dict(
        units="mm", axes="X 뒤(+), Y 왼쪽(+), Z 위(+)",
        nodes={n: [round(float(c), 1) for c in v] for n, v in frame.nodes.items()},
        members=[dict(a=m.a, b=m.b, g=m.group, od=TUBES[GROUPS[m.group].tube].od)
                 for m in frame.members],
        groups={k: dict(name=g.name, color=g.color, tube=TUBES[g.tube].label, cls=g.rule_class)
                for k, g in GROUPS.items()},
        driver={k: ([round(float(c), 1) for c in v] if hasattr(v, "__len__") else v)
                for k, v in frame.driver.items()},
        params=vars(frame.params),
    )
    if extra:
        data.update(extra)
    with open(path, "w", encoding="utf-8") as fp:
        json.dump(data, fp, ensure_ascii=False, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
