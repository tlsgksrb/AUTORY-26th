"""그림 출력 (matplotlib)."""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import koreanize_matplotlib  # noqa: F401  (한글 폰트)
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

from .config import GROUPS, PACKAGING, TUBES
from .geometry import Frame

VIEWS = {"side": (0, 2), "top": (0, 1), "front": (1, 2)}


def _segments(frame: Frame, include_rigid=False, disp=None, scale=1.0, model=None):
    segs = []
    for m in frame.members:
        g = GROUPS[m.group]
        if g.rule_class == "rigid" and not include_rigid:
            continue
        a, b = frame.nodes[m.a].copy(), frame.nodes[m.b].copy()
        if disp is not None:
            a = a + scale * model.disp(disp, m.a)
            b = b + scale * model.disp(disp, m.b)
        segs.append((m, a, b, g))
    return segs


def _draw_2d(ax, frame, view, include_rigid=True, lw_scale=1.0):
    i, j = VIEWS[view]
    for m, a, b, g in _segments(frame, include_rigid):
        if g.rule_class == "rigid":
            ax.plot([a[i], b[i]], [a[j], b[j]], color=g.color, lw=0.8, ls="--", zorder=1)
        else:
            lw = (2.6 if g.rule_class == "hoop" else 1.9 if g.rule_class == "primary" else 1.2) * lw_scale
            ax.plot([a[i], b[i]], [a[j], b[j]], color=g.color, lw=lw, solid_capstyle="round", zorder=3)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)


def _wheels_2d(ax, frame, view):
    p = frame.params
    r = p.tire_d / 2
    if view == "side":
        for x in (0, p.wheelbase):
            ax.add_patch(Circle((x, r), r, fill=False, ec="#444", lw=1, ls=":"))
    elif view == "top":
        for x, tr in ((0, p.track_f), (p.wheelbase, p.track_r)):
            for s in (1, -1):
                ax.add_patch(plt.Rectangle((x - r, s * tr / 2 - p.tire_w / 2), 2 * r, p.tire_w,
                                           fill=False, ec="#444", lw=1, ls=":"))
    elif view == "front":
        for tr in (p.track_f,):
            for s in (1, -1):
                ax.add_patch(plt.Rectangle((s * tr / 2 - p.tire_w / 2, 0), p.tire_w, p.tire_d,
                                           fill=False, ec="#444", lw=1, ls=":"))
    ax.axhline(0, color="#999", lw=0.8) if view != "top" else None


def _driver_2d(ax, frame):
    d = frame.driver
    p = frame.params
    for key, rk in (("hip", "r_hip"), ("shoulder", "r_shoulder"), ("head", "r_head")):
        ax.add_patch(Circle(d[key], d[rk], fill=False, ec="#1a9850", lw=1.2))
    pts = np.array([d["hip"], d["shoulder"], d["head"]])
    ax.plot(pts[:, 0], pts[:, 1], color="#1a9850", lw=1)
    leg = np.array([d["hip"], d["knee"], d["heel"]])
    ax.plot(leg[:, 0], leg[:, 1], color="#1a9850", lw=1, ls="--")
    fh, mh = frame.nodes["FH_top_C"], frame.nodes["MH_top_C"]
    ax.plot([fh[0], mh[0]], [fh[2], mh[2]], color="#d73027", lw=0.8, ls="-.")
    for name, (x0, x1, z0, z1, _) in PACKAGING.items():
        ax.add_patch(plt.Rectangle((x0, z0), x1 - x0, z1 - z0, fill=False, ec="#6a51a3", lw=1, ls="--"))
        ax.text((x0 + x1) / 2, z1 + 15, name, ha="center", fontsize=8, color="#6a51a3")
    ax.add_patch(Circle((p.sw_x, p.sw_z), 12, color="#555"))
    ax.plot([p.sw_x, p.sw_x], [p.sw_z - p.sw_r, p.sw_z + p.sw_r], color="#555", lw=2)


def _legend(fig):
    handles = [Line2D([0], [0], color=g.color, lw=2.5,
                      label=f"{g.name} {TUBES[g.tube].label}" if g.rule_class != "rigid" else g.name)
               for g in GROUPS.values()]
    handles.append(Line2D([0], [0], color="#1a9850", lw=1.2, label="95th 드라이버 템플릿"))
    handles.append(Line2D([0], [0], color="#d73027", lw=0.8, ls="-.", label="헬멧 여유 기준선"))
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=8, frameon=False)


def _set_iso(ax, frame):
    pts = np.array(list(frame.nodes.values()))
    mid = (pts.max(0) + pts.min(0)) / 2
    span = (pts.max(0) - pts.min(0)).max() / 2
    for setlim, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), mid):
        setlim(c - span, c + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=22, azim=-128)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")


def overview(frame: Frame, path: str, title: str):
    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.18, wspace=0.08,
                          left=0.04, right=0.98, top=0.93, bottom=0.12)
    ax = fig.add_subplot(gs[0, 0])
    _draw_2d(ax, frame, "side"); _wheels_2d(ax, frame, "side"); _driver_2d(ax, frame)
    ax.set_title("측면도 (X-Z)"); ax.invert_xaxis()
    ax = fig.add_subplot(gs[1, 0])
    _draw_2d(ax, frame, "top"); _wheels_2d(ax, frame, "top")
    ax.set_title("평면도 (X-Y)"); ax.invert_xaxis()
    ax = fig.add_subplot(gs[1, 1])
    _draw_2d(ax, frame, "front"); _wheels_2d(ax, frame, "front")
    ax.set_title("정면도 (Y-Z, 앞에서 본 모습)"); ax.invert_xaxis()
    ax = fig.add_subplot(gs[0, 1], projection="3d")
    for m, a, b, g in _segments(frame, include_rigid=True):
        if g.rule_class == "rigid":
            ax.plot(*zip(a, b), color=g.color, lw=0.6, ls="--")
        else:
            ax.plot(*zip(a, b), color=g.color, lw=2.4 if g.rule_class != "secondary" else 1.4)
    _set_iso(ax, frame); ax.set_title("등각도")
    fig.suptitle(title, fontsize=15)
    _legend(fig)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def deformed(frame, model, u, path, title, scale=None):
    disp = np.array([model.disp(u, n) for n in frame.nodes])
    umax = np.abs(disp).max()
    span = np.ptp(np.array(list(frame.nodes.values())), axis=0).max()
    scale = scale or (0.08 * span / umax if umax > 0 else 1)
    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(1, 1, 1, projection="3d")
    for m, a, b, g in _segments(frame):
        ax.plot(*zip(a, b), color="#bbbbbb", lw=1)
    for m, a, b, g in _segments(frame, disp=u, scale=scale, model=model):
        ax.plot(*zip(a, b), color="#d62728", lw=1.6)
    _set_iso(ax, frame)
    ax.set_title(f"{title}  (변형 {scale:.0f}배 확대)")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def safety_grid(frame, results_by_case: dict, path: str, sf_cap=4.0):
    """하중조건별 부재 안전율 색상 지도."""
    keys = list(results_by_case)
    n = len(keys)
    cols = 3
    rows = int(np.ceil(n / cols))
    fig = plt.figure(figsize=(6 * cols, 4.6 * rows))
    cmap = plt.get_cmap("RdYlGn")
    for k, key in enumerate(keys):
        name, res = results_by_case[key]
        ax = fig.add_subplot(rows, cols, k + 1, projection="3d")
        for r in res:
            if GROUPS[r.member.group].rule_class == "rigid":
                continue
            a, b = frame.nodes[r.member.a], frame.nodes[r.member.b]
            sf_eff = min(r.sf, 1 / r.buckling_util if r.buckling_util > 0 else np.inf)
            c = cmap(min(sf_eff, sf_cap) / sf_cap)
            ax.plot(*zip(a, b), color=c, lw=2.2)
        worst = min((min(r.sf, 1 / r.buckling_util if r.buckling_util > 0 else np.inf)
                     for r in res if GROUPS[r.member.group].rule_class != "rigid"))
        _set_iso(ax, frame)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        ax.set_title(f"{name}\n최소 안전율 {worst:.2f}", fontsize=10)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, sf_cap))
    fig.colorbar(sm, ax=fig.axes, shrink=0.5, label=f"안전율 (항복·좌굴 중 작은 값, {sf_cap:g} 이상은 같은 색)")
    fig.savefig(path, dpi=110)
    plt.close(fig)
