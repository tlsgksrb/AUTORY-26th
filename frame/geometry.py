"""파라미터로 스페이스프레임 노드/부재를 만든다.

노드 이름 규칙: *_L 은 왼쪽(Y+) 노드이며 자동으로 *_R(Y-)이 대칭 생성된다. *_C 는 중앙(Y=0).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import GROUPS, TEMPLATE, Params


@dataclass
class Member:
    name: str
    a: str
    b: str
    group: str
    kind: str = "beam"   # "beam" | "truss"


@dataclass
class Frame:
    params: Params
    nodes: dict[str, np.ndarray] = field(default_factory=dict)
    members: list[Member] = field(default_factory=list)
    driver: dict = field(default_factory=dict)
    brace_node: str = "MH_br_L"   # 메인 후프 지지대가 붙는 메인 후프 노드(왼쪽)

    def length(self, m: Member) -> float:
        return float(np.linalg.norm(self.nodes[m.b] - self.nodes[m.a]))

    def frame_members(self) -> list[Member]:
        return [m for m in self.members if GROUPS[m.group].rule_class != "rigid"]


def _mirror(name: str) -> str:
    return name[:-2] + "_R" if name.endswith("_L") else name


def _lerp(p, q, x):
    """p→q 직선 위에서 X 좌표가 x 인 점."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    t = (x - p[0]) / (q[0] - p[0])
    return p + t * (q - p)


def _on_segment_z(p, q, z):
    p, q = np.asarray(p, float), np.asarray(q, float)
    t = (z - p[2]) / (q[2] - p[2])
    return p + t * (q - p)


def build(p: Params) -> Frame:
    f = Frame(params=p)
    N = f.nodes

    def node(name, xyz):
        xyz = np.asarray(xyz, float)
        N[name] = xyz
        if name.endswith("_L"):
            N[_mirror(name)] = xyz * np.array([1, -1, 1])

    zf = p.z_floor

    # --- 전방 벌크헤드 -------------------------------------------------------
    node("FB_bot_L", (p.x_fb, p.fb_half_w_bot, zf))
    node("FB_top_L", (p.x_fb, p.fb_half_w_top, p.z_fb_top))
    node("FBX_C", (p.x_fb, 0, zf))

    # --- 전방 롤후프 ---------------------------------------------------------
    node("FH_bot_L", (p.x_fh, p.fh_half_w, zf))
    node("FH_mid_L", (p.x_fh, p.fh_half_w, p.z_sis))
    node("FH_up_L", (p.x_fh, p.fh_half_w_up, p.z_fh_up))
    node("FH_top_L", (p.x_fh, p.fh_half_w_top, p.z_fh_top_corner))
    node("FH_top_C", (p.x_fh, 0, p.z_fh_top))

    # --- 전방 박스(서스펜션 마운트) -----------------------------------------
    for tag, x in (("a", p.x_fs_a), ("b", p.x_fs_b)):
        node(f"FL_{tag}_L", _lerp(N["FB_bot_L"], N["FH_bot_L"], x))
        node(f"FU_{tag}_L", _lerp(N["FB_top_L"], N["FH_mid_L"], x))

    # --- 메인 롤후프 ---------------------------------------------------------
    node("MH_bot_L", (p.x_mh, p.mh_half_w, zf))
    node("MH_mid_L", (p.x_mh, p.mh_half_w, p.z_sis))
    node("MH_sh_L", (p.x_mh, p.mh_half_w_shb, p.z_shb))
    node("MH_top_L", (p.x_mh, p.mh_half_w_top, p.z_mh_top_corner))
    at_corner = p.z_mhb_attach >= p.z_mh_top_corner - 1.0
    br = "MH_top_L" if at_corner else "MH_br_L"
    f.brace_node = br
    if not at_corner:
        node("MH_br_L", _on_segment_z(N["MH_sh_L"], N["MH_top_L"], p.z_mhb_attach))
    node("MH_top_C", (p.x_mh, 0, p.z_mh_top))
    node("SH_L", (p.x_mh, p.shoulder_mount_half, p.z_shb))

    # --- 콕핏 측면 중간 ---------------------------------------------------------
    node("LR_m_L", _lerp(N["FH_bot_L"], N["MH_bot_L"], p.x_cp_mid))
    node("SU_m_L", _lerp(N["FH_mid_L"], N["MH_mid_L"], p.x_cp_mid))
    node("LRX_C", (p.x_cp_mid, 0, zf))

    # --- 후방 프레임 ---------------------------------------------------------
    node("RL_a_L", (p.x_rs_a, p.rr_half_w_a, zf))
    node("RU_a_L", (p.x_rs_a, p.rr_half_w_a, p.z_rr_up_a))
    node("RL_b_L", (p.x_rb, p.rr_half_w_b, zf))
    node("RU_b_L", (p.x_rb, p.rr_half_w_b, p.z_rr_up_b))

    # --- 서스펜션 하드포인트(해석용) ----------------------------------------
    rw = p.tire_d / 2
    for ax, x, tr in (("F", 0.0, p.track_f), ("R", p.wheelbase, p.track_r)):
        node(f"WC_{ax}_L", (x, tr / 2, rw))
        node(f"LBJ_{ax}_L", (x, tr / 2 - p.lbj_in, p.z_lbj))
        node(f"UBJ_{ax}_L", (x, tr / 2 - p.ubj_in, p.z_ubj))

    # ------------------------------------------------------------------------
    M = f.members

    def add(a, b, g, kind="beam", sym=True):
        M.append(Member(f"{a}-{b}", a, b, g, kind))
        if sym and (a.endswith("_L") or b.endswith("_L")):
            ra, rb = _mirror(a), _mirror(b)
            if (ra, rb) != (a, b):
                M.append(Member(f"{ra}-{rb}", ra, rb, g, kind))

    def cross(a_l, g, center=None):
        a_r = _mirror(a_l)
        if center:
            add(a_l, center, g)            # 대칭 생성됨
        else:
            add(a_l, a_r, g, sym=False)

    # 전방 벌크헤드
    add("FB_bot_L", "FB_top_L", "FBH")
    cross("FB_bot_L", "FBH", center="FBX_C")
    cross("FB_top_L", "FBH")
    add("FB_bot_L", "FB_top_R", "FBH", sym=False)          # 벌크헤드 대각

    # 전방 벌크헤드 지지(전방 박스 측면, Warren 트러스)
    add("FB_bot_L", "FL_a_L", "FBS"); add("FL_a_L", "FL_b_L", "FBS"); add("FL_b_L", "FH_bot_L", "FBS")
    add("FB_top_L", "FU_a_L", "FBS"); add("FU_a_L", "FU_b_L", "FBS"); add("FU_b_L", "FH_mid_L", "FBS")
    add("FL_a_L", "FU_a_L", "SEC"); add("FL_b_L", "FU_b_L", "SEC")
    add("FB_bot_L", "FU_a_L", "FBS"); add("FL_a_L", "FU_b_L", "SEC"); add("FL_b_L", "FH_mid_L", "FBS")
    # 전방 박스 바닥
    cross("FL_a_L", "SEC"); cross("FL_b_L", "SEC")
    add("FBX_C", "FL_a_L", "SEC", sym=False)
    add("FL_a_L", "FL_b_R", "SEC", sym=False)
    add("FL_b_R", "FH_bot_L", "SEC", sym=False)

    # 전방 롤후프(한 개의 굽힘 튜브를 폴리라인으로 근사)
    add("FH_bot_L", "FH_mid_L", "FH"); add("FH_mid_L", "FH_up_L", "FH")
    add("FH_up_L", "FH_top_L", "FH"); add("FH_top_L", "FH_top_C", "FH")
    cross("FH_bot_L", "SEC")
    add("FH_bot_L", "FH_mid_R", "SEC", sym=False)           # 전방 롤후프 하부 대각
    if p.dash_bar:
        cross("FH_up_L", "FHB")                              # 대시바
    if p.front_top_diag:
        add("FB_top_L", "FH_top_R", "FHB", sym=False)        # 전방 박스 윗면 대각
    # 전방 롤후프 지지대
    add("FH_top_L", "FB_top_L", "FHB")

    # 콕핏 측면(측면 충격구조) + 콕핏 레일
    add("FH_bot_L", "LR_m_L", "SIS"); add("LR_m_L", "MH_bot_L", "SIS")
    add("FH_mid_L", "SU_m_L", "SIS"); add("SU_m_L", "MH_mid_L", "SIS")
    add("FH_bot_L", "SU_m_L", "SIS"); add("SU_m_L", "MH_bot_L", "SIS")
    add("LR_m_L", "SU_m_L", "SIS")
    add("FH_up_L", "MH_sh_L", "SEC")                         # 콕핏 레일
    add("SU_m_L", "FH_up_L", "SEC"); add("SU_m_L", "MH_sh_L", "SEC")
    # 콕핏 바닥
    cross("LR_m_L", "LAP", center="LRX_C")
    if p.cockpit_floor_x:
        add("FH_bot_L", "LRX_C", "SEC"); add("LRX_C", "MH_bot_L", "SEC")

    # 메인 롤후프
    add("MH_bot_L", "MH_mid_L", "MH"); add("MH_mid_L", "MH_sh_L", "MH")
    if at_corner:
        add("MH_sh_L", "MH_top_L", "MH")
    else:
        add("MH_sh_L", "MH_br_L", "MH"); add("MH_br_L", "MH_top_L", "MH")
    add("MH_top_L", "MH_top_C", "MH")
    cross("MH_bot_L", "SEC"); cross("MH_mid_L", "SEC")
    add("MH_bot_L", "MH_mid_R", "SEC", sym=False)           # 방화벽면 대각
    if p.mh_upper_diag:
        add("MH_mid_R", "SH_L", "SEC", sym=False)            # 방화벽면 상부 대각
    # 어깨벨트 바
    add("MH_sh_L", "SH_L", "SHB"); cross("SH_L", "SHB")
    # 메인 롤후프 지지대
    add(br, "RU_b_L", "MHB")
    if p.mh_head_diag:
        add("MH_sh_R", "MH_top_L", "SEC", sym=False)         # 헤드레스트 뒤 후프면 대각(횡하중)
    if p.shb_support:
        add("SH_L", "RU_a_L", "SHS")

    # 후방 프레임
    add("MH_bot_L", "RL_a_L", "REAR2"); add("RL_a_L", "RL_b_L", "REAR")
    add("MH_mid_L", "RU_a_L", "REAR2"); add("RU_a_L", "RU_b_L", "REAR")
    add("RL_a_L", "RU_a_L", "REAR2"); add("RL_b_L", "RU_b_L", "REAR")
    add("MH_bot_L", "RU_a_L", "REAR2"); add("RL_a_L", "RU_b_L", "REAR2")
    cross("RL_a_L", "REAR2"); cross("RU_a_L", "REAR2")
    cross("RL_b_L", "REAR"); cross("RU_b_L", "REAR")
    add("RL_b_L", "RU_b_R", "REAR", sym=False)              # 후방 벌크헤드 대각
    add("MH_bot_L", "RL_a_R", "REAR2", sym=False)           # 후방 바닥 대각
    add("RL_a_L", "RL_b_R", "REAR2", sym=False)
    if p.rear_top_diag:
        add("RU_a_L", "RU_b_R", "REAR2", sym=False)

    # 서스펜션(축력만 받는 강체 링크) — 비틀림/주행 하중을 휠 중심에서 프레임으로 전달
    for ax, (fa, fb_, ua, ub, push, toe) in {
        "F": ("FL_a_L", "FL_b_L", "FU_a_L", "FU_b_L", "FU_b_L", "FU_a_L"),
        "R": ("RL_a_L", "RL_b_L", "RU_a_L", "RU_b_L", "RU_a_L", "RL_b_L"),
    }.items():
        lbj, ubj, wc = f"LBJ_{ax}_L", f"UBJ_{ax}_L", f"WC_{ax}_L"
        for a in (fa, fb_):
            add(a, lbj, "SUSP", "truss")
        for a in (ua, ub):
            add(a, ubj, "SUSP", "truss")
        add(push, lbj, "SUSP", "truss")          # 푸시로드(댐퍼 잠금)
        add(lbj, ubj, "SUSP", "truss")           # 업라이트
        add(lbj, wc, "SUSP", "truss")
        add(ubj, wc, "SUSP", "truss")
        add(toe, wc, "SUSP", "truss")            # 타이로드/토우링크

    f.driver = driver_template(p)
    return f


def driver_template(p: Params) -> dict:
    """95th 템플릿(엉덩이·어깨·헬멧 원)과 무릎 위치를 측면도(X-Z)로 계산."""
    t = TEMPLATE
    dz = p.hip_z - p.pedal_face_z
    hip_x = p.pedal_face_x + math.sqrt(t["pedal_to_hip"] ** 2 - dz ** 2)
    th = math.radians(p.recline_deg)
    u = np.array([math.sin(th), math.cos(th)])
    hip = np.array([hip_x, p.hip_z])
    shoulder = hip + t["hip_to_shoulder"] * u
    head = shoulder + t["shoulder_to_head"] * u

    # 무릎: 엉덩이~발뒤꿈치 사이 2링크(허벅지 610, 정강이 560 가정)
    heel = np.array([p.pedal_face_x + 50, p.z_floor + 65])
    thigh, shin = t["thigh"], t["shin"]
    d_vec = heel - hip
    d = np.linalg.norm(d_vec)
    a = (thigh ** 2 - shin ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(max(thigh ** 2 - a ** 2, 0))
    uu = d_vec / d
    n = np.array([uu[1], -uu[0]])
    if n[1] < 0:
        n = -n
    knee = hip + a * uu + h * n
    return dict(hip=hip, shoulder=shoulder, head=head, knee=knee, heel=heel,
                r_hip=t["r_hip"], r_shoulder=t["r_shoulder"], r_head=t["r_head"])
