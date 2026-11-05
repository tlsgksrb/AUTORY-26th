"""해석 하중 조건. 하중값·경계조건은 출처와 함께 적어 두었다(대부분 가정/차용)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import G, MASSES
from .fea import Model

XYZ = (0, 1, 2)
ALL = (0, 1, 2, 3, 4, 5)

# 부품 질량을 어느 노드에 나눠 실을지 (가중치 합 = 1)
MASS_MAP = {
    "driver": {"FL_a_L": .075, "FL_a_R": .075, "LR_m_L": .2, "LR_m_R": .2, "LRX_C": .1,
               "MH_bot_L": .1, "MH_bot_R": .1, "SH_L": .075, "SH_R": .075},
    "battery": {"MH_bot_L": .3, "MH_bot_R": .3, "LRX_C": .2, "MH_mid_L": .1, "MH_mid_R": .1},
    "motor_inverter": {"RL_a_L": .3, "RL_a_R": .3, "RU_a_L": .2, "RU_a_R": .2},
    "drivetrain": {"RL_b_L": .25, "RL_b_R": .25, "RL_a_L": .25, "RL_a_R": .25},
    "misc": {n: .1 for n in ("FB_bot_L", "FB_bot_R", "FH_mid_L", "FH_mid_R", "SU_m_L", "SU_m_R",
                              "MH_mid_L", "MH_mid_R", "RU_b_L", "RU_b_R")},
}
WHEELS = ("WC_F_L", "WC_F_R", "WC_R_L", "WC_R_R")


@dataclass
class LoadCase:
    key: str
    name: str
    kind: str              # stiffness | operating | crash
    loads: dict
    supports: dict
    note: str


def sprung_masses() -> dict[str, float]:
    out: dict[str, float] = {}
    for part, dist in MASS_MAP.items():
        for n, w in dist.items():
            out[n] = out.get(n, 0.0) + MASSES[part] * w
    return out


def vehicle_mass(model: Model) -> float:
    frame = sum(model.node_masses(frame_extra_ratio=MASSES["frame_extra_ratio"]).values())
    parts = sum(MASSES[k] for k in MASS_MAP) + MASSES["unsprung"]
    return frame + parts


def inertia_loads(model: Model, accel_g) -> dict[str, np.ndarray]:
    """가속도(g 단위 벡터)에 따른 관성력. accel_g 는 '차체가 받는 겉보기 하중 방향'."""
    m = model.node_masses(sprung_masses(), frame_extra_ratio=MASSES["frame_extra_ratio"])
    a = np.asarray(accel_g, float) * G
    return {n: w * a for n, w in m.items() if w > 0}


def build_cases(model: Model) -> list[LoadCase]:
    fr = model.frame
    p = fr.params
    mv = vehicle_mass(model)
    cases: list[LoadCase] = []

    # 1) 비틀림 강성: 앞바퀴 중심에 ±F, 뒷바퀴 중심 고정
    F = 1000.0
    cases.append(LoadCase(
        "torsion", "비틀림 강성", "stiffness",
        {"WC_F_L": [0, 0, F], "WC_F_R": [0, 0, -F]},
        {"WC_R_L": XYZ, "WC_R_R": XYZ, "FBX_C": (2,)},
        f"앞바퀴 중심 ±{F:.0f} N(우력 {F * p.track_f / 1000:.0f} N·m), 뒷바퀴 중심 핀 고정, "
        "서스펜션은 축력 강체 링크(댐퍼 잠금)"))

    # 2) 1g 굽힘
    cases.append(LoadCase("bending_1g", "정적 1 g 굽힘", "stiffness",
                          inertia_loads(model, [0, 0, -1]), {w: XYZ for w in WHEELS},
                          "스프링 상질량 + 프레임 자중, 네 바퀴 중심 핀 지지"))
    # 3) 범프 3 g
    cases.append(LoadCase("bump_3g", "범프 3 g", "operating",
                          inertia_loads(model, [0, 0, -3]), {w: XYZ for w in WHEELS},
                          "수직 3 g (노면 충격), 네 바퀴 중심 핀 지지"))
    # 4) 제동 + 선회 복합
    cases.append(LoadCase("brake_corner", "제동 1.5 g + 선회 1.5 g + 1 g", "operating",
                          inertia_loads(model, [-1.5, 1.5, -1]), {w: XYZ for w in WHEELS},
                          "관성력: 앞(-X) 1.5 g, 좌(+Y) 1.5 g, 수직 1 g"))

    hoop_load = np.array([6000.0, 5000.0, -9000.0])
    hoop_sup = {n: XYZ for n in ("MH_bot_L", "MH_bot_R", "FH_bot_L", "FH_bot_R")}
    # 5) 메인 롤후프
    def top_spread(prefix):
        # 뒤집힌 차가 평지에 닿을 때 후프 상단 직선부 전체가 접촉 → 모서리 1/4씩, 중앙 1/2
        return {f"{prefix}_top_L": hoop_load / 4, f"{prefix}_top_R": hoop_load / 4,
                f"{prefix}_top_C": hoop_load / 2}
    cases.append(LoadCase("main_hoop", "메인 롤후프(전복)", "crash",
                          top_spread("MH"), hoop_sup,
                          "후프 상단 Fx 6 kN(후방), Fy 5 kN, Fz -9 kN — FSAE SES 관례값. "
                          "상단 직선부(모서리 1/4·1/4, 중앙 1/2)에 분포, 메인·전방 후프 하단 변위 고정"))
    # 6) 전방 롤후프
    cases.append(LoadCase("front_hoop", "전방 롤후프(전복)", "crash",
                          top_spread("FH"), hoop_sup, "메인 롤후프와 같은 하중·분포·구속"))
    # 7) 측면 충돌
    side = 7000.0
    cases.append(LoadCase("side_impact", "측면 충돌 7 kN", "crash",
                          {"FH_mid_L": [0, -side / 4, 0], "SU_m_L": [0, -side / 2, 0],
                           "MH_mid_L": [0, -side / 4, 0]},
                          {n: XYZ for n in ("FL_a_R", "FL_b_R", "FU_a_R", "FU_b_R",
                                            "RL_a_R", "RL_b_R", "RU_a_R", "RU_b_R")},
                          "왼쪽 측면충격구조에 7 kN(FSAE SES 관례), 반대쪽 서스펜션 마운트 고정"))
    # 8) 전면 충돌 (범퍼 충격완화장치 평균 20 g 가정)
    fc = mv * G * 20
    cases.append(LoadCase("front_impact", f"전면 충돌 20 g ({fc / 1000:.1f} kN)", "crash",
                          {n: [fc / 4, 0, 0] for n in ("FB_bot_L", "FB_bot_R", "FB_top_L", "FB_top_R")},
                          {n: XYZ for n in ("MH_bot_L", "MH_bot_R", fr.brace_node,
                                            fr.brace_node[:-2] + "_R")},
                          f"차량 {mv:.0f} kg × 20 g 를 범퍼 마운트(벌크헤드 4점)에 입력, "
                          "메인 후프 하단·지지대 결합부 고정(보수적)"))
    # 9) 후면 충돌
    cases.append(LoadCase("rear_impact", f"후면 충돌 20 g ({fc / 1000:.1f} kN)", "crash",
                          {n: [-fc / 4, 0, 0] for n in ("RL_b_L", "RL_b_R", "RU_b_L", "RU_b_R")},
                          {n: XYZ for n in ("MH_bot_L", "MH_bot_R", "MH_mid_L", "MH_mid_R")},
                          "후면 범퍼 마운트(후방 벌크헤드 4점) 입력, 메인 후프 하부 고정"))
    # 10) 6점식 벨트 (전면 20 g 시 드라이버 관성)
    drv = MASSES["driver"] * G * 20
    d = fr.driver
    hip3 = np.array([d["hip"][0], 0, d["hip"][1]])
    sh3 = np.array([d["shoulder"][0], 0, d["shoulder"][1]])
    loads = {}
    for side_n, ysgn in (("L", 1), ("R", -1)):
        sh_node = f"SH_{side_n}"
        v = sh3 + [0, ysgn * p.shoulder_mount_half, 0] - fr.nodes[sh_node]
        loads[sh_node] = 0.20 * drv * v / np.linalg.norm(v)
        lap = f"LR_m_{side_n}"
        v = hip3 + [0, ysgn * 150, 0] - fr.nodes[lap]
        loads[lap] = 0.25 * drv * v / np.linalg.norm(v)
    v = hip3 - fr.nodes["LRX_C"]
    loads["LRX_C"] = 0.10 * drv * v / np.linalg.norm(v)
    cases.append(LoadCase("harness", f"6점식 벨트 하중 ({drv / 1000:.1f} kN)", "crash",
                          loads, {n: XYZ for n in ("FB_bot_L", "FB_bot_R", "FB_top_L", "FB_top_R")},
                          "드라이버 90 kg × 20 g: 어깨 2×20 %, 랩 2×25 %, 안티서브 10 % "
                          "(벨트 방향), 반력은 전방 벌크헤드(범퍼)"))
    return cases
