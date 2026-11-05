"""설계 입력값 모음.

단위: 길이 mm, 힘 N, 응력 MPa(N/mm^2), 질량 kg.
좌표계: X = 앞차축 중심에서 뒤쪽(+), Y = 왼쪽(+), Z = 지면에서 위쪽(+).

규정값마다 출처와 확인 상태를 달아 두었다. 2027 대회 규정집 원문을 받으면
RULES 의 value 만 바꾸고 `python run_frame.py` 를 다시 돌리면 된다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# 규정 / 설계 기준
# ---------------------------------------------------------------------------
# status 값
CONFIRMED_PRESS = "2026 변경 확인(언론 보도)"
EV_PARTIAL = "EV 규정(2022) 일부 확인"
BORROWED_KSAE = "KSAE 타 부문 규정 차용 · EV 원문 미확인"
BORROWED_FSAE = "FSAE 관례 차용 · EV 원문 미확인"
OWN_TARGET = "자체 설계 목표"


@dataclass(frozen=True)
class Rule:
    key: str
    text: str
    value: float | None
    unit: str
    source: str
    status: str


RULES: dict[str, Rule] = {r.key: r for r in [
    Rule("wheelbase_min", "휠베이스 하한", 1500, "mm",
         "2026 대회 보도: 1,250 → 1,500 mm 이상으로 상향", CONFIRMED_PRESS),
    Rule("tire_width_min", "타이어 폭 하한", 200, "mm",
         "2026 대회 보도: 125 → 200 mm 이상", CONFIRMED_PRESS),
    Rule("harness_points", "안전벨트 점식", 6, "점",
         "2026 대회 보도: 4점식 → 6점식", CONFIRMED_PRESS),
    Rule("bumper_absorber", "전·후면 범퍼 충격완화장치", 1, "필수",
         "2026 대회 보도", CONFIRMED_PRESS),
    Rule("motor_power_max", "모터 출력 상한", 10, "kW",
         "2026 대회 보도", CONFIRMED_PRESS),
    Rule("battery_voltage_max", "배터리 전압 상한", 59, "V",
         "2026 대회 보도", CONFIRMED_PRESS),
    Rule("hoop_od_min", "롤후프 강관 외경 하한", 25.0, "mm",
         "EV 차량기술규정(2022.11.17 시행) 검색 발췌", EV_PARTIAL),
    Rule("hoop_t_min", "메인·전방 롤후프, 어깨벨트 바 두께 하한", 1.8, "mm",
         "FSK 2026 규정 표(25×1.8), Baja 2020·KASA 규정(25 mm, 1.8 mm 이상)", BORROWED_KSAE),
    Rule("primary_t_min", "롤후프 지지대·측면/전방 충격구조 두께 하한(외경 25)", 1.6, "mm",
         "FSK 2026 규정 표(25×1.6)", BORROWED_KSAE),
    Rule("secondary_od_min", "기타 프레임재 외경 하한", 20.0, "mm",
         "FSK 2026 규정 표(20×1.2)", BORROWED_KSAE),
    Rule("secondary_t_min", "기타 프레임재 두께 하한", 1.2, "mm",
         "FSK 2026 규정 표(20×1.2)", BORROWED_KSAE),
    Rule("yield_min", "강관 항복강도 하한", 285, "MPa",
         "Baja 2020 규정(285 MPa) · KASA 2020(300 MPa)", BORROWED_KSAE),
    Rule("tensile_min", "강관 인장강도 하한", 360, "MPa",
         "Baja 2020 규정(360 MPa) · KASA 2020(365 MPa)", BORROWED_KSAE),
    Rule("helmet_clearance", "헬멧 상단 ~ 전방/메인 롤후프 연결선 여유", 50.8, "mm",
         "FSAE 95th 템플릿 기준", BORROWED_FSAE),
    Rule("mhb_angle_min", "메인 롤후프-지지대 각도 하한(측면도)", 30, "deg",
         "FSAE", BORROWED_FSAE),
    Rule("mhb_attach_max", "메인 롤후프 지지대 부착점: 후프 상단에서 이내", 160, "mm",
         "FSAE", BORROWED_FSAE),
    Rule("fh_to_sw_max", "전방 롤후프가 스티어링 휠보다 앞서는 거리 상한", 250, "mm",
         "FSAE", BORROWED_FSAE),
    Rule("shoulder_angle_up", "어깨벨트 각도 상한(수평 위)", 10, "deg", "FSAE", BORROWED_FSAE),
    Rule("shoulder_angle_down", "어깨벨트 각도 하한(수평 아래)", 20, "deg", "FSAE", BORROWED_FSAE),
    Rule("shoulder_spacing_min", "어깨벨트 마운트 간격 하한", 178, "mm", "FSAE", BORROWED_FSAE),
    Rule("shoulder_spacing_max", "어깨벨트 마운트 간격 상한", 229, "mm", "FSAE", BORROWED_FSAE),
    Rule("lap_angle_min", "랩벨트 각도 하한(측면도)", 45, "deg", "FSAE", BORROWED_FSAE),
    Rule("lap_angle_max", "랩벨트 각도 상한(측면도)", 65, "deg", "FSAE", BORROWED_FSAE),
    Rule("hoop_deflection_max", "롤후프/측면충돌 하중시 최대 변위", 25, "mm",
         "FSAE SES 관례", BORROWED_FSAE),
    Rule("torsion_target", "비틀림 강성 목표", 1500, "N·m/deg",
         "철제 스페이스프레임 FSAE급 통상 범위 하단", OWN_TARGET),
    Rule("frame_mass_target", "프레임 질량 목표(용접·탭 포함)", 35, "kg", "", OWN_TARGET),
    Rule("sf_operating", "주행 하중 안전율 하한(항복 기준)", 1.5, "-", "", OWN_TARGET),
    Rule("sf_crash", "충돌·전복 하중 안전율 하한(항복 기준)", 1.0, "-", "", OWN_TARGET),
    Rule("ground_clearance_min", "프레임 최저지상고(설계값)", 40, "mm",
         "규정값 미확인, 자체 기준", OWN_TARGET),
]}


# ---------------------------------------------------------------------------
# 재료 / 튜브
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Material:
    name: str
    E: float = 205_000.0     # MPa
    G: float = 79_000.0      # MPa
    rho: float = 7.85e-6     # kg/mm^3
    yield_: float = 380.0    # MPa
    tensile: float = 510.0   # MPa
    note: str = ""


# KS D 3517 / JIS G3445 기계구조용 탄소강관. 13A(항복 215 MPa)는 차용 규정의 285 MPa를
# 못 넘기므로 13C를 기본으로 둔다. 4130 크로몰리로 바꾸려면 아래 CRMO 를 쓴다.
STKM13C = Material("STKM13C (KS D 3517)", yield_=380.0, tensile=510.0,
                   note="C ≤ 0.25 %, 일반 TIG/MIG 용접 가능")
CRMO4130 = Material("AISI 4130 (노멀라이즈드)", yield_=460.0, tensile=670.0,
                    note="TIG 권장, 용접부 열영향 고려")
MATERIAL = STKM13C


@dataclass(frozen=True)
class Tube:
    od: float
    t: float

    @property
    def inner(self) -> float:
        return self.od - 2 * self.t

    @property
    def area(self) -> float:
        return math.pi / 4 * (self.od ** 2 - self.inner ** 2)

    @property
    def inertia(self) -> float:
        return math.pi / 64 * (self.od ** 4 - self.inner ** 4)

    @property
    def polar(self) -> float:
        return 2 * self.inertia

    @property
    def modulus(self) -> float:
        return self.inertia / (self.od / 2)

    @property
    def label(self) -> str:
        return f"Ø{self.od:g}×{self.t:g}"


# 국내 유통이 많은 인치계 규격(25.4 = 1", 22.2 = 7/8"). 시장 재고는 납품처에 꼭 확인.
TUBES = {
    "T0": Tube(31.8, 2.0),   # 전방 롤후프 (전복 횡하중에서 다리 굽힘이 지배)
    "T1": Tube(25.4, 2.3),   # 메인/전방 롤후프, 어깨벨트 바 (규정 하한 1.8 대비 전복 하중 여유)
    "T2": Tube(25.4, 1.6),   # 지지대, 측면충돌구조, 전방 벌크헤드/지지, 후방 레일
    "T3": Tube(22.2, 1.2),   # 기타(바닥 대각, 가로재 등)
    "RIGID": Tube(60.0, 10.0),  # 해석용 강체 링크(서스펜션 대용) — 질량·응력 집계 제외
}


@dataclass(frozen=True)
class Group:
    key: str
    name: str
    tube: str
    rule_class: str   # "hoop" | "primary" | "secondary" | "rigid"
    color: str


GROUPS: dict[str, Group] = {g.key: g for g in [
    Group("MH", "메인 롤후프", "T1", "hoop", "#d62728"),
    Group("FH", "전방 롤후프", "T0", "hoop", "#ff7f0e"),
    Group("SHB", "어깨벨트 바", "T1", "hoop", "#8c564b"),
    Group("MHB", "메인 롤후프 지지대", "T2", "primary", "#e377c2"),
    Group("SHS", "어깨벨트 바 지지", "T2", "primary", "#f7b6d2"),
    Group("FHB", "전방 롤후프 지지대", "T2", "primary", "#bcbd22"),
    Group("SIS", "측면 충격구조", "T2", "primary", "#1f77b4"),
    Group("LAP", "랩·안티서브 벨트 가로재", "T2", "primary", "#aec7e8"),
    Group("FBH", "전방 벌크헤드", "T2", "primary", "#2ca02c"),
    Group("FBS", "전방 벌크헤드 지지", "T2", "primary", "#17becf"),
    Group("REAR", "후방 벌크헤드·후방 지지", "T2", "primary", "#9467bd"),
    Group("REAR2", "후방 프레임(모터 베이)", "T3", "secondary", "#c5b0d5"),
    Group("SEC", "기타 프레임재", "T3", "secondary", "#7f7f7f"),
    Group("SUSP", "서스펜션 링크(해석용 강체)", "RIGID", "rigid", "#c7c7c7"),
]}


# ---------------------------------------------------------------------------
# 차량 · 프레임 형상 파라미터
# ---------------------------------------------------------------------------
@dataclass
class Params:
    # 차량
    wheelbase: float = 1550.0
    track_f: float = 1200.0
    track_r: float = 1150.0
    tire_d: float = 520.0           # 20.5x8.0-13 급 가정
    tire_w: float = 203.0

    # 공통 높이
    z_floor: float = 65.0           # 하부 레일 중심 높이
    z_sis: float = 330.0            # 측면충격구조 상부 부재 중심 높이

    # 전방 벌크헤드
    x_fb: float = -430.0
    fb_half_w_bot: float = 160.0
    fb_half_w_top: float = 140.0
    z_fb_top: float = 380.0

    # 전방 서스펜션 마운트 x (앞차축 기준)
    x_fs_a: float = -130.0
    x_fs_b: float = 130.0

    # 전방 롤후프
    x_fh: float = 380.0
    fh_half_w: float = 255.0
    z_fh_up: float = 540.0
    fh_half_w_up: float = 235.0
    z_fh_top_corner: float = 740.0
    fh_half_w_top: float = 160.0
    z_fh_top: float = 760.0

    # 콕핏 중간 노드 (랩벨트 마운트 겸)
    x_cp_mid: float = 770.0

    # 메인 롤후프
    x_mh: float = 1290.0
    mh_half_w: float = 285.0
    z_shb: float = 600.0
    mh_half_w_shb: float = 280.0
    z_mhb_attach: float = 1180.0    # = 상단 모서리 높이면 모서리에 직접 부착
    z_mh_top_corner: float = 1180.0
    mh_half_w_top: float = 160.0
    z_mh_top: float = 1200.0
    shoulder_mount_half: float = 95.0

    # 후방 프레임
    x_rs_a: float = 1450.0
    x_rb: float = 1740.0
    rr_half_w_a: float = 250.0
    rr_half_w_b: float = 210.0
    z_rr_up_a: float = 340.0
    z_rr_up_b: float = 420.0

    # 보강 옵션(형상 비교용)
    front_top_diag: bool = True     # 전방 박스 윗면 대각(다리 위)
    dash_bar: bool = True           # 전방 후프 대시바(스티어링 컬럼 마운트 겸)
    mh_upper_diag: bool = True      # 메인 후프면 상부 대각(방화벽면)
    mh_head_diag: bool = True       # 메인 후프면 헤드레스트 뒤 대각(전복 횡하중)
    shb_support: bool = True        # 어깨벨트 바 지지대(후방 프레임으로)
    rear_top_diag: bool = False     # 후방 모터 베이 윗면 대각(정비성 저하)
    # v0.2: 부재 제거 검토(모든 체크 유지)에서 빼도 된다고 나온 부재 — 기본 제외
    cockpit_rail: bool = False      # 콕핏 레일(전방 후프 대시 높이 ~ 어깨벨트 바)
    rear_top_cross: bool = False    # 모터 베이 상부 가로재(X 1,450)
    rear_bulkhead_diag: bool = False  # 후방 벌크헤드 대각
    rear_floor_diag2: bool = False  # 모터 베이 바닥 뒤쪽 대각
    cockpit_floor_x: bool = False   # 콕핏 바닥 V 대각(비틀림 기여 작음 → 기본 제외)

    # 서스펜션(해석용 하드포인트)
    lbj_in: float = 70.0            # 볼조인트가 휠 중심면 안쪽으로 들어온 거리
    ubj_in: float = 90.0
    z_lbj: float = 140.0
    z_ubj: float = 390.0

    # 범퍼(충격완화장치 포함 돌출량)
    bumper_f: float = 150.0
    bumper_r: float = 150.0

    # 드라이버(95th 템플릿) · 조향
    pedal_face_x: float = -230.0
    pedal_face_z: float = 300.0
    hip_z: float = 190.0
    recline_deg: float = 30.0       # 상체 기울기(수직 기준)
    sw_x: float = 450.0
    sw_z: float = 610.0
    sw_r: float = 125.0
    headrest_gap: float = 70.0      # 헬멧 뒤 ~ 메인 롤후프 중심 (헤드레스트 패드+간격)


# 95th 백분위 남성 템플릿(FSAE "Percy" 치수 차용)
TEMPLATE = dict(r_hip=100.0, r_shoulder=100.0, r_head=150.0,
                hip_to_shoulder=490.0, shoulder_to_head=280.0, pedal_to_hip=915.0,
                # 다리(무릎 간섭 확인용, 95th 남성 근사 — Size Korea 치수로 갱신 권장)
                thigh=500.0, shin=560.0, knee_r=60.0, knee_y=110.0)

# 패키징 영역(측면도 상자, x0, x1, z0, z1, 반폭) — 질량 배치와 같은 위치
PACKAGING = {
    "배터리(≤59 V)": (900.0, 1265.0, 85.0, 310.0, 190.0),
    "모터·인버터": (1330.0, 1520.0, 110.0, 330.0, 150.0),
}

# 질량 가정 (kg)
MASSES = dict(driver=90.0, battery=22.0, motor_inverter=20.0, drivetrain=10.0,
              misc=25.0, unsprung=40.0, frame_extra_ratio=0.10)

G = 9.81  # m/s^2
