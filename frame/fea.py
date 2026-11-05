"""3차원 골조(Euler-Bernoulli 보 + 트러스) 선형 정적 해석기. numpy 만 사용."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .config import GROUPS, MATERIAL, TUBES
from .geometry import Frame, Member


def _beam_k(E, G, A, I, J, L):
    k = np.zeros((12, 12))
    ea, gj = E * A / L, G * J / L
    k[0, 0] = k[6, 6] = ea; k[0, 6] = k[6, 0] = -ea
    k[3, 3] = k[9, 9] = gj; k[3, 9] = k[9, 3] = -gj
    a, b, c, d = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
    # x-y 평면 굽힘 (v, θz)
    k[1, 1] = k[7, 7] = a; k[1, 7] = k[7, 1] = -a
    for i, j in ((1, 5), (1, 11)):
        k[i, j] = k[j, i] = b
    for i, j in ((7, 5), (7, 11)):
        k[i, j] = k[j, i] = -b
    k[5, 5] = k[11, 11] = c; k[5, 11] = k[11, 5] = d
    # x-z 평면 굽힘 (w, θy)
    k[2, 2] = k[8, 8] = a; k[2, 8] = k[8, 2] = -a
    for i, j in ((2, 4), (2, 10)):
        k[i, j] = k[j, i] = -b
    for i, j in ((8, 4), (8, 10)):
        k[i, j] = k[j, i] = b
    k[4, 4] = k[10, 10] = c; k[4, 10] = k[10, 4] = d
    return k


def _truss_k(E, A, L):
    k = np.zeros((12, 12))
    ea = E * A / L
    k[0, 0] = k[6, 6] = ea; k[0, 6] = k[6, 0] = -ea
    return k


def _rotation(pa, pb):
    ex = (pb - pa) / np.linalg.norm(pb - pa)
    ref = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey)
    ez = np.cross(ex, ey)
    lam = np.vstack([ex, ey, ez])
    T = np.zeros((12, 12))
    for i in range(4):
        T[3 * i:3 * i + 3, 3 * i:3 * i + 3] = lam
    return T


@dataclass
class ElementResult:
    member: Member
    axial: float          # N (+인장)
    moment: float         # N·mm (양 끝 중 큰 값)
    torque: float         # N·mm
    vm: float             # MPa (외곽 섬유, 보수적 합산)
    buckling_util: float  # 압축력 / 오일러 좌굴하중 (K=1)
    sf: float             # 항복강도 / vm


class Model:
    def __init__(self, frame: Frame, material=MATERIAL):
        self.frame = frame
        self.mat = material
        self.names = list(frame.nodes)
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.ndof = 6 * len(self.names)
        self._elems = []
        K = np.zeros((self.ndof, self.ndof))
        beam_nodes = set()
        for m in frame.members:
            g = GROUPS[m.group]
            tube = TUBES[g.tube]
            pa, pb = frame.nodes[m.a], frame.nodes[m.b]
            L = float(np.linalg.norm(pb - pa))
            if m.kind == "truss":
                kl = _truss_k(self.mat.E, tube.area, L)
            else:
                kl = _beam_k(self.mat.E, self.mat.G, tube.area, tube.inertia, tube.polar, L)
                beam_nodes.update((m.a, m.b))
            T = _rotation(pa, pb)
            dofs = np.r_[6 * self.idx[m.a]:6 * self.idx[m.a] + 6, 6 * self.idx[m.b]:6 * self.idx[m.b] + 6]
            K[np.ix_(dofs, dofs)] += T.T @ kl @ T
            self._elems.append((m, tube, L, kl, T, dofs))
        self.K = K
        # 트러스만 연결된 노드는 회전 강성이 없으므로 회전 자유도를 고정
        self.truss_only = [n for n in self.names if n not in beam_nodes]

    # ------------------------------------------------------------------
    def node_masses(self, extra: dict[str, float] | None = None, include_frame=True,
                    frame_extra_ratio=0.10) -> dict[str, float]:
        mass = {n: 0.0 for n in self.names}
        if include_frame:
            for m, tube, L, *_ in self._elems:
                if GROUPS[m.group].rule_class == "rigid":
                    continue
                w = tube.area * L * self.mat.rho * (1 + frame_extra_ratio)
                mass[m.a] += w / 2
                mass[m.b] += w / 2
        for n, w in (extra or {}).items():
            mass[n] += w
        return mass

    def solve(self, loads: dict[str, np.ndarray], supports: dict[str, tuple[int, ...]]):
        F = np.zeros(self.ndof)
        for n, v in loads.items():
            v = np.asarray(v, float)
            F[6 * self.idx[n]:6 * self.idx[n] + len(v)] += v
        fixed = np.zeros(self.ndof, bool)
        for n, dofs in supports.items():
            for d in dofs:
                fixed[6 * self.idx[n] + d] = True
        for n in self.truss_only:
            fixed[6 * self.idx[n] + 3:6 * self.idx[n] + 6] = True
        free = ~fixed
        Kff = self.K[np.ix_(free, free)]
        u = np.zeros(self.ndof)
        u[free] = np.linalg.solve(Kff, F[free])
        resid = np.linalg.norm(Kff @ u[free] - F[free]) / max(np.linalg.norm(F[free]), 1.0)
        if resid > 1e-6:
            raise RuntimeError(f"해석 잔차가 큼({resid:.2e}): 구속 조건/메커니즘 확인 필요")
        reactions = self.K @ u - F
        return u, reactions

    def element_results(self, u) -> list[ElementResult]:
        out = []
        for m, tube, L, kl, T, dofs in self._elems:
            fl = kl @ (T @ u[dofs])
            axial = fl[6]
            m1 = math.hypot(fl[4], fl[5]); m2 = math.hypot(fl[10], fl[11])
            mom = max(m1, m2)
            tq = abs(fl[9])
            sigma = abs(axial) / tube.area + (0 if m.kind == "truss" else mom / tube.modulus)
            tau = 0 if m.kind == "truss" else tq / (2 * tube.modulus)
            vm = math.sqrt(sigma ** 2 + 3 * tau ** 2)
            pcr = math.pi ** 2 * self.mat.E * tube.inertia / L ** 2
            bu = (-axial / pcr) if axial < 0 else 0.0
            sf = self.mat.yield_ / vm if vm > 1e-9 else math.inf
            out.append(ElementResult(m, axial, mom, tq, vm, bu, sf))
        return out

    def disp(self, u, node) -> np.ndarray:
        i = 6 * self.idx[node]
        return u[i:i + 3]
