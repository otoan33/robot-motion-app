"""RMP（Riemannian Motion Policies）で動作を作る。

各空間（先端の位置・先端の姿勢・関節）に置いた RMP（ヤコビアン J、加速度 a、計量 M）を関節空間へ引き戻して合成し、
    q̈ = (Σ JᵀMJ)⁻¹ Σ JᵀMa
を解いて時間積分する（RMPflow の pullback / resolve。各 RMP の加速度 a からは、関節が等速でも先端が加速してしまう分 J̇q̇ を差し引く）。
障害物回避は、robot-viser-app の距離計算から作った RMP を同じ列に足して合成する。
"""
import numpy as np

from backend.kinematics import rotvec
from backend.obstacles import distances
from backend.path import Path
from backend.planner import arm


# ソフト正規化 s(v) = v / h(|v|)。遠くでは単位ベクトル、目標から η 程度の距離に入ると v に比例する（RMPflow の soft-normalization）
def soft_normalize(v: np.ndarray, eta: float) -> np.ndarray:
    z = np.linalg.norm(v)
    return v / (z + eta * np.log1p(np.exp(-2 * z / eta)))


# 目標到達ポリシー a = α·s(e) - β·ẋ。遠くでは速さ vmax（= α/β）で目標へ向かい、目標の近くでは臨界減衰のばね（k = β²/4）になる
def attractor(e: np.ndarray, xd: np.ndarray, vmax: float, beta: float) -> np.ndarray:
    return beta * vmax * soft_normalize(e, 4 * vmax / (beta * np.log(2))) - beta * xd


# RMP の列を関節空間で合成して関節の加速度を解き、h 秒進める。関節の加速度・速度が上限を超えるときは、向きを保ったまま全体を縮める
def step(q, qd, leaves, vmax, amax, h: float) -> tuple[np.ndarray, np.ndarray]:
    M, f = sum(J.T @ M @ J for J, _, M in leaves), sum(J.T @ M @ a for J, a, M in leaves)
    qdd = np.linalg.solve(M, f)
    qdd *= min(1, np.min(amax / (np.abs(qdd) + 1e-12)))
    qd = qd + h * qdd
    qd *= min(1, np.min(vmax / (np.abs(qd) + 1e-12)))
    return q + h * qd, qd


# 先端の 4x4 変換・ヤコビアンと、関節が等速のときの先端の加速度 J̇q̇（少し先の姿勢のヤコビアンとの差から求める）
def jacobian(q, qd) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    T, J = arm.jacobian(q)
    return T, J, (arm.jacobian(q + 1e-6 * qd)[1] - J) @ qd / 1e-6


class Avoidance:
    """障害物回避 RMP（RMPflow の衝突回避 RMP にならう）。近似球と障害物の組ごとに、距離 d（1 次元）の空間に置く。z = r/d - 1（影響距離 r で 0、近いほど大きい）として
    - 加速度 a = κ·a_max·z² + β·max(-ḋ, 0)·(z + 1)。押し返しは弱くし、主に近づく速さを近いほど強くブレーキする
    - 計量 m = w·z²·u(ḋ)。u は近づいているときだけ 1 に近づく（u = ε + (1-ε)(1 - exp(-min(ḋ,0)²/2σ²))）
    近づいていなければ他の RMP をほとんど邪魔しないので、障害物に沿って滑るように避ける（止まったままでも z² で計量は増えるので、めり込む前に釣り合う）。
    robot-viser-app の /distances は dt ごとに問い合わせ、その間の細かい刻みでは距離をヤコビアンで線形に進める。"""

    def __init__(self, r: float, a_max: float, beta: float, w: float = 10.0, kappa: float = 0.1, eps: float = 0.05, sigma: float = 0.05):
        self.r, self.a_max, self.beta, self.w, self.kappa, self.eps, self.sigma = r, a_max, beta, w, kappa, eps, sigma
        self.pairs, self.q0, self.min_distance = [], None, np.inf

    # 今の関節角度 q [rad] で、影響距離の内側にある組の距離とヤコビアンを取り直し、軌道全体での最小距離を記録する
    # めり込んだ（距離が負）軌道は使えないので、その時点で止める（開始姿勢で既にめり込んでいる場合も含む）
    def update(self, q) -> None:
        self.pairs, d_min = distances(np.degrees(q), self.r)
        self.q0 = q.copy()
        if d_min is None: return
        self.min_distance = min(self.min_distance, d_min)
        if d_min < 0: raise ValueError(f"障害物にめり込みました（距離 {d_min * 1000:.1f} mm、関節角度 {np.round(np.degrees(q), 1).tolist()} [deg]）。障害物の配置や経路を見直してください")

    # 軌道全体での最小距離 [mm]（障害物がなければ None）
    def result(self) -> float | None:
        return self.min_distance * 1000 if np.isfinite(self.min_distance) else None

    # 組ごとの RMP（ヤコビアン 1×6, 加速度, 計量）
    def leaves(self, q, qd) -> list:
        out = []
        for d0, Jd in self.pairs:
            d, dd = d0 + Jd @ (q - self.q0), Jd @ qd
            z = max(self.r / max(d, 1e-3) - 1, 0)
            u = self.eps + (1 - self.eps) * (1 - np.exp(-min(dd, 0) ** 2 / (2 * self.sigma**2)))
            out.append((Jd[None], np.array([self.kappa * self.a_max * z**2 + self.beta * max(-dd, 0) * (z + 1)]), np.array([[self.w * z**2 * u]])))
        return out


# 積分は安定のため dt を 2ms 以下に分けて行う（分割数と刻み）
def substeps(dt: float) -> tuple[int, float]:
    n = int(np.ceil(dt / 0.002))
    return n, dt / n


# 目標到達 RMP による PTP。先端（tool0）の位置・姿勢を目標の関節角度での先端姿勢へ引き寄せつつ、関節空間の弱い引き寄せで最終の関節の形態を目標に合わせる
# 先端の最大速度 [mm/s]・[deg/s] と最大加速度 [mm/s²]・[deg/s²] から各ポリシーの係数を決め、関節の速度・加速度は上限で頭打ちにする
# avoid [mm] を指定すると、その影響距離で障害物回避 RMP を合成する（返り値の最後は軌道全体での障害物との最小距離 [mm]。回避なしなら None）
def plan_rmp(start, goal, max_vel, max_acc, lin_vel, lin_acc, rot_vel, rot_acc, dt: float, avoid: float | None = None,
             weights=(1.0, 0.3, 0.05), time_limit: float = 60.0, tol: float = 1e-2) -> tuple[list[float], list[list[float]], float | None]:
    q, qd, qg = np.radians(start), np.zeros(6), np.radians(goal)
    vmax, amax = np.radians(max_vel), np.radians(max_acc)
    Tg = arm.forward(qg)[0]
    # 先端の位置 [m]・姿勢 [rad] と関節 [rad] の各ポリシーの最大速度と減衰（β = 最大加速度 / 最大速度 で、動き出しの加速度が最大加速度になる）
    v_p, v_r, v_q = lin_vel / 1000, np.radians(rot_vel), np.min(vmax)
    b_p, b_r = lin_acc / lin_vel, rot_acc / rot_vel
    w_p, w_r, w_q = weights
    # 障害物回避の押し返しは先端の最大加速度、近づく速さの減衰は先端と同じ β
    obs = Avoidance(avoid / 1000, lin_acc / 1000, b_p) if avoid else None

    n, h = substeps(dt)
    ts, qs, stalled = [0.0], [q.copy()], 0.0
    while ts[-1] < time_limit:
        if obs: obs.update(q)
        for _ in range(n):
            T, J, Jdqd = jacobian(q, qd)
            xd = J @ qd
            # 各 RMP（ヤコビアン, 加速度, 計量）。先端の位置・姿勢、関節、障害物回避の順
            q, qd = step(q, qd, [(J[:3], attractor(Tg[:3, 3] - T[:3, 3], xd[:3], v_p, b_p) - Jdqd[:3], w_p * np.eye(3)),
                                 (J[3:], attractor(rotvec(Tg[:3, :3] @ T[:3, :3].T), xd[3:], v_r, b_r) - Jdqd[3:], w_r * np.eye(3)),
                                 (np.eye(6), attractor(qg - q, qd, v_q, b_p), w_q * np.eye(6)), *(obs.leaves(q, qd) if obs else [])], vmax, amax, h)
        ts.append(len(ts) * dt)
        qs.append(q.copy())
        # 目標の関節角度に十分近づいて止まったら、最後の点を目標ちょうどにして終える
        err, moving = np.max(np.abs(np.degrees(qg - q))), np.max(np.abs(np.degrees(qd))) >= tol
        if err < tol and not moving:
            qs[-1] = qg
            return ts, np.degrees(qs).tolist(), obs.result() if obs else None
        # 目標の手前で 0.5 秒止まったままなら、障害物との釣り合い（局所解）などで着けない
        stalled = 0.0 if moving else stalled + dt
        if stalled > 0.5: raise ValueError(f"RMP が目標の手前で止まりました（障害物による局所解、または関節の形態の違い。関節の誤差 {np.round(np.degrees(qg - q), 2).tolist()} [deg]）")
    raise ValueError(f"RMP が {time_limit:.0f} 秒以内に目標へ収束しません（関節の誤差 {np.round(np.degrees(qg - q), 2).tolist()} [deg]）")


# 経路追従 RMP。開始・経由点・目標の先端位置を結んだ折れ線（角は blend [mm] で丸める）に沿って、経路上の最寄り点から
# 接線方向へ経路の速度プロファイルで進みつつ横ずれを戻し、姿勢は最寄り点に割り振った姿勢へ引き寄せる。時刻ではなく経路上の位置で進むため、遅れても急がない
# avoid [mm] を指定すると、その影響距離で障害物回避 RMP を合成する（返り値の最後は軌道全体での障害物との最小距離 [mm]。回避なしなら None）
def plan_rmp_path(start, via, goal, max_vel, max_acc, lin_vel, lin_acc, rot_vel, rot_acc, blend: float, dt: float, avoid: float | None = None,
                  weights=(1.0, 0.3, 1e-5), time_limit: float = 120.0, tol: float = 1e-2) -> tuple[list[float], list[list[float]], float | None]:
    q, qd, qg = np.radians(start), np.zeros(6), np.radians(goal)
    vmax, amax = np.radians(max_vel), np.radians(max_acc)
    a_p, a_r = lin_acc / 1000, np.radians(rot_acc)
    w_p, w_r, w_q = weights
    path = Path([arm.forward(np.radians(a))[0] for a in [start, *via, goal]], blend / 1000, lin_vel / 1000, a_p, np.radians(rot_vel))
    # 接線方向の進みは、速さの差に β = 最大加速度 / 最大速度 を掛けて合わせる（動き出しの加速度が最大加速度になる）
    # 横ずれ・姿勢のずれ・終点の詰めは、経路の速度プロファイルの最後（v = ω_end·r）と同じ速さで収まる硬い臨界減衰（β = 2ω_end）で戻す。戻す加速度は最大加速度まで
    last, b_t, b_fix = len(path.S) - 1, lin_acc / lin_vel, 2 * path.omega_end
    obs = Avoidance(avoid / 1000, a_p, b_t) if avoid else None

    n, h = substeps(dt)
    k, sd, ts, qs, stalled = 0, 0.0, [0.0], [q.copy()], 0.0
    while ts[-1] < time_limit:
        if obs: obs.update(q)
        for _ in range(n):
            T, J, Jdqd = jacobian(q, qd)
            xd = J @ qd
            k = path.nearest(T[:3, 3], k)
            # 接線方向に進む速さ sd と、その変化（ひとつ前の刻みとの差）から求めた実際の加速度 sdd
            t_hat, sd_prev = path.tangent[k], sd
            sd = max(t_hat @ xd[:3], 0)
            sdd = (sd - sd_prev) / h
            # 位置: 経路の途中は横ずれ（接線方向を除いたずれ）を硬く戻しつつ、接線方向は速度プロファイルの速さで進む
            # 速度プロファイルの変化と曲がるための向心加速度は、実際の進む速さ sd を使って先回りで与える（遅れても急がない）。終点では残りのずれを同じ硬さで戻して止める
            e = path.points[k] - T[:3, 3]
            if k < last:
                N = np.eye(3) - np.outer(t_hat, t_hat)
                a_lin = attractor(N @ e, N @ xd[:3], a_p / b_fix, b_fix) + (b_t * (path.speed[k] - t_hat @ xd[:3]) + path.dspeed[k] * sd) * t_hat + sd**2 * path.kappa[k]
            else: a_lin = attractor(e, xd[:3], a_p / b_fix, b_fix)
            # 姿勢: 最寄り点の姿勢へ、経路に沿って回る角速度・角加速度込みで引き寄せる（角加速度は、予定ではなく実際の進み方の加速度 sdd から求める）
            e_r = rotvec(path.orientation[k] @ T[:3, :3].T)
            a_rot = attractor(e_r, xd[3:] - path.rot_rate[k] * sd, a_r / b_fix, b_fix) + path.rot_rate[k] * sdd + path.drot_rate[k] * sd**2
            # 計量は横ずれ方向を強く、接線方向を弱くする（障害物回避などと合成したとき、横ずれを優先して抑えつつ進みを譲る）
            # 関節は特異点付近で解を安定させるためのごく弱い正則化（強いと手首など腕の短い関節の動きが鈍り、収束が遅くなる）
            q, qd = step(q, qd, [(J[:3], a_lin - Jdqd[:3], w_p * (np.eye(3) - 0.5 * np.outer(t_hat, t_hat))), (J[3:], a_rot - Jdqd[3:], w_r * np.eye(3)),
                                 (np.eye(6), np.zeros(6), w_q * np.eye(6)), *(obs.leaves(q, qd) if obs else [])], vmax, amax, h)
        ts.append(len(ts) * dt)
        qs.append(q.copy())
        # 終点で止まったら、関節の形態が目標と同じか確かめ、最後の点を目標ちょうどにして終える
        if k == last and np.linalg.norm(e) < 1e-5 and np.linalg.norm(e_r) < 1e-4 and np.max(np.abs(np.degrees(qd))) < tol:
            if not np.allclose((np.degrees(q - qg) + 180) % 360 - 180, 0, atol=tol):
                raise ValueError(f"経路の終点での関節角度 {np.round(np.degrees(q), 2).tolist()} が目標と一致しません（開始と目標で関節の形態が違います）")
            qs[-1] = qg
            return ts, np.degrees(qs).tolist(), obs.result() if obs else None
        # 終点の手前で 0.5 秒止まったままなら、経路が障害物を通り抜けるなどで進めない
        stalled = 0.0 if np.max(np.abs(np.degrees(qd))) >= tol else stalled + dt
        if stalled > 0.5: raise ValueError(f"経路追従が終点の手前で止まりました（経路の {path.S[k] * 1000:.0f} / {path.S[-1] * 1000:.0f} mm。経路が障害物を通り抜けているなど）")
    raise ValueError(f"経路追従が {time_limit:.0f} 秒以内に終点へ収束しません（経路の {path.S[k] * 1000:.0f} / {path.S[-1] * 1000:.0f} mm）")
