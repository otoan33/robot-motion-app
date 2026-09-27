"""RMP（Riemannian Motion Policies）で動作を作る。

各空間（先端の位置・先端の姿勢・関節）に置いた RMP（ヤコビアン J、加速度 a、計量 M）を関節空間へ引き戻して合成し、
    q̈ = (Σ JᵀMJ)⁻¹ Σ JᵀMa
を解いて時間積分する（RMPflow の pullback / resolve。各 RMP の加速度 a からは、関節が等速でも先端が加速してしまう分 J̇q̇ を差し引く）。障害物回避などは RMP を leaves に足せば組み込める。
"""
import numpy as np

from backend.kinematics import rotvec
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


# 積分は安定のため dt を 2ms 以下に分けて行う（分割数と刻み）
def substeps(dt: float) -> tuple[int, float]:
    n = int(np.ceil(dt / 0.002))
    return n, dt / n


# 目標到達 RMP による PTP。先端（tool0）の位置・姿勢を目標の関節角度での先端姿勢へ引き寄せつつ、関節空間の弱い引き寄せで最終の関節の形態を目標に合わせる
# 先端の最大速度 [mm/s]・[deg/s] と最大加速度 [mm/s²]・[deg/s²] から各ポリシーの係数を決め、関節の速度・加速度は上限で頭打ちにする
def plan_rmp(start, goal, max_vel, max_acc, lin_vel, lin_acc, rot_vel, rot_acc, dt: float,
             weights=(1.0, 0.3, 0.05), time_limit: float = 60.0, tol: float = 1e-2) -> tuple[list[float], list[list[float]]]:
    q, qd, qg = np.radians(start), np.zeros(6), np.radians(goal)
    vmax, amax = np.radians(max_vel), np.radians(max_acc)
    Tg = arm.forward(qg)[0]
    # 先端の位置 [m]・姿勢 [rad] と関節 [rad] の各ポリシーの最大速度と減衰（β = 最大加速度 / 最大速度 で、動き出しの加速度が最大加速度になる）
    v_p, v_r, v_q = lin_vel / 1000, np.radians(rot_vel), np.min(vmax)
    b_p, b_r = lin_acc / lin_vel, rot_acc / rot_vel
    w_p, w_r, w_q = weights

    n, h = substeps(dt)
    ts, qs = [0.0], [q.copy()]
    while ts[-1] < time_limit:
        for _ in range(n):
            T, J, Jdqd = jacobian(q, qd)
            xd = J @ qd
            # 各 RMP（ヤコビアン, 加速度, 計量）。先端の位置・姿勢、関節の順
            q, qd = step(q, qd, [(J[:3], attractor(Tg[:3, 3] - T[:3, 3], xd[:3], v_p, b_p) - Jdqd[:3], w_p * np.eye(3)),
                                 (J[3:], attractor(rotvec(Tg[:3, :3] @ T[:3, :3].T), xd[3:], v_r, b_r) - Jdqd[3:], w_r * np.eye(3)),
                                 (np.eye(6), attractor(qg - q, qd, v_q, b_p), w_q * np.eye(6))], vmax, amax, h)
        ts.append(len(ts) * dt)
        qs.append(q.copy())
        # 目標の関節角度に十分近づいて止まったら、最後の点を目標ちょうどにして終える
        if np.max(np.abs(np.degrees(qg - q))) < tol and np.max(np.abs(np.degrees(qd))) < tol:
            qs[-1] = qg
            return ts, np.degrees(qs).tolist()
    raise ValueError(f"RMP が {time_limit:.0f} 秒以内に目標へ収束しません（関節の誤差 {np.round(np.degrees(qg - q), 2).tolist()} [deg]）")


# 経路追従 RMP。開始・経由点・目標の先端位置を結んだ折れ線（角は blend [mm] で丸める）に沿って、経路上の最寄り点から
# 接線方向へ経路の速度プロファイルで進みつつ横ずれを戻し、姿勢は最寄り点に割り振った姿勢へ引き寄せる。時刻ではなく経路上の位置で進むため、遅れても急がない
def plan_rmp_path(start, via, goal, max_vel, max_acc, lin_vel, lin_acc, rot_vel, rot_acc, blend: float, dt: float,
                  weights=(1.0, 0.3, 1e-5), time_limit: float = 120.0, tol: float = 1e-2) -> tuple[list[float], list[list[float]]]:
    q, qd, qg = np.radians(start), np.zeros(6), np.radians(goal)
    vmax, amax = np.radians(max_vel), np.radians(max_acc)
    v_p, v_r = lin_vel / 1000, np.radians(rot_vel)
    b_p, b_r = lin_acc / lin_vel, rot_acc / rot_vel
    w_p, w_r, w_q = weights
    path = Path([arm.forward(np.radians(a))[0] for a in [start, *via, goal]], blend / 1000, v_p, lin_acc / 1000, v_r)
    # 終点で止めるときの引き寄せの最大速度（最後の 5 mm・1° を最大加速度で詰める速さ）
    last, v_end, w_end = len(path.S) - 1, np.sqrt(2 * lin_acc / 1000 * 0.005), np.sqrt(2 * np.radians(rot_acc) * np.radians(1))

    n, h = substeps(dt)
    k, ts, qs = 0, [0.0], [q.copy()]
    while ts[-1] < time_limit:
        for _ in range(n):
            T, J, Jdqd = jacobian(q, qd)
            xd = J @ qd
            k = path.nearest(T[:3, 3], k)
            t_hat, sd = path.tangent[k], max(path.tangent[k] @ xd[:3], 0)
            # 位置: 最寄り点へのずれ（終点以外は接線方向を除いた横ずれ）を戻しつつ、経路の速度で接線方向に進む
            # 速度プロファイルの変化と曲がるための向心加速度は、実際の進む速さ sd を使って先回りで与える（遅れても急がない）
            # 終点では、最後の 5 mm・1° を詰める速さと最大加速度に合わせた引き寄せで素早く止める（通常の係数のままだと、着いたときの勢いで行き過ぎて戻りが遅い）
            e = path.points[k] - T[:3, 3]
            if k < last: a_p = attractor(e - t_hat * (t_hat @ e), xd[:3] - path.speed[k] * t_hat, v_p, b_p) + path.dspeed[k] * sd * t_hat + sd**2 * path.kappa[k]
            else: a_p = attractor(e, xd[:3], v_end, lin_acc / 1000 / v_end)
            # 姿勢: 最寄り点の姿勢へ、経路に沿って回る角速度・角加速度込みで引き寄せる
            e_r = rotvec(path.orientation[k] @ T[:3, :3].T)
            if k < last: a_r = attractor(e_r, xd[3:] - path.rot_rate[k] * sd, v_r, b_r) + path.rot_rate[k] * path.dspeed[k] * sd + path.drot_rate[k] * sd**2
            else: a_r = attractor(e_r, xd[3:], w_end, np.radians(rot_acc) / w_end)
            # 計量は横ずれ方向を強く、接線方向を弱くする（障害物回避などと合成したとき、横ずれを優先して抑えつつ進みを譲る）
            # 関節は特異点付近で解を安定させるためのごく弱い正則化（強いと手首など腕の短い関節の動きが鈍り、収束が遅くなる）
            q, qd = step(q, qd, [(J[:3], a_p - Jdqd[:3], w_p * (np.eye(3) - 0.5 * np.outer(t_hat, t_hat))), (J[3:], a_r - Jdqd[3:], w_r * np.eye(3)),
                                 (np.eye(6), np.zeros(6), w_q * np.eye(6))], vmax, amax, h)
        ts.append(len(ts) * dt)
        qs.append(q.copy())
        # 終点で止まったら、関節の形態が目標と同じか確かめ、最後の点を目標ちょうどにして終える
        if k == last and np.linalg.norm(e) < 1e-5 and np.linalg.norm(e_r) < 1e-4 and np.max(np.abs(np.degrees(qd))) < tol:
            if not np.allclose((np.degrees(q - qg) + 180) % 360 - 180, 0, atol=tol):
                raise ValueError(f"経路の終点での関節角度 {np.round(np.degrees(q), 2).tolist()} が目標と一致しません（開始と目標で関節の形態が違います）")
            qs[-1] = qg
            return ts, np.degrees(qs).tolist()
    raise ValueError(f"経路追従が {time_limit:.0f} 秒以内に終点へ収束しません（経路の {path.S[k] * 1000:.0f} / {path.S[-1] * 1000:.0f} mm）")
