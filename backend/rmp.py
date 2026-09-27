"""RMP（Riemannian Motion Policies）の目標到達ポリシーで PTP 動作を作る。

各空間（先端の位置・先端の姿勢・関節）に置いた RMP（加速度 a と重み w の計量 M = w·I）を、ヤコビアン J で関節空間へ引き戻して合成し、
    q̈ = (Σ w JᵀJ)⁻¹ Σ w Jᵀ a
を解いて時間積分する（RMPflow の pullback / resolve。J̇q̇ の曲率項は省略）。障害物回避などは RMP を leaves に足せば組み込める。
"""
import numpy as np

from backend.kinematics import rotvec
from backend.planner import arm


# ソフト正規化 s(v) = v / h(|v|)。遠くでは単位ベクトル、目標から η 程度の距離に入ると v に比例する（RMPflow の soft-normalization）
def soft_normalize(v: np.ndarray, eta: float) -> np.ndarray:
    z = np.linalg.norm(v)
    return v / (z + eta * np.log1p(np.exp(-2 * z / eta)))


# 目標到達ポリシー a = α·s(e) - β·ẋ。遠くでは速さ vmax（= α/β）で目標へ向かい、目標の近くでは臨界減衰のばね（k = β²/4）になる
def attractor(e: np.ndarray, xd: np.ndarray, vmax: float, beta: float) -> np.ndarray:
    return beta * vmax * soft_normalize(e, 4 * vmax / (beta * np.log(2))) - beta * xd


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

    # 積分は安定のため dt を細かく分けて行い、dt ごとに記録する
    n = int(np.ceil(dt / 0.002))
    h = dt / n
    ts, qs = [0.0], [q.copy()]
    while ts[-1] < time_limit:
        for _ in range(n):
            T, J = arm.jacobian(q)
            xd = J @ qd
            # 各 RMP（ヤコビアン, 加速度, 重み）。先端の位置・姿勢、関節の順
            leaves = [(J[:3], attractor(Tg[:3, 3] - T[:3, 3], xd[:3], v_p, b_p), w_p),
                      (J[3:], attractor(rotvec(Tg[:3, :3] @ T[:3, :3].T), xd[3:], v_r, b_r), w_r),
                      (np.eye(6), attractor(qg - q, qd, v_q, b_p), w_q)]
            # 関節空間へ引き戻して合成し、関節の加速度を解く
            M, f = sum(w * Jl.T @ Jl for Jl, _, w in leaves), sum(w * Jl.T @ a for Jl, a, w in leaves)
            qdd = np.linalg.solve(M, f)
            # 関節の加速度・速度が上限を超えるときは、向きを保ったまま全体を縮める
            qdd *= min(1, np.min(amax / (np.abs(qdd) + 1e-12)))
            qd += h * qdd
            qd *= min(1, np.min(vmax / (np.abs(qd) + 1e-12)))
            q = q + h * qd
        ts.append(len(ts) * dt)
        qs.append(q.copy())
        # 目標の関節角度に十分近づいて止まったら、最後の点を目標ちょうどにして終える
        if np.max(np.abs(np.degrees(qg - q))) < tol and np.max(np.abs(np.degrees(qd))) < tol:
            qs[-1] = qg
            return ts, np.degrees(qs).tolist()
    raise ValueError(f"RMP が {time_limit:.0f} 秒以内に目標へ収束しません（関節の誤差 {np.round(np.degrees(qg - q), 2).tolist()} [deg]）")
