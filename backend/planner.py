import numpy as np

from backend.kinematics import Arm, matrix_rpy, rotation, rotvec

arm = Arm()


# 経路パラメータ s を 0→1 へ台形速度で動かす。s の最大速度 1/kv・最大加速度 1/ka（経路上で最も厳しい制限から決める）
# duration を指定すると、制限内で最短の時間より長いときだけその時間にする（加速度は上限のまま、頂点速度を下げる）
def trapezoid(kv: float, ka: float, dt: float, duration: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    # 動かないなら指定時間だけその場に留まる
    if kv == 0:
        t = np.append(np.arange(0, (duration or 0) - dt / 2, dt), duration or 0)
        return t, np.zeros_like(t)
    # 最短時間: 最大速度に届くなら台形、届かないなら三角
    t_min = kv + ka / kv if ka / kv**2 < 1 else 2 * np.sqrt(ka)
    T = max(duration or 0, t_min)
    # 同じ加速度のまま T で到着する加速時間（T が長いほど頂点速度が下がる）
    a, ta = 1 / ka, (T - np.sqrt(max(T**2 - 4 * ka, 0))) / 2
    # 加速・等速・減速の区間ごとに s(t) を求める（終端 T は必ず含め、直前の点と近すぎる点は省く）
    t = np.append(np.arange(0, T - dt / 2, dt), T)
    return t, np.where(t < ta, a * t**2 / 2, np.where(t < T - ta, a * ta * (t - ta / 2), 1 - a * (T - t) ** 2 / 2))


# PTP（関節補間）。全関節で共通の s を使い、全関節を同時に加減速・到着させる
# 角度 [deg]、速度 [deg/s]、加速度 [deg/s²]、時刻 [s]
def plan_ptp(start, goal, max_vel, max_acc, dt: float, duration: float | None = None) -> tuple[list[float], list[list[float]]]:
    q0, d = np.asarray(start, float), np.asarray(goal, float) - np.asarray(start, float)
    t, s = trapezoid(np.max(np.abs(d) / max_vel), np.max(np.abs(d) / max_acc), dt, duration)
    return t.tolist(), (q0 + np.outer(s, d)).tolist()


# LIN（直線補間）。先端（tool0）の位置を直線で、姿勢を一定の回転軸まわりに補間し、各時刻の関節角度を逆運動学で求める
# 先端の速度 [mm/s]・加速度 [mm/s²]・角速度 [deg/s]・角加速度 [deg/s²] を守り、関節の速度・加速度の上限を超える場合は全体をゆっくりにする
def plan_lin(start, goal, max_vel, max_acc, lin_vel, lin_acc, rot_vel, rot_acc, dt: float, duration: float | None = None) -> tuple[list[float], list[list[float]]]:
    q0 = np.radians(start)
    T0, T1 = arm.forward(q0)[0], arm.forward(np.radians(goal))[0]
    # 移動距離 [mm] と姿勢の回転（軸 × 角度）
    dp, w = (T1[:3, 3] - T0[:3, 3]) * 1000, rotvec(T1[:3, :3] @ T0[:3, :3].T)
    L, theta = np.linalg.norm(dp), np.degrees(np.linalg.norm(w))
    kv, ka = max(L / lin_vel, theta / rot_vel), max(L / lin_acc, theta / rot_acc)

    # s の列に沿った先端姿勢を、ひとつ前の解を初期値にして順に逆運動学で解く（連続した姿勢の枝をたどる）
    def solve(s):
        q, qs = q0, []
        for si in s:
            target = np.eye(4)
            target[:3, :3], target[:3, 3] = rotation(w / (np.linalg.norm(w) or 1), si * np.linalg.norm(w)) @ T0[:3, :3], T0[:3, 3] + si * dp / 1000
            q, err = arm.inverse(target, q)
            if err > 1e-6: raise ValueError(f"直線経路上で逆運動学が解けません（s={si:.3f}、特異点または可動範囲外）")
            qs.append(q)
        return np.degrees(qs)

    t, s = trapezoid(kv, ka, dt, duration)
    q = solve(s)
    # 関節の速度・加速度の上限に対する超過率。時間を r 倍に引き延ばすと速度は 1/r、加速度は 1/r² になる
    if len(t) > 2:
        v = np.gradient(q, t, axis=0)
        r = max(np.max(np.abs(v) / max_vel), np.sqrt(np.max(np.abs(np.gradient(v, t, axis=0)) / max_acc)))
        if r > 1:
            t, s = trapezoid(kv * r, ka * r**2, dt, duration)
            q = solve(s)
    # 先端姿勢が同じでも関節の形態（手首の反転など）が違う目標には、直線補間では着けない（±360° の違いは同じ姿勢とみなす）
    if not np.allclose((q[-1] - goal + 180) % 360 - 180, 0, atol=1e-3): raise ValueError(f"直線補間の終点 {np.round(q[-1], 2).tolist()} が目標の関節角度と一致しません（開始と目標で関節の形態が違います）")
    return t.tolist(), q.tolist()


# 関節角度 [deg] での先端の位置 [mm] と姿勢 roll / pitch / yaw [deg]
def tcp_pose(angles) -> tuple[list[float], list[float]]:
    T = arm.forward(np.radians(angles))[0]
    return (T[:3, 3] * 1000).tolist(), np.degrees(matrix_rpy(T[:3, :3])).tolist()
