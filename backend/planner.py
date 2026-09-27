import numpy as np


# 台形速度の PTP（関節補間）。全関節で共通の経路パラメータ s∈[0,1] を台形速度で動かし、全関節を同時に加減速・到着させる
# 角度 [deg]、速度 [deg/s]、加速度 [deg/s²]、時刻 [s]。duration を指定すると、制限内で最短の時間より長いときだけその時間にする
def plan_ptp(start, goal, max_vel, max_acc, dt: float, duration: float | None = None) -> tuple[list[float], list[list[float]]]:
    q0, d = np.asarray(start, float), np.asarray(goal, float) - np.asarray(start, float)
    # 移動量がなければ開始姿勢に留まる
    if not d.any():
        times = np.append(np.arange(0, (duration or 0) - dt / 2, dt), duration or 0)
        return times.tolist(), np.tile(q0, (len(times), 1)).tolist()

    # s の速度・加速度の上限は、制限に対して最も厳しい関節で決まる（s の最大速度 1/kv、最大加速度 1/ka）
    kv, ka = np.max(np.abs(d) / max_vel), np.max(np.abs(d) / max_acc)
    # 最短時間: 最大速度に届くなら台形、届かないなら三角
    t_min = kv + ka / kv if ka / kv**2 < 1 else 2 * np.sqrt(ka)
    T = max(duration or 0, t_min)
    # 同じ加速度のまま T で到着する加速時間（T が長いほど頂点速度が下がる）
    a = 1 / ka
    ta = (T - np.sqrt(max(T**2 - 4 * ka, 0))) / 2

    # 加速・等速・減速の区間ごとに s(t) を求め、各関節の角度にする（終端 T は必ず含め、直前の点と近すぎる点は省く）
    t = np.append(np.arange(0, T - dt / 2, dt), T)
    s = np.where(t < ta, a * t**2 / 2, np.where(t < T - ta, a * ta * (t - ta / 2), 1 - a * (T - t) ** 2 / 2))
    return t.tolist(), (q0 + np.outer(s, d)).tolist()
