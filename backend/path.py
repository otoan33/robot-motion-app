"""経由点の折れ線（角を丸める）による先端の経路。弧長に沿った点列として、接線・曲率・姿勢・速度の上限を持つ。長さは m、角度は rad。"""
import numpy as np

from backend.kinematics import rotation, rotvec


# 回転ベクトル → 回転軸（単位ベクトル）と角度
def unit(w: np.ndarray) -> tuple[np.ndarray, float]:
    theta = np.linalg.norm(w)
    return w / (theta or 1), theta


# p0 → p1 の直線を step 以下の間隔の点列にする（p0 は含めない。長さ 0 なら空）
def line(p0: np.ndarray, p1: np.ndarray, step: float) -> np.ndarray:
    n = int(np.ceil(np.linalg.norm(p1 - p0) / step))
    return p0 + (p1 - p0) * np.linspace(0, 1, n + 1)[1:, None]


class Path:
    # poses: 開始・経由点・目標の先端姿勢（4x4）。blend: 角を丸める距離 [m]。v, a, w_max: 先端の最大速度 [m/s]・最大加速度 [m/s²]・姿勢の最大角速度 [rad/s]
    def __init__(self, poses: list[np.ndarray], blend: float, v: float, a: float, w_max: float, step: float = 1e-3):
        P, self.R = [T[:3, 3] for T in poses], [T[:3, :3] for T in poses]
        L = [np.linalg.norm(P[i + 1] - P[i]) for i in range(len(P) - 1)]
        # 先端位置が重なる点は、弧長に沿って姿勢を割り振れないため受け付けない
        if min(L) < 1e-4: raise ValueError("先端位置が重なる点（開始・経由点・目標）があります。経路追従では隣り合う点の先端位置を 0.1 mm 以上離してください")
        u = [(P[i + 1] - P[i]) / L[i] for i in range(len(L))]

        # 折れ線を細かい点列にする。角は、角から d（隣の辺の半分まで）手前〜先を 2 次ベジェ曲線で丸める（d = 0 なら尖った角で止まる）
        # corners は各点（開始・経由点・目標）の姿勢を割り振る点列の番号（丸めた角はベジェの中央）、spans は丸めた角の始め・終わりの番号
        pts, corners, spans, cur = [P[0][None]], [0], [], P[0]
        for i in range(1, len(P) - 1):
            d = min(blend, L[i - 1] / 2, L[i] / 2)
            a_in, a_out = P[i] - d * u[i - 1], P[i] + d * u[i]
            pts.append(line(cur, a_in, step))
            s = np.linspace(0, 1, max(2, int(np.ceil(2 * d / step))) + 1)[1:, None] if d > 0 else np.empty((0, 1))
            n0 = sum(map(len, pts)) - 1
            corners.append(n0 + len(s) // 2)
            if d > 0: spans.append((n0, n0 + len(s)))
            pts.append((1 - s) ** 2 * a_in + 2 * s * (1 - s) * P[i] + s**2 * a_out)
            cur = a_out
        pts.append(line(cur, P[-1], step))
        self.points = np.concatenate(pts)
        corners.append(len(self.points) - 1)

        # 弧長・単位接線・曲率ベクトル（接線の弧長微分）
        self.S = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))])
        t = np.gradient(self.points, self.S, axis=0)
        self.tangent = t / np.linalg.norm(t, axis=1, keepdims=True)
        self.kappa = np.gradient(self.tangent, self.S, axis=0)

        # 姿勢: 点（開始・経由点・目標）の間は、弧長に比例して一定の回転軸まわりに回す。その回し方を区間の外へ延長したものを ext(i, s) とする
        # 丸めた角の範囲では前後の区間の ext を smoothstep で混ぜ、角で角速度が急に変わらないようにする（点ちょうどでは両方とも点の姿勢に一致する）
        sigma = self.S[corners]
        w = [rotvec(self.R[i + 1] @ self.R[i].T) for i in range(len(self.R) - 1)]
        ext = lambda i, s: rotation(unit(w[i])[0], (s - sigma[i]) / (sigma[i + 1] - sigma[i]) * unit(w[i])[1]) @ self.R[i]
        seg = np.clip(np.searchsorted(sigma, self.S, side="right") - 1, 0, len(w) - 1)
        self.orientation = np.array([ext(i, s) for i, s in zip(seg, self.S)])
        for j, (b0, b1) in enumerate(spans, 1):
            for k in range(b0, b1 + 1):
                # 角の中の位置 x に応じた割合（smoothstep）だけ、前の区間の姿勢 Ra から次の区間の姿勢へ回す
                x = (self.S[k] - self.S[b0]) / (self.S[b1] - self.S[b0])
                Ra = ext(j - 1, self.S[k])
                axis, theta = unit(rotvec(ext(j, self.S[k]) @ Ra.T))
                self.orientation[k] = rotation(axis, (3 * x**2 - 2 * x**3) * theta) @ Ra
        # 弧長あたりの回転（角速度 = これ × 進む速さ）とその弧長微分（角加速度の先回り用）
        dw = np.array([rotvec(self.orientation[k + 1] @ self.orientation[k].T) for k in range(len(self.S) - 1)]) / np.diff(self.S)[:, None]
        self.rot_rate = np.concatenate([dw[:1], (dw[:-1] + dw[1:]) / 2, dw[-1:]])
        self.drot_rate = np.gradient(self.rot_rate, self.S, axis=0)
        rate = np.linalg.norm(self.rot_rate, axis=1)

        # 速度の上限: 最大速度・曲がるときの向心加速度・姿勢の角速度。さらに最大加速度で加減速できるよう、前後から絞る
        # （終点は 0。始点は動き出せるよう 5 mm 分だけ速度を持たせる）
        # 最後の 5 mm は残り距離に比例（v = ω_end·r）させ、終点の引き寄せ（同じ ω_end の臨界減衰）へ勢いを残さずに引き継ぐ
        self.omega_end = np.sqrt(2 * a / 0.005)
        speed = np.minimum.reduce([np.full(len(self.S), v), np.sqrt(a / (np.linalg.norm(self.kappa, axis=1) + 1e-12)), w_max / (rate + 1e-12),
                                   self.omega_end * (self.S[-1] - self.S)])
        speed[0], speed[-1] = min(speed[0], np.sqrt(2 * a * 0.005)), 0
        dS = np.diff(self.S)
        for i in range(1, len(speed)): speed[i] = min(speed[i], np.sqrt(speed[i - 1] ** 2 + 2 * a * dS[i - 1]))
        for i in range(len(speed) - 2, -1, -1): speed[i] = min(speed[i], np.sqrt(speed[i + 1] ** 2 + 2 * a * dS[i]))
        # 速度の弧長微分（加減速の先回り用）
        self.speed, self.dspeed = speed, np.gradient(speed, self.S)

    # 先端位置 x に最も近い点の番号。後戻りしないよう、前回の番号 k から前方 50 mm の範囲で探す
    def nearest(self, x: np.ndarray, k: int) -> int:
        j = np.searchsorted(self.S, self.S[k] + 0.05) + 1
        return k + int(np.argmin(np.linalg.norm(self.points[k:j] - x, axis=1)))
