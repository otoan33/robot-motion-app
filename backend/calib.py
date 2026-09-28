"""レーザートラッカーでのキャリブレーション用の測定動作を作る。ターゲットがトラッカーを向き、干渉も光路の遮蔽もない測定点を広く選び、
全軸 0° から PTP でつないで 0° に戻る順を、合計動作時間が短くなるように決める（干渉する区間には経由点を入れる）。
内部の長さは m・角度は rad（関節角度は deg）、入出力は mm・deg。"""
import heapq

import numpy as np

from backend import obstacles
from backend.kinematics import rpy_matrix
from backend.planner import arm, plan_ptp


# 点 P (N,3) と障害物の表面との符号付き距離 (N,)（中に入ると負）。形状の定義は robot-viser-app の /obstacles と同じ
def obstacle_distance(ob: dict, P: np.ndarray) -> np.ndarray:
    if ob["type"] == "box":
        q = np.abs((P - ob["center"]) @ rpy_matrix(ob["rpy"])) - np.asarray(ob["size"]) / 2
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    # 球は中心、カプセルは線分上の最近点からの距離
    if ob["type"] == "sphere": C = np.asarray(ob["center"], float)
    else:
        a, b = np.asarray(ob["p1"], float), np.asarray(ob["p2"], float)
        C = a + np.clip((P - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1)[:, None] * (b - a)
    return np.linalg.norm(P - C, axis=1) - ob["radius"]


# 辺の集合 adj・辺のコスト C のグラフで、s から g への最短経路（節点の列。つながらなければ None）。ダイクストラ法
def shortest(adj: list[set], C: np.ndarray, s: int, g: int) -> list[int] | None:
    dist, prev, heap = {s: 0.0}, {}, [(0.0, s)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == g: break
        if d > dist[u]: continue
        for v in adj[u]:
            if d + C[u, v] < dist.get(v, np.inf):
                dist[v], prev[v] = d + C[u, v], u
                heapq.heappush(heap, (dist[v], v))
    if g not in dist: return None
    path = [g]
    while path[-1] != s: path.append(prev[path[-1]])
    return path[::-1]


# 測定点の候補の位置 X (M,3) から、互いに最も離れるよう n 点を選ぶ（最遠点サンプリング。重心から最も遠い点から始める）。ok(i) が偽の候補は飛ばす
def farthest(X: np.ndarray, n: int, ok) -> list[int]:
    d, idx = np.linalg.norm(X - X.mean(0), axis=1) + 1e6, []
    while len(idx) < n and np.isfinite(d.max(initial=-np.inf)):
        i = int(d.argmax())
        if ok(i):
            idx.append(i)
            d = np.minimum(d, np.linalg.norm(X - X[i], axis=1))
        d[i] = -np.inf
    return idx


# 節点 0 から出て全節点を回り 0 に戻る順（コスト行列 C）。最近傍法で作り、2-opt（区間の反転）で短くなる限り直す
def tour(C: list[list[float]]) -> list[int]:
    n, t = len(C), [0]
    left = set(range(1, n))
    while left:
        t.append(min(left, key=lambda j: C[t[-1]][j]))
        left.remove(t[-1])
    t.append(0)
    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                if C[t[i - 1]][t[j]] + C[t[i]][t[j + 1]] < C[t[i - 1]][t[i]] + C[t[j]][t[j + 1]] - 1e-9:
                    t[i:j + 1], improved = t[i:j + 1][::-1], True
    return t


class Calib:
    """測定条件（トラッカー・ターゲット・エリア・可動範囲・安全距離・関節の速度と加速度）と、干渉を調べるロボットの近似球・障害物を持つ。"""

    def __init__(self, tracker, target_offset, target_dir, cone, area, joint_min, joint_max, margin, max_vel, max_acc, seed):
        self.tracker, self.offset, self.dir = np.array(tracker) / 1000, np.array(target_offset) / 1000, np.array(target_dir, float) / np.linalg.norm(target_dir)
        self.cone, self.margin, self.area = np.radians(cone), margin / 1000, None if area is None else np.array(area) / 1000
        self.lo, self.hi, self.max_vel, self.max_acc = np.array(joint_min, float), np.array(joint_max, float), np.array(max_vel, float), np.array(max_acc, float)
        self.rng = np.random.default_rng(seed)
        # 近似球は robot-viser-app から全軸 0° の中心を 1 回だけ取り、中心が動く最後の関節（ヤコビアンの列が 0 でない最後の関節）の先のリンク座標に直して手元で動かす（/distances を毎回呼ぶと遅いため）
        cps, frames = obstacles.control_points([0] * 6), arm.forward_many(np.zeros((1, 6)))[1][0]
        self.parent = [max([i + 1 for i in range(6) if np.abs(np.array(p["jacobian"])[:, i]).max() > 1e-9], default=0) for p in cps]
        self.local = np.array([(np.linalg.inv(frames[k]) @ [*p["position"], 1])[:3] for k, p in zip(self.parent, cps)])
        self.radii = np.array([p["radius"] for p in cps])
        # 自己干渉を見る球の組: リンクの並び（根元→先端→ハンド）で 2 つ以上離れ、全軸 0° で重なっていないもの（隣のリンクや、近似で元から重なる組は除く）
        rank = {l: i for i, l in enumerate(dict.fromkeys(p["link"] for p in cps))}
        rank, P0 = np.array([rank[p["link"]] for p in cps]), self.poses(np.zeros((1, 6)))[0][0]
        I, J = np.triu_indices(len(cps), 1)
        keep = (np.abs(rank[I] - rank[J]) >= 2) & (np.linalg.norm(P0[I] - P0[J], axis=1) > self.radii[I] + self.radii[J])
        self.pairs, self.obstacles = (I[keep], J[keep]), obstacles.registered()
        # 根元に固定された球は動かして避けられないので、障害物（床など）との干渉は見ない（自己干渉と光路の遮蔽には使う）
        self.moving = np.array(self.parent) > 0
        # 関節 i の回転で球の中心が動く量の上限 = 腕の長さ reach[i] × 回転角。腕の長さは、関節 i から球の親の関節までの関節間の距離の合計 + 球のリンク座標でのずれ、の最大
        lens, off = np.cumsum([0] + [np.linalg.norm(F[:3, 3]) for F, axis in arm.joints if axis is not None]), np.linalg.norm(self.local, axis=1)
        self.reach = np.array([max([lens[k] - lens[i] + o for k, o in zip(self.parent, off) if k >= i], default=0) for i in range(1, 7)])

    # 関節角度 Q (K,6) [deg] での近似球の中心 (K,N,3)、ターゲットの位置 (K,3)、ミラーの向き (K,3)。多くの姿勢をまとめて計算する
    def poses(self, Q):
        T, frames = arm.forward_many(np.radians(Q))
        F = frames[:, self.parent]
        return np.einsum("knij,nj->kni", F[..., :3, :3], self.local) + F[..., :3, 3], T[:, :3, :3] @ self.offset + T[:, :3, 3], T[:, :3, :3] @ self.dir

    # 近似球の中心 P (K,N,3) の姿勢ごとに、障害物との距離から安全距離を引いたものと、自己干渉を見る球どうしの距離のうち小さい方 (K,) [m]（負なら干渉）
    def clearance(self, P) -> np.ndarray:
        I, J = self.pairs
        c = (np.linalg.norm(P[:, I] - P[:, J], axis=2) - self.radii[I] - self.radii[J]).min(1, initial=np.inf)
        Pm, rm = P[:, self.moving], self.radii[self.moving]
        for ob in self.obstacles: c = np.minimum(c, (obstacle_distance(ob, Pm.reshape(-1, 3)).reshape(len(P), -1) - rm).min(1) - self.margin)
        return c

    # トラッカーからターゲット p への光路（10 mm 間隔の点）が、障害物か近似球 P (N,3)（ターゲットを内側に含む球は除く）に遮られるか
    def blocked(self, P, p) -> bool:
        B = p + np.linspace(0, 1, max(int(np.linalg.norm(self.tracker - p) / 0.01), 2))[1:, None] * (self.tracker - p)
        other = np.linalg.norm(P - p, axis=1) >= self.radii
        return bool((np.linalg.norm(B[:, None] - P[other], axis=2) < self.radii[other]).any()) or any((obstacle_distance(ob, B) < 0).any() for ob in self.obstacles)

    # 関節角度 [deg] を測定姿勢に使えるか（可動範囲・ミラーの向きが許容角以内・干渉なし・光路が遮られない）
    def valid(self, q) -> bool:
        if np.any(q < self.lo) or np.any(q > self.hi): return False
        (P,), (p,), (n,) = self.poses(q[None])
        d = self.tracker - p
        return n @ d >= np.cos(self.cone) * np.linalg.norm(d) and self.clearance(P[None])[0] >= 0 and not self.blocked(P, p)

    # PTP の最短時間 [s]（planner.trapezoid と同じ台形速度）。a, b は (..., 6) [deg] で、組ごとに求める
    def cost(self, a, b):
        d = np.abs(np.asarray(b) - np.asarray(a))
        kv, ka = (d / self.max_vel).max(-1), (d / self.max_acc).max(-1)
        return np.where(ka < kv**2, kv + ka / np.maximum(kv, 1e-12), 2 * np.sqrt(ka))

    # ターゲットの位置を P に保ったまま、ミラーの向きとトラッカー方向のなす角を許容角以内に入れる関節角度 [deg] を、初期値 Q0 [deg] から減衰最小二乗法で探す（K 組まとめて解く）
    # ミラーの軸まわり（U 回転）は拘束せず、向きは許容角を超えたぶんだけ回すので、初期値に近い解になる。±360° は可動範囲内で初期値に近い方を選ぶ。解けない組は NaN
    def aim(self, Q0, P) -> np.ndarray:
        q = np.radians(Q0)
        for _ in range(50):
            T, J = arm.jacobian_many(q)
            r, n = T[:, :3, :3] @ self.offset, T[:, :3, :3] @ self.dir
            d = self.tracker - T[:, :3, 3] - r
            d /= np.linalg.norm(d, axis=1, keepdims=True)
            w = np.cross(n, d)
            wn = np.linalg.norm(w, axis=1, keepdims=True)
            e = np.concatenate([P - T[:, :3, 3] - r, w / np.maximum(wn, 1e-12) * np.maximum(np.arctan2(wn, (n * d).sum(1, keepdims=True)) - self.cone * 0.99, 0)], 1)
            en = np.linalg.norm(e, axis=1, keepdims=True)
            if np.nanmax(en) < 1e-9: break
            # ターゲット点の速度 v + ω×r と、ミラーの軸に垂直な角速度。遠いときは 1 回に動かす量を抑えて発散を防ぐ
            A = np.concatenate([J[:, :3] + np.cross(J[:, 3:], r[:, :, None], axis=1), (np.eye(3) - n[:, :, None] * n[:, None]) @ J[:, 3:]], 1)
            At = A.transpose(0, 2, 1)
            q = q + (At @ np.linalg.solve(A @ At + 1e-6 * np.eye(6), (e * np.minimum(1, 0.3 / np.maximum(en, 1e-12)))[..., None]))[..., 0]
        opts = ((np.degrees(q) - Q0 + 180) % 360 - 180 + Q0)[..., None] + [-360, 0, 360]
        gap = np.where((opts >= self.lo[:, None]) & (opts <= self.hi[:, None]), np.abs(opts - Q0[..., None]), np.inf)
        ok = (en[:, 0] < 1e-6) & ~np.isinf(gap.min(2)).any(1)
        return np.where(ok[:, None], np.take_along_axis(opts, gap.argmin(2)[..., None], 2)[..., 0], np.nan)

    # 測定点の候補（関節角度 [deg], ターゲット位置）。可動範囲内のランダムな関節角度でのターゲット位置（エリア指定時はその中だけ）のまま、ミラーをトラッカーへ向ける
    # 途中で見つかった干渉しない姿勢は、経由点の経路網に使うため self.free_poses に残す
    def candidates(self, count: int, tries: int) -> list:
        found, self.free_poses = [], []
        for _ in range(0, tries, 500):
            # 500 姿勢ずつまとめて、エリアの外と干渉している姿勢を捨てる（向きを直す逆運動学は重いので先に減らす）
            Q = self.rng.uniform(self.lo, self.hi, (500, 6))
            P, X, _ = self.poses(Q)
            ok = self.clearance(P) >= 0
            self.free_poses += list(Q[ok])
            if self.area is not None: ok &= np.all((X >= self.area[0]) & (X <= self.area[1]), axis=1)
            if not ok.any(): continue
            for q, p in zip(self.aim(Q[ok], X[ok]), X[ok]):
                if not np.isnan(q[0]) and self.valid(q): found.append((q, p))
                if len(found) == count: return found
        return found

    # PTP の直線 a→b [deg] が干渉しないか。球どうし・球と障害物の距離が s（0→1）あたりに縮みうる最大量 D（2 × Σ 関節ごとの腕の長さ × 移動量）を使い、
    # 隣り合う点の余裕の和が D × 間隔 より小さい間だけ二分して調べ直す（それ以外の間は干渉しないことが保証される）
    def free(self, a, b) -> bool:
        D = 2 * self.reach @ np.abs(np.radians(b - a))
        s = np.linspace(0, 1, int(D / 0.05) + 2)
        c = self.clearance(self.poses(a + s[:, None] * (b - a))[0])
        while (c >= 0).all():
            bad = (c[:-1] + c[1:] < D * np.diff(s)) & (np.diff(s) > 1e-4)
            if not bad.any(): return True
            mid = (s[:-1] + s[1:])[bad] / 2
            s, c = np.concatenate([s, mid]), np.concatenate([c, self.clearance(self.poses(a + mid[:, None] * (b - a))[0])])
            s, c = s[np.argsort(s)], c[np.argsort(s)]
        return False

    # 経由点に使う経路網。干渉しない姿勢を節点にし、PTP の動作時間が近い k 個どうしを辺で結ぶ（辺の干渉は使うときに調べて覚える）
    def roadmap(self, nodes, k: int = 15):
        self.nodes, self.k, self.checked = np.asarray(nodes), k, {}
        self.C = np.array([self.cost(v, self.nodes) for v in self.nodes])
        self.adj = [set() for _ in nodes]
        for i, row in enumerate(self.C):
            for j in np.argsort(row)[1:k + 1]: self.adj[i].add(int(j)); self.adj[j].add(i)

    # a→b [deg] を干渉せずにつなぐ経由点の列（直接つながれば空）。a・b を経路網の近い節点へつなぎ、動作時間の合計が最短の経路の辺を順に調べ、
    # 干渉する辺は外して探し直す（lazy PRM）。100 回探して見つからなければ全軸 0° を経由する（測定点は全軸 0° から直接行けるものだけなので必ずつながる）
    def connect(self, a, b):
        if self.free(a, b): return []
        M, V = len(self.nodes), np.vstack([self.nodes, a, b])
        C, adj = np.zeros((M + 2, M + 2)), [set(s) for s in self.adj] + [set(), set()]
        C[:M, :M] = self.C
        for i in (M, M + 1):
            C[i], C[:, i] = self.cost(V[i], V), self.cost(V[i], V)
            for j in np.argsort(C[i, :M])[:self.k]: adj[i].add(int(j)); adj[j].add(i)
        for _ in range(100):
            if (path := shortest(adj, C, M, M + 1)) is None: break
            for u, v in zip(path[:-1], path[1:]):
                # 経路網の節点どうしの辺は結果を覚えておき、他の区間でも使い回す
                key = (min(u, v), max(u, v))
                ok = self.checked.get(key) if key[1] < M else None
                if ok is None: ok = self.free(V[u], V[v])
                if key[1] < M: self.checked[key] = ok
                if not ok:
                    adj[u].discard(v); adj[v].discard(u)
                    break
            else: return [V[i] for i in path[1:-1]]
        return [np.zeros(6)]


# 測定動作を作る。長さ [mm]・角度 [deg]。area は [[xmin, ymin, zmin], [xmax, ymax, zmax]]（None なら動作領域全体）
# 返り値: 全ポイント（label: start / measure / via / end、関節角度、ターゲット位置 [mm]）と、それを PTP でつないだ軌道（時刻・関節角度）
def plan_calib(num_points, tracker, target_offset, target_dir, cone, area, joint_min, joint_max, margin, max_vel, max_acc, dt, seed=0) -> dict:
    c, home = Calib(tracker, target_offset, target_dir, cone, area, joint_min, joint_max, margin, max_vel, max_acc, seed), np.zeros(6)
    if c.clearance(c.poses(home[None])[0])[0] < 0: raise ValueError("全軸 0° の姿勢が障害物と干渉しています")
    # 条件を満たす候補を点数の 5 倍集め、ターゲット位置が空間に広がるよう選ぶ。全軸 0° から PTP で直接行ける点だけにする（どの 2 点の間も、全軸 0° を経由すれば必ず干渉せずにつなげる）
    found = c.candidates(5 * num_points, 300 * num_points)
    idx = farthest(np.array([p for _, p in found]).reshape(-1, 3), num_points, lambda i: c.free(home, found[i][0]))
    if len(idx) < num_points: raise ValueError(f"条件を満たす測定点が {len(idx)} 点しか見つかりません（トラッカーの位置・エリア・許容角・障害物を見直してください）")
    Q, X = np.array([home] + [found[i][0] for i in idx]), np.array([[np.nan] * 3] + [found[i][1] for i in idx])

    # 全軸 0° を始点・終点に、PTP の動作時間の合計が短い順に並べる。並べた順で、前後の点に近い姿勢（U 回転・許容角内の傾きを変えたもの）に解き直して動作量を減らし、並べ直す
    for k in range(3):
        t = tour(c.cost(Q[:, None], Q[None]).tolist())
        if k == 2: break
        # 解き直しの初期値は前・後・その中間の点の姿勢（まとめて解き、順にその時点の前後の点との動作時間で比べる）。解き直した姿勢も全軸 0° から直接行けるものに限る
        prev, nxt = Q[t[:-2]], Q[t[2:]]
        A = c.aim(np.concatenate([prev, nxt, (prev + nxt) / 2]), np.tile(X[t[1:-1]], (3, 1))).reshape(3, len(t) - 2, 6)
        for m in range(1, len(t) - 1):
            a, b, i = Q[t[m - 1]], Q[t[m + 1]], t[m]
            best = c.cost(a, Q[i]) + c.cost(Q[i], b)
            for q in A[:, m - 1]:
                if not np.isnan(q[0]) and c.cost(a, q) + c.cost(q, b) < best and c.valid(q) and c.free(home, q): Q[i], best = q, c.cost(a, q) + c.cost(q, b)

    # 並べた順に PTP でつなぎ、干渉する区間には、干渉しない姿勢（候補探しの途中のものと測定点の候補・全軸 0°）の経路網から経由点を入れる
    c.roadmap([home, *c.free_poses[:1000], *(q for q, _ in found)])
    rows = [("start", home, None)]
    for i, j in zip(t[:-1], t[1:]):
        vias = c.connect(Q[i], Q[j])
        rows += [("via", v, None) for v in vias] + [("end" if j == 0 else "measure", Q[j], None if j == 0 else X[j])]

    # 再生・確認用に、全ポイントを PTP（各点で停止）でつないだ軌道にする
    times, angles = [0.0], [home.tolist()]
    for (_, a, _), (_, b, _) in zip(rows[:-1], rows[1:]):
        ts, qs = plan_ptp(a, b, max_vel, max_acc, dt)
        times += [times[-1] + ti for ti in ts[1:]]
        angles += qs[1:]
    points = [{"label": label, "angles": q.tolist(), "position": ((c.poses(q[None])[1][0] if p is None else p) * 1000).tolist()} for label, q, p in rows]
    return {"points": points, "times": times, "angles": angles}
