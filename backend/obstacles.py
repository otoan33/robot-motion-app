"""robot-viser-app の衝突判定 API（/distances）で、ロボットの近似球と障害物の距離を求める。障害物は robot-viser-app に登録されたものを使う。"""
import os

import httpx
import numpy as np

# robot-viser-app の API。compose では VISER_API_URL=http://host.docker.internal:8000 を渡す
client = httpx.Client(base_url=os.environ.get("VISER_API_URL", "http://127.0.0.1:8000"))


# 関節角度 [deg] での /distances の結果（距離が r [m] 以内の組だけ）
def query(angles_deg, r: float) -> dict:
    res = client.post("/distances", json={"angles": list(angles_deg), "max_distance": r})
    # 衝突判定を有効にせずに起動した robot-viser-app には /distances がない
    if res.status_code == 404: raise ValueError("robot-viser-app の衝突判定が無効です（COLLISION=1 docker compose up -d で起動してください）")
    res.raise_for_status()
    return res.json()


# 関節角度 [deg] で、距離が r [m] 以内の（近似球, 障害物）の組ごとの距離 [m] と距離のヤコビアン（障害物から離れる向き × 球中心のヤコビアン, 6 要素 [m/rad]）、
# および全ての組の中での最小距離（障害物がなければ None）
def distances(angles_deg, r: float) -> tuple[list[tuple[float, np.ndarray]], float | None]:
    d = query(angles_deg, r)
    J = [np.array(p["jacobian"]) for p in d["control_points"]]
    return [(p["distance"], np.array(p["normal"]) @ J[p["point"]]) for p in d["pairs"]], d["min_distance"]


# 関節角度 [deg] での近似球（link 名・中心 [m]・半径 [m]・中心のヤコビアン）。測定動作の生成では 1 回だけ取り、あとは手元で動かす
def control_points(angles_deg) -> list[dict]:
    return query(angles_deg, 0)["control_points"]


# robot-viser-app に登録中の障害物（/obstacles と同じ形式、座標は base_link 基準 [m]）
def registered() -> list[dict]:
    return client.get("/obstacles").json()["obstacles"]
