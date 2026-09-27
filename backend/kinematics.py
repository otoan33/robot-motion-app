"""URDF の関節定義から、6軸アームの順運動学・逆運動学を求める。座標は base_link（= viser のワールド）基準、長さは m、角度は rad。"""
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# robot-viser-app で表示しているアーム（robotA）の URDF。形状は使わず、関節の位置・回転軸だけを読む
ARM_URDF = Path(__file__).parent / "assets" / "arms" / "robotA" / "arm.urdf"


# 回転軸 axis まわりに q[rad] 回す回転行列（ロドリゲスの公式）
def rotation(axis: np.ndarray, q: float) -> np.ndarray:
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


# 回転行列 → 回転ベクトル（回転軸 × 角度）。姿勢の補間と逆運動学の誤差に使う
def rotvec(R: np.ndarray) -> np.ndarray:
    theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w / 2 if theta < 1e-9 else w * theta / (2 * np.sin(theta))


# roll / pitch / yaw [rad] ⇔ 回転行列（URDF と同じ固定軸 X→Y→Z の順）
def rpy_matrix(rpy) -> np.ndarray:
    return rotation(np.array([0, 0, 1.0]), rpy[2]) @ rotation(np.array([0, 1.0, 0]), rpy[1]) @ rotation(np.array([1.0, 0, 0]), rpy[0])


def matrix_rpy(R: np.ndarray) -> np.ndarray:
    return np.array([np.arctan2(R[2, 1], R[2, 2]), np.arcsin(-np.clip(R[2, 0], -1, 1)), np.arctan2(R[1, 0], R[0, 0])])


class Arm:
    # 関節を定義順（根元→先端の一本鎖）に読み、各関節の取り付け位置・姿勢と回転軸（固定関節は None）を持つ
    def __init__(self, path: Path = ARM_URDF):
        self.joints = []
        for j in ET.parse(path).getroot().iter("joint"):
            o = j.find("origin")
            F = np.eye(4)
            F[:3, :3], F[:3, 3] = rpy_matrix([float(v) for v in o.get("rpy", "0 0 0").split()]), [float(v) for v in o.get("xyz", "0 0 0").split()]
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()]) if j.get("type") == "revolute" else None
            self.joints.append((F, axis))

    # 関節角度 q[rad] での先端（tool0）の 4x4 変換と、ヤコビアン用の各可動関節の位置・回転軸（ワールド座標）
    def forward(self, q) -> tuple[np.ndarray, list]:
        T, axes, i = np.eye(4), [], 0
        for F, axis in self.joints:
            T = T @ F
            if axis is not None:
                axes.append((T[:3, 3], T[:3, :3] @ axis))
                T[:3, :3] = T[:3, :3] @ rotation(axis, q[i])
                i += 1
        return T, axes

    # 目標の先端姿勢 target（4x4）になる関節角度を、初期値 q から減衰最小二乗法で探す。誤差（位置 m・回転 rad のノルム）も返す
    def inverse(self, target: np.ndarray, q, iters: int = 50, damping: float = 1e-3) -> tuple[np.ndarray, float]:
        q = np.array(q, float)
        for _ in range(iters):
            T, axes = self.forward(q)
            e = np.concatenate([target[:3, 3] - T[:3, 3], rotvec(target[:3, :3] @ T[:3, :3].T)])
            if np.linalg.norm(e) < 1e-10: break
            # 幾何ヤコビアン（各列: 回転軸 × 関節から先端へのベクトル、回転軸）
            J = np.array([np.concatenate([np.cross(a, T[:3, 3] - p), a]) for p, a in axes]).T
            q += J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
        return q, float(np.linalg.norm(e))
