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
    # 関節を定義順（根元→先端の一本鎖）に読み、各関節の取り付け位置・姿勢と回転軸（固定関節は None）を持つ。可動範囲 limits は [下限, 上限] [deg] の組
    def __init__(self, path: Path = ARM_URDF):
        self.joints, self.limits = [], []
        for j in ET.parse(path).getroot().iter("joint"):
            o = j.find("origin")
            F = np.eye(4)
            F[:3, :3], F[:3, 3] = rpy_matrix([float(v) for v in o.get("rpy", "0 0 0").split()]), [float(v) for v in o.get("xyz", "0 0 0").split()]
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()]) if j.get("type") == "revolute" else None
            self.joints.append((F, axis))
            if axis is not None: self.limits.append([float(np.degrees(float(j.find("limit").get(k)))) for k in ("lower", "upper")])

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

    # forward を多くの関節角度 q (K,6)[rad] でまとめて計算する（干渉チェックで多くの姿勢を一度に調べる用）。先端の変換 (K,4,4) と、根元と各可動関節より先のリンクの座標系 (K,7,4,4)（frames[:, 0] が根元、frames[:, i] が関節 i の先）
    def forward_many(self, q) -> tuple[np.ndarray, np.ndarray]:
        T, i = np.broadcast_to(np.eye(4), (len(q), 4, 4)), 0
        frames = [T]
        for F, axis in self.joints:
            T = T @ F
            if axis is not None:
                # ロドリゲスの公式を姿勢ごとにまとめて計算する
                K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
                T[:, :3, :3] = T[:, :3, :3] @ (np.eye(3) + np.sin(q[:, i])[:, None, None] * K + (1 - np.cos(q[:, i]))[:, None, None] * K @ K)
                frames.append(T)
                i += 1
        return T, np.stack(frames, 1)

    # jacobian を多くの関節角度 q (K,6)[rad] でまとめて計算する。先端の変換 (K,4,4) と幾何ヤコビアン (K,6,6)
    def jacobian_many(self, q) -> tuple[np.ndarray, np.ndarray]:
        T, frames = self.forward_many(q)
        # 各関節の回転軸・位置（ワールド座標）を列に並べる (K,3,6)
        A = np.stack([frames[:, i + 1, :3, :3] @ axis for i, axis in enumerate(a for _, a in self.joints if a is not None)], 2)
        O = frames[:, 1:, :3, 3].transpose(0, 2, 1)
        return T, np.concatenate([np.cross(A, T[:, :3, 3, None] - O, axis=1), A], 1)

    # 先端の 4x4 変換と幾何ヤコビアン（6x6。上3行が並進速度、下3行が角速度。各列: 回転軸 × 関節から先端へのベクトル、回転軸）
    def jacobian(self, q) -> tuple[np.ndarray, np.ndarray]:
        T, axes = self.forward(q)
        return T, np.array([np.concatenate([np.cross(a, T[:3, 3] - p), a]) for p, a in axes]).T

    # 目標の先端姿勢 target（4x4）になる関節角度を、初期値 q から減衰最小二乗法で探す。誤差（位置 m・回転 rad のノルム）も返す
    def inverse(self, target: np.ndarray, q, iters: int = 50, damping: float = 1e-3) -> tuple[np.ndarray, float]:
        q = np.array(q, float)
        for _ in range(iters):
            T, J = self.jacobian(q)
            e = np.concatenate([target[:3, 3] - T[:3, 3], rotvec(target[:3, :3] @ T[:3, :3].T)])
            if np.linalg.norm(e) < 1e-10: break
            q += J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), e)
        return q, float(np.linalg.norm(e))
