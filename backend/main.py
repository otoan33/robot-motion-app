from typing import Annotated, Literal

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse
from pydantic import BaseModel, Field

from backend.planner import plan_lin, plan_ptp, tcp_pose
from backend.rmp import plan_rmp, plan_rmp_path

app = FastAPI(title="robot-motion API")


# 生成できない条件（直線経路上で逆運動学が解けないなど）は、理由を 422 で返して画面に出せるようにする
@app.exception_handler(ValueError)
def _(_request: Request, e: ValueError):
    return JSONResponse({"detail": str(e)}, status_code=422)

# 6軸アーム専用のため、関節ごとの値は常に6個
Joints = Annotated[list[float], Field(min_length=6, max_length=6)]

# frontend の初期値に使う既定の設定（dt はロボットログと同じ 10ms）。lin_* / rot_* は LIN・RMP の先端の速度・加速度
DEFAULTS = {"max_vel": [180.0] * 6, "max_acc": [720.0] * 6, "lin_vel": 250.0, "lin_acc": 1000.0, "rot_vel": 90.0, "rot_acc": 360.0, "dt": 0.01}


# 軌道の生成条件。mode で動作の種類を切り替える（ptp: 関節補間、lin: 先端の直線補間、rmp: RMP の目標到達ポリシー、rmp_path: RMP の経路追従）
# duration は ptp・lin だけで使う（rmp・rmp_path の動作時間はポリシーで決まる）。via（経由点の関節角度）と blend（角を丸める距離 [mm]）は rmp_path だけで使う
class TrajectoryRequest(BaseModel):
    mode: Literal["ptp", "lin", "rmp", "rmp_path"] = "ptp"
    start: Joints
    via: list[Joints] = []
    goal: Joints
    max_vel: Joints = DEFAULTS["max_vel"]
    max_acc: Joints = DEFAULTS["max_acc"]
    lin_vel: float = Field(DEFAULTS["lin_vel"], gt=0)
    lin_acc: float = Field(DEFAULTS["lin_acc"], gt=0)
    rot_vel: float = Field(DEFAULTS["rot_vel"], gt=0)
    rot_acc: float = Field(DEFAULTS["rot_acc"], gt=0)
    dt: float = Field(DEFAULTS["dt"], gt=0)
    duration: float | None = None
    blend: float = Field(50.0, ge=0)


# 生成条件から軌道（時刻 [s] と6関節角度 [deg] の列）を作る
def generate(req: TrajectoryRequest) -> tuple[list[float], list[list[float]]]:
    if req.mode == "lin":
        return plan_lin(req.start, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.dt, req.duration)
    if req.mode == "rmp":
        return plan_rmp(req.start, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.dt)
    if req.mode == "rmp_path":
        return plan_rmp_path(req.start, req.via, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.blend, req.dt)
    return plan_ptp(req.start, req.goal, req.max_vel, req.max_acc, req.dt, req.duration)


# 選べる動作モードと既定の設定を返す
@app.get("/modes")
def get_modes():
    return {"modes": ["ptp", "lin", "rmp", "rmp_path"], "defaults": DEFAULTS}


# 関節角度 [deg] での先端（tool0）の位置 [mm] と姿勢 roll / pitch / yaw [deg] を返す
class AnglesRequest(BaseModel):
    angles: Joints


@app.post("/fk")
def post_fk(req: AnglesRequest):
    position, rpy = tcp_pose(req.angles)
    return {"position": position, "rpy": rpy}


# 軌道を生成して JSON で返す
@app.post("/trajectory")
def post_trajectory(req: TrajectoryRequest):
    times, angles = generate(req)
    return {"times": times, "angles": angles, "duration_sec": times[-1], "num_points": len(times)}


# 軌道を生成し、robot-viser-app がそのまま読める t 形式の CSV（t[sec], joint1..joint6[deg]）で返す
@app.post("/trajectory/csv", response_class=PlainTextResponse)
def post_trajectory_csv(req: TrajectoryRequest):
    times, angles = generate(req)
    rows = [f"{t:.4f}," + ",".join(f"{a:.6f}" for a in q) for t, q in zip(times, angles)]
    return "\n".join(["t," + ",".join(f"joint{i}" for i in range(1, 7)), *rows]) + "\n"
