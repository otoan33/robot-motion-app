from typing import Annotated, Literal

from fastapi import FastAPI
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from backend.planner import plan_ptp

app = FastAPI(title="robot-motion API")

# 6軸アーム専用のため、関節ごとの値は常に6個
Joints = Annotated[list[float], Field(min_length=6, max_length=6)]

# frontend の初期値に使う既定の設定（dt はロボットログと同じ 10ms）
DEFAULTS = {"max_vel": [180.0] * 6, "max_acc": [720.0] * 6, "dt": 0.01}


# 軌道の生成条件。mode で動作の種類を切り替える（今は PTP のみ）
class TrajectoryRequest(BaseModel):
    mode: Literal["ptp"] = "ptp"
    start: Joints
    goal: Joints
    max_vel: Joints = DEFAULTS["max_vel"]
    max_acc: Joints = DEFAULTS["max_acc"]
    dt: float = Field(DEFAULTS["dt"], gt=0)
    duration: float | None = None


# 生成条件から軌道（時刻 [s] と6関節角度 [deg] の列）を作る
def generate(req: TrajectoryRequest) -> tuple[list[float], list[list[float]]]:
    return plan_ptp(req.start, req.goal, req.max_vel, req.max_acc, req.dt, req.duration)


# 選べる動作モードと既定の設定を返す
@app.get("/modes")
def get_modes():
    return {"modes": ["ptp"], "defaults": DEFAULTS}


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
