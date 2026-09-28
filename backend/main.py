from typing import Annotated, Literal

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse
from pydantic import BaseModel, Field

from backend.calib import plan_calib
from backend.planner import arm, plan_lin, plan_ptp, tcp_pose
from backend.rmp import plan_rmp, plan_rmp_path

app = FastAPI(title="robot-motion API")


# 生成できない条件（直線経路上で逆運動学が解けないなど）は、理由を 422 で返して画面に出せるようにする
@app.exception_handler(ValueError)
def _(_request: Request, e: ValueError):
    return JSONResponse({"detail": str(e)}, status_code=422)

# 6軸アーム専用のため、関節ごとの値は常に6個
Joints = Annotated[list[float], Field(min_length=6, max_length=6)]

# frontend の初期値に使う既定の設定（dt はロボットログと同じ 10ms）。lin_* / rot_* は LIN・RMP の先端の速度・加速度
# joint_min / joint_max は測定動作の可動範囲 [deg]（URDF の limit）
DEFAULTS = {"max_vel": [180.0] * 6, "max_acc": [720.0] * 6, "lin_vel": 250.0, "lin_acc": 1000.0, "rot_vel": 90.0, "rot_acc": 360.0, "dt": 0.01,
            "joint_min": [round(lo, 1) for lo, _ in arm.limits], "joint_max": [round(hi, 1) for _, hi in arm.limits]}


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
    # rmp・rmp_path で、robot-viser-app に登録した障害物を避ける RMP を合成する。avoid_distance は回避を効かせ始める距離 [mm]
    avoid: bool = False
    avoid_distance: float = Field(100.0, gt=0)


# 生成条件から軌道（時刻 [s] と6関節角度 [deg] の列）と、障害物との最小距離 [mm]（障害物回避したときのみ。それ以外は None）を作る
def generate(req: TrajectoryRequest) -> tuple[list[float], list[list[float]], float | None]:
    avoid = req.avoid_distance if req.avoid else None
    if req.mode == "lin":
        return *plan_lin(req.start, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.dt, req.duration), None
    if req.mode == "rmp":
        return plan_rmp(req.start, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.dt, avoid)
    if req.mode == "rmp_path":
        return plan_rmp_path(req.start, req.via, req.goal, req.max_vel, req.max_acc, req.lin_vel, req.lin_acc, req.rot_vel, req.rot_acc, req.blend, req.dt, avoid)
    return *plan_ptp(req.start, req.goal, req.max_vel, req.max_acc, req.dt, req.duration), None


# レーザートラッカーでのキャリブレーション用の測定動作の生成条件。長さは mm（base_link 基準）、角度は deg
# target_offset / target_dir はターゲット（SMR）の tool0 座標での位置と、ミラーが向く方向。cone はミラーの向きとトラッカー方向のなす角の許容値
# area_min / area_max を両方指定すると、ターゲット位置をその箱の中に限る（なければ動作領域全体）。margin は障害物との安全距離
Vec3 = Annotated[list[float], Field(min_length=3, max_length=3)]


class CalibRequest(BaseModel):
    num_points: int = Field(100, ge=1)
    tracker: Vec3 = [0.0, 2500.0, 800.0]
    target_offset: Vec3 = [0.0, 0.0, 0.0]
    target_dir: Vec3 = [0.0, 0.0, 1.0]
    cone: float = Field(30.0, gt=0, le=90)
    area_min: Vec3 | None = None
    area_max: Vec3 | None = None
    joint_min: Joints = DEFAULTS["joint_min"]
    joint_max: Joints = DEFAULTS["joint_max"]
    margin: float = Field(10.0, ge=0)
    max_vel: Joints = DEFAULTS["max_vel"]
    max_acc: Joints = DEFAULTS["max_acc"]
    dt: float = Field(DEFAULTS["dt"], gt=0)
    seed: int = 0


# 測定動作を生成し、全ポイント（label: start / measure / via / end）の関節角度の CSV（no,label,joint1..joint6）と、それを PTP でつないだ再生用の軌道 CSV（t,joint1..joint6）を返す
# points には各ポイントのターゲット位置 [mm] も入れる（画面での表示用）
@app.post("/calib")
def post_calib(req: CalibRequest):
    area = None if req.area_min is None or req.area_max is None else [req.area_min, req.area_max]
    r = plan_calib(req.num_points, req.tracker, req.target_offset, req.target_dir, req.cone, area, req.joint_min, req.joint_max, req.margin, req.max_vel, req.max_acc, req.dt, req.seed)
    points_csv = "\n".join(["no,label," + ",".join(f"joint{i}" for i in range(1, 7)), *(f"{i},{p['label']}," + ",".join(f"{a:.6f}" for a in p["angles"]) for i, p in enumerate(r["points"]))]) + "\n"
    trajectory_csv = "\n".join(["t," + ",".join(f"joint{i}" for i in range(1, 7)), *(f"{t:.4f}," + ",".join(f"{a:.6f}" for a in q) for t, q in zip(r["times"], r["angles"]))]) + "\n"
    return {"points": r["points"], "points_csv": points_csv, "trajectory_csv": trajectory_csv, "duration_sec": r["times"][-1],
            "num_measure": sum(p["label"] == "measure" for p in r["points"]), "num_via": sum(p["label"] == "via" for p in r["points"])}


# 選べる動作モードと既定の設定を返す（calib は /calib で作る測定動作）
@app.get("/modes")
def get_modes():
    return {"modes": ["ptp", "lin", "rmp", "rmp_path", "calib"], "defaults": DEFAULTS}


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
    times, angles, min_distance = generate(req)
    return {"times": times, "angles": angles, "duration_sec": times[-1], "num_points": len(times), "min_distance": min_distance}


# 軌道を生成し、robot-viser-app がそのまま読める t 形式の CSV（t[sec], joint1..joint6[deg]）で返す。障害物との最小距離 [mm] はヘッダ X-Min-Distance で返す
@app.post("/trajectory/csv", response_class=PlainTextResponse)
def post_trajectory_csv(req: TrajectoryRequest):
    times, angles, min_distance = generate(req)
    rows = [f"{t:.4f}," + ",".join(f"{a:.6f}" for a in q) for t, q in zip(times, angles)]
    csv = "\n".join(["t," + ",".join(f"joint{i}" for i in range(1, 7)), *rows]) + "\n"
    return PlainTextResponse(csv, headers={} if min_distance is None else {"X-Min-Distance": f"{min_distance:.1f}"})
