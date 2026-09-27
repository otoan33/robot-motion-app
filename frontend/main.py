import os
from datetime import datetime

import httpx
from fastapi import Request
from nicegui import ui

# 軌道生成 API（本アプリの backend）と、表示・再生を任せる robot-viser-app の API への接続
client = httpx.AsyncClient(base_url=os.environ.get("BACKEND_URL", "http://127.0.0.1:8100"))
viser_api = httpx.AsyncClient(base_url=os.environ.get("VISER_API_URL", "http://127.0.0.1:8000"))

# 動作モードの表示名
MODE_LABELS = {"ptp": "PTP（関節補間・台形速度）", "lin": "LIN（直線補間・台形速度）", "rmp": "PTP（RMP 目標到達ポリシー）"}


@ui.page("/")
async def index(request: Request):
    info = (await client.get("/modes")).json()
    defaults = info["defaults"]
    # 最後に生成した軌道の CSV（再生・保存に使う）
    state = {"csv": None}

    # 6軸分の数値入力を並べる（制限値は桁が少ないので6列で詰める）
    def joint_inputs(values: list[float], columns: int = 3):
        with ui.grid(columns=columns).classes("w-full"):
            return [ui.number(f"J{i}", value=v).classes("w-full") for i, v in enumerate(values, 1)]

    with ui.row().classes("w-full no-wrap"):
        # 左: 設定パネル
        with ui.column().classes("w-96 shrink-0 h-[calc(100vh-2rem)] overflow-y-auto no-wrap"):
            ui.label("robot-motion").classes("text-2xl font-bold")

            # 動作モード（PTP: 関節補間、LIN: 先端の直線補間、RMP: 目標到達ポリシーによる PTP）
            with ui.card().classes("w-full"):
                ui.label("動作モード").classes("text-lg font-bold")
                mode = ui.select({m: MODE_LABELS.get(m, m) for m in info["modes"]}, value=info["modes"][0]).classes("w-full")

            # 開始位置・目標位置 [deg]。「表示」で robot-viser にその姿勢を表示して確認する
            async def show_pose(inputs): (await viser_api.post("/joints", json={"angles": [n.value for n in inputs]})).raise_for_status()
            # 入力中の関節角度での先端位置・姿勢を表示する（LIN の直線の両端の確認用）
            async def update_tcp(inputs, label):
                if any(n.value is None for n in inputs): return
                tcp = (await client.post("/fk", json={"angles": [n.value for n in inputs]})).json()
                label.text = "先端 XYZ [mm] " + ", ".join(f"{v:.1f}" for v in tcp["position"]) + " / RPY [deg] " + ", ".join(f"{v:.1f}" for v in tcp["rpy"])
            def pose_card(title: str, values: list[float]):
                with ui.card().classes("w-full"):
                    ui.label(title).classes("text-lg font-bold")
                    inputs = joint_inputs(values)
                    tcp = ui.label().classes("text-xs text-gray-500")
                    for n in inputs: n.on_value_change(lambda: update_tcp(inputs, tcp))
                    ui.timer(0, lambda: update_tcp(inputs, tcp), once=True)
                    buttons = ui.row()
                    with buttons: ui.button("表示", on_click=lambda: show_pose(inputs)).props("flat")
                return inputs, buttons
            start, _ = pose_card("開始位置 [deg]", [0, -20, 30, 0, 40, 0])
            goal, goal_buttons = pose_card("目標位置 [deg]", [40, -10, 20, 0, 60, 30])
            with goal_buttons:
                # 往復動作を作りやすいよう、開始と目標を入れ替える
                def swap():
                    for s, g in zip(start, goal): s.value, g.value = g.value, s.value
                ui.button("開始⇄目標", on_click=swap).props("flat")

            # 関節ごとの速度・加速度の上限（LIN・RMP でも関節はこれを超えない）と、LIN・RMP の先端の速度・加速度、サンプリング周期・動作時間（RMP はポリシーで決まるので使わない）
            with ui.card().classes("w-full"):
                ui.label("制限").classes("text-lg font-bold")
                ui.label("関節の最大速度 [deg/s]")
                max_vel = joint_inputs(defaults["max_vel"], 6)
                ui.label("関節の最大加速度 [deg/s²]")
                max_acc = joint_inputs(defaults["max_acc"], 6)
                with ui.grid(columns=2).classes("w-full").bind_visibility_from(mode, "value", backward=lambda m: m in ("lin", "rmp")):
                    lin_vel = ui.number("先端速度 [mm/s]", value=defaults["lin_vel"], min=0)
                    lin_acc = ui.number("先端加速度 [mm/s²]", value=defaults["lin_acc"], min=0)
                    rot_vel = ui.number("姿勢の角速度 [deg/s]", value=defaults["rot_vel"], min=0)
                    rot_acc = ui.number("姿勢の角加速度 [deg/s²]", value=defaults["rot_acc"], min=0)
                with ui.grid(columns=2).classes("w-full"):
                    dt = ui.number("周期 dt [s]", value=defaults["dt"], min=0.001, step=0.001, format="%.3f")
                    duration = ui.number("動作時間 [s]", min=0).props("hint=空欄なら制限内で最短").bind_visibility_from(mode, "value", backward=lambda m: m != "rmp")

            # 生成した軌道をグラフで確認し、robot-viser での再生や CSV 保存に使う
            async def generate():
                req = {"mode": mode.value, "start": [n.value for n in start], "goal": [n.value for n in goal], "max_vel": [n.value for n in max_vel],
                       "max_acc": [n.value for n in max_acc], "lin_vel": lin_vel.value, "lin_acc": lin_acc.value, "rot_vel": rot_vel.value, "rot_acc": rot_acc.value,
                       "dt": dt.value, "duration": duration.value}
                # LIN・RMP は逆運動学が解けない経路や目標へ収束しない場合などに生成できないため、理由を表示する（時間がかかる経路もあるのでタイムアウトなし）
                res = await client.post("/trajectory/csv", json=req, timeout=None)
                if res.status_code == 422: return ui.notify(res.json()["detail"], type="negative", multi_line=True)
                res.raise_for_status()
                state["csv"] = res.text
                # CSV（ヘッダ t,joint1..6）をグラフの系列にする
                rows = [[float(v) for v in line.split(",")] for line in res.text.splitlines()[1:]]
                chart.options["series"] = [{"name": f"J{j}", "type": "line", "showSymbol": False, "data": [[r[0], r[j]] for r in rows]} for j in range(1, 7)]
                chart.update()
                chart.set_visibility(True)
                ui.notify(f"{len(rows)}点 / {rows[-1][0]:.2f}秒の軌道を生成しました")

            # robot-viser に CSV ファイルとして送って再生させる（シーク・停止は viser 画面の Time スライダーと Play/Stop で行う）
            async def play():
                (await viser_api.post("/trajectory/upload", files={"file": ("motion.csv", state["csv"].encode())})).raise_for_status()

            with ui.card().classes("w-full"):
                with ui.row():
                    ui.button("生成", on_click=generate)
                    ui.button("再生", on_click=play).bind_enabled_from(state, "csv", backward=bool)
                    ui.button("CSV 保存", on_click=lambda: ui.download.content(state["csv"], f"{mode.value}_{datetime.now():%Y%m%d_%H%M%S}.csv")).props("outline").bind_enabled_from(state, "csv", backward=bool)
                chart = ui.echart({"tooltip": {"trigger": "axis"}, "legend": {"top": 0}, "grid": {"left": 40, "right": 10, "top": 50, "bottom": 40},
                                   "xAxis": {"type": "value", "name": "t [s]", "nameLocation": "middle", "nameGap": 25}, "yAxis": {"type": "value", "name": "deg"},
                                   "series": []}).classes("w-full h-64")
                chart.set_visibility(False)

        # 右: robot-viser の 3D ビューア。ブラウザが直接 viser に接続するので、このページと同じホスト名を使う
        viser_url = os.environ.get("VISER_URL", f"http://{request.url.hostname}:{os.environ.get('VISER_PORT', 8081)}")
        ui.element("iframe").props(f'src="{viser_url}"').classes("grow h-[calc(100vh-2rem)] border-0")


# 単体起動（Docker / Dev Container）用
if __name__ in {"__main__", "__mp_main__"}:
    ui.run(host="0.0.0.0", port=8180, title="robot-motion", reload=False, show=False)
