import os
from datetime import datetime

import httpx
from fastapi import Request
from nicegui import ui

# 軌道生成 API（本アプリの backend）と、表示・再生を任せる robot-viser-app の API への接続
client = httpx.AsyncClient(base_url=os.environ.get("BACKEND_URL", "http://127.0.0.1:8100"))
viser_api = httpx.AsyncClient(base_url=os.environ.get("VISER_API_URL", "http://127.0.0.1:8000"))

# 動作モードの表示名
MODE_LABELS = {"ptp": "PTP（関節補間・台形速度）"}


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

            # 動作モード（今は PTP のみ）
            with ui.card().classes("w-full"):
                ui.label("動作モード").classes("text-lg font-bold")
                mode = ui.select({m: MODE_LABELS.get(m, m) for m in info["modes"]}, value=info["modes"][0]).classes("w-full")

            # 開始位置・目標位置 [deg]。「表示」で robot-viser にその姿勢を表示して確認する
            async def show_pose(inputs): (await viser_api.post("/joints", json={"angles": [n.value for n in inputs]})).raise_for_status()
            with ui.card().classes("w-full"):
                ui.label("開始位置 [deg]").classes("text-lg font-bold")
                start = joint_inputs([0] * 6)
                ui.button("表示", on_click=lambda: show_pose(start)).props("flat")
            with ui.card().classes("w-full"):
                ui.label("目標位置 [deg]").classes("text-lg font-bold")
                goal = joint_inputs([30, -30, 30, 0, 45, 0])
                with ui.row():
                    ui.button("表示", on_click=lambda: show_pose(goal)).props("flat")
                    # 往復動作を作りやすいよう、開始と目標を入れ替える
                    def swap():
                        for s, g in zip(start, goal): s.value, g.value = g.value, s.value
                    ui.button("開始⇄目標", on_click=swap).props("flat")

            # 関節ごとの速度・加速度の上限と、サンプリング周期・動作時間
            with ui.card().classes("w-full"):
                ui.label("制限").classes("text-lg font-bold")
                ui.label("最大速度 [deg/s]")
                max_vel = joint_inputs(defaults["max_vel"], 6)
                ui.label("最大加速度 [deg/s²]")
                max_acc = joint_inputs(defaults["max_acc"], 6)
                with ui.grid(columns=2).classes("w-full"):
                    dt = ui.number("周期 dt [s]", value=defaults["dt"], min=0.001, step=0.001, format="%.3f")
                    duration = ui.number("動作時間 [s]", min=0).props("hint=空欄なら制限内で最短")

            # 生成した軌道をグラフで確認し、robot-viser での再生や CSV 保存に使う
            async def generate():
                req = {"mode": mode.value, "start": [n.value for n in start], "goal": [n.value for n in goal], "max_vel": [n.value for n in max_vel],
                       "max_acc": [n.value for n in max_acc], "dt": dt.value, "duration": duration.value}
                res = await client.post("/trajectory/csv", json=req)
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
