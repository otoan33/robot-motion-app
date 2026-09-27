"""各ポリシーのサンプル動作を robot-viser-app で録画し、ポリシーごとに 1 本の動画（docs/videos/*.mp4）にする。

アプリの改修後に撮り直すためのスクリプト。robot-motion-app（docker compose up -d）と、衝突判定ありの robot-viser-app（COLLISION=1 docker compose up -d）を
起動した状態で、プロジェクト直下から実行する。録画は robot-viser-app の /trajectory/record（viser 画面を開いているブラウザで 1 コマずつ描画）を使う。
他のブラウザで viser 画面を開いていると、そちらのカメラで描画されることがあるので閉じておく。

    docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/work -w /work \\
        mcr.microsoft.com/playwright/python:v1.63.0-noble \\
        sh -c "pip install -q --user --break-system-packages playwright==1.63.0 numpy matplotlib pillow imageio imageio-ffmpeg httpx && \\
               xvfb-run -a -s '-screen 0 1920x1080x24' python scripts/make_videos.py"

--dry-run を付けると録画せず、各サンプルの生成結果（時間・点数・障害物との最小距離）だけを表示する。--only 3 のように番号を指定すると、その動画だけを作る。
3D ビューアは WebGL で描画するため、robot-viser-app の撮影スクリプトと同じく Xvfb 上の Chromium（Mesa の llvmpipe）を使う（1 コマ 0.4 秒ほど）。
"""
import argparse
import csv
import io
import sys
import tempfile
from pathlib import Path

import httpx
import imageio.v2 as imageio
import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from playwright.sync_api import sync_playwright

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend.planner import tcp_pose  # noqa: E402

MOTION_URL, VISER_API_URL, VISER_URL = "http://127.0.0.1:8100", "http://127.0.0.1:8000", "http://127.0.0.1:8081"
OUT = Path("docs/videos")
FPS, RENDER_W, PANEL_W, H = 20, 800, 480, 720
FONT = "/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf"
plt.rcParams["font.family"] = "IPAGothic"

# サンプルの姿勢（関節角度 [deg]）。PTP・LIN・RMP は同じ A→B→C→A をつなげて、ポリシーの違いを比べられるようにする
A, B, C = [0, -20, 30, 0, 40, 0], [40, -10, 20, 0, 60, 30], [-35, 5, 10, 0, 45, -30]


# 開始・目標の先端位置 [m] の中間を少しずらした点（障害物の置き場所）
def mid(p, q, offset=(0, 0, 0)):
    return (np.add(tcp_pose(p)[0], tcp_pose(q)[0]) / 2000 + offset).round(3).tolist()


# 動画ごとの題名・説明・サンプル（説明文, 生成条件, 障害物）。生成条件は robot-motion-app の /trajectory のリクエスト
VIDEOS = [
    ("01_ptp", "PTP（関節補間・台形速度）", "全関節が同時に加減速し、同時に目標へ着く。先端の軌跡は曲線になる",
     [(f"{n}", {"mode": "ptp", "start": s, "goal": g, "max_vel": [30] * 6, "max_acc": [60] * 6}, [])
      for n, s, g in [("A → B", A, B), ("B → C", B, C), ("C → A", C, A)]]),
    ("02_lin", "LIN（直線補間・台形速度）", "先端を直線、姿勢を一定の回転軸まわりに補間し、各時刻の関節角度を逆運動学で求める",
     [(f"{n}", {"mode": "lin", "start": s, "goal": g, "lin_vel": 250, "lin_acc": 500}, [])
      for n, s, g in [("A → B", A, B), ("B → C", B, C), ("C → A", C, A)]]),
    ("03_rmp", "RMP 目標到達ポリシー", "先端の位置・姿勢と関節角度への引き寄せを合成。遠くでは一定の速さで向かい、近くで滑らかに止まる",
     [(f"{n}", {"mode": "rmp", "start": s, "goal": g}, []) for n, s, g in [("A → B", A, B), ("B → C", B, C), ("C → A", C, A)]]),
    ("04_rmp_path", "RMP 経路追従（経由点の折れ線）", "経由点を結んだ折れ線（角は丸める）に沿って、経路上の位置で進む。遅れても急がない",
     [("A → B（経由点 1、角の丸め 50 mm）", {"mode": "rmp_path", "start": A, "via": [[20, -30, 50, 0, 20, 15]], "goal": B, "blend": 50}, []),
      ("B → C（経由点 2、角の丸め 100 mm）", {"mode": "rmp_path", "start": B, "via": [[25, -30, 45, 0, 40, 15], [-15, -25, 40, 0, 35, -15]], "goal": C, "blend": 100}, []),
      ("C → A（経由点 1、角の丸め 0：経由点で止まる）", {"mode": "rmp_path", "start": C, "via": [[-15, 5, 5, 0, 60, -15]], "goal": A, "blend": 0}, [])]),
    ("05_rmp_avoid", "RMP ＋ 障害物回避", "robot-viser-app の距離計算で、近似球と障害物の距離の RMP を合成。近づくときだけ強くブレーキして沿うように避ける",
     [("目標到達 A → B：先端の直線上に球", {"mode": "rmp", "start": A, "goal": B, "avoid": True},
       [{"type": "sphere", "name": "ball", "center": mid(A, B), "radius": 0.06}]),
      ("目標到達 B → C：先端の直線上に柱", {"mode": "rmp", "start": B, "goal": C, "avoid": True},
       [{"type": "capsule", "name": "pole", "p1": mid(B, C, (0, 0, -0.4)), "p2": mid(B, C, (0, 0, 0.05)), "radius": 0.04}]),
      ("経路追従 A → B（直線）：経路の 22 mm 横を通る球", {"mode": "rmp_path", "start": A, "goal": B, "avoid": True},
       [{"type": "sphere", "name": "ball", "center": mid(A, B, (0, 0, 0.18)), "radius": 0.06}])]),
]


# 生成した CSV（t 形式）→ 時刻 [s] と関節角度 [deg]
def parse(text: str) -> tuple[np.ndarray, np.ndarray]:
    rows = np.array([[float(v) for v in r] for r in list(csv.reader(text.splitlines()))[1:]])
    return rows[:, 0], rows[:, 1:]


# 右側のパネル（関節角度と、先端の速さまたは障害物との距離の時系列）を描き、時刻 → 横位置 [px] の変換と一緒に返す
def draw_panel(t, q, metric, metric_label) -> tuple[Image.Image, callable]:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(PANEL_W / 100, H / 100), dpi=100, sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    for j in range(6): ax1.plot(t, q[:, j], label=f"J{j + 1}", lw=1.5)
    ax1.set_ylabel("関節角度 [deg]"), ax1.legend(ncol=3, fontsize=8, loc="best"), ax1.grid(alpha=0.3)
    ax2.plot(t, metric, color="#444", lw=1.5)
    if "距離" in metric_label: ax2.axhline(0, color="#d33", lw=1, ls="--")
    ax2.set_ylabel(metric_label), ax2.set_xlabel("t [s]"), ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    img = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
    # データ座標の時刻 → 図の横位置（画素）。縦はパネル全体に線を引く
    to_px = lambda x: ax1.transData.transform((x, 0))[0]
    plt.close(fig)
    return img, to_px


# 文字を描く（半透明の帯の上）
def caption(img: Image.Image, lines: list[tuple[str, int]]) -> Image.Image:
    img = img.convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    y, boxes = 12, []
    for text, size in lines:
        f = ImageFont.truetype(FONT, size)
        w = d.textlength(text, font=f)
        boxes.append((text, f, y, w))
        y += size + 10
    d.rectangle([0, 0, max(w for *_, w in boxes) + 32, y + 4], fill=(255, 255, 255, 210))
    for text, f, y0, _ in boxes: d.text((16, y0), text, font=f, fill=(20, 20, 20, 255))
    return Image.alpha_composite(img, layer).convert("RGB")


# 題名の画面（ポリシー名・説明・サンプルの一覧）
def title_card(title: str, desc: str, samples: list[str]) -> np.ndarray:
    img = Image.new("RGB", (RENDER_W + PANEL_W, H), "white")
    d = ImageDraw.Draw(img)
    d.text((80, 200), title, font=ImageFont.truetype(FONT, 48), fill="#111")
    d.text((80, 290), desc, font=ImageFont.truetype(FONT, 22), fill="#333")
    for i, s in enumerate(samples): d.text((100, 370 + i * 44), f"{i + 1}. {s}", font=ImageFont.truetype(FONT, 26), fill="#333")
    return np.asarray(img)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="録画せず、生成結果だけを表示する")
    parser.add_argument("--only", type=int, nargs="*", help="作る動画の番号（1 始まり）")
    args = parser.parse_args()
    motion, viser = httpx.Client(base_url=MOTION_URL, timeout=None), httpx.Client(base_url=VISER_API_URL, timeout=None)
    OUT.mkdir(parents=True, exist_ok=True)

    with sync_playwright() as p:
        page = None
        if not args.dry_run:
            # 録画の描画用に viser 画面を開き、ロボットが大きく写るように寄って、注視点を床からアームの高さへ上げる（右ドラッグで平行移動）
            browser = p.chromium.launch(headless=False, args=["--ignore-gpu-blocklist", "--use-angle=gl"])
            page = browser.new_page(viewport={"width": RENDER_W, "height": H})
            page.goto(VISER_URL)
            page.wait_for_timeout(8000)
            page.mouse.move(RENDER_W / 2, H / 2)
            for _ in range(9): page.mouse.wheel(0, -150); page.wait_for_timeout(300)
            page.mouse.move(RENDER_W / 2, 300); page.mouse.down(button="right"); page.mouse.move(RENDER_W / 2, 360, steps=10); page.mouse.up(button="right")
            page.wait_for_timeout(2000)

        for no, (name, title, desc, samples) in enumerate(VIDEOS, 1):
            if args.only and no not in args.only: continue
            print(f"== {no}. {title}", flush=True)
            frames = [title_card(title, desc, [s for s, _, _ in samples])] * (FPS * 3)
            for i, (label, req, obstacles) in enumerate(samples, 1):
                # 障害物を登録してから生成する（回避の計算と録画の表示に使う）
                viser.post("/obstacles", json={"obstacles": obstacles}).raise_for_status()
                res = motion.post("/trajectory/csv", json=req)
                if res.status_code != 200: raise SystemExit(f"{label}: {res.text}")
                t, q = parse(res.text)
                dmin = res.headers.get("X-Min-Distance")
                print(f"  {i}. {label}: {t[-1]:.2f} s / {len(t)} 点" + (f" / 障害物との最小距離 {dmin} mm" if dmin else ""), flush=True)
                if args.dry_run: continue

                # 右のパネルの下段: 障害物があれば近似球と障害物の最小距離、なければ先端の速さ
                if obstacles:
                    metric = np.array([viser.post("/distances", json={"angles": list(a)}).json()["min_distance"] * 1000 for a in q])
                    metric_label = "障害物との最小距離 [mm]"
                else:
                    P = np.array([tcp_pose(a)[0] for a in q])
                    metric, metric_label = np.linalg.norm(np.gradient(P, t, axis=0), axis=1), "先端の速さ [mm/s]"
                panel, to_px = draw_panel(t, q, metric, metric_label)

                # robot-viser-app で録画（1 コマずつ描画した mp4）し、コマごとに説明とパネル（今の時刻の縦線つき）を付ける
                video = viser.post("/trajectory/record", params={"format": "mp4"}, files={"file": ("motion.csv", res.text.encode())})
                video.raise_for_status()
                with tempfile.NamedTemporaryFile(suffix=".mp4") as f:
                    f.write(video.content)
                    f.flush()
                    render = [Image.fromarray(fr).resize((RENDER_W, H)) for fr in imageio.mimread(f.name, memtest=False)]
                sub = f"{i}/{len(samples)}  {label}" + (f"（最小距離 {dmin} mm）" if dmin else "")
                for k, fr in enumerate(render):
                    left = caption(fr, [(title, 28), (sub, 22)])
                    right = panel.copy()
                    x = to_px(min(k / FPS, t[-1]))
                    ImageDraw.Draw(right).line([(x, 0), (x, H)], fill=(220, 40, 40), width=2)
                    frame = Image.new("RGB", (RENDER_W + PANEL_W, H), "white")
                    frame.paste(left, (0, 0)), frame.paste(right, (RENDER_W, 0))
                    frames.append(np.asarray(frame))
                # 次のサンプルの前に、最後のコマを 1 秒止める
                frames += [frames[-1]] * FPS
            viser.post("/obstacles", json={"obstacles": []}).raise_for_status()
            if args.dry_run: continue
            path = OUT / f"{name}.mp4"
            imageio.mimwrite(path, frames, fps=FPS, quality=7, macro_block_size=8)
            print(f"  -> {path}（{len(frames) / FPS:.1f} 秒）", flush=True)


if __name__ == "__main__":
    main()
