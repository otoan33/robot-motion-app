"""使い方マニュアル（docs/manual/manual.md）用のスクリーンショットを、サンプルの姿勢で操作しながら docs/manual/img/ に撮る。

アプリの改修後に画面を撮り直すためのスクリプト。robot-motion-app（docker compose up -d）と、衝突判定ありの robot-viser-app（COLLISION=1 docker compose up -d）を
起動した状態で、プロジェクト直下から実行する。撮影中は robot-viser の姿勢・障害物・軌道を書き換える（終わったら障害物は消す）。
他のブラウザで viser 画面を開いていると、そちらのカメラで描画されることがあるので閉じておく。

    docker run --rm --network host -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/work -w /work \\
        mcr.microsoft.com/playwright/python:v1.63.0-noble \\
        sh -c "pip install -q --user --break-system-packages playwright==1.63.0 && \\
               xvfb-run -a -s '-screen 0 1920x1080x24' python scripts/make_manual.py"

3D ビューアは WebGL で描画するため、ヘッドレスではなく Xvfb 上の Chromium で撮る（ヘッドレスでは撮影が止まる）。
Playwright の公式 Docker イメージでも動くよう、標準ライブラリと playwright だけを使う。
"""
import argparse
import re
from pathlib import Path

from playwright.sync_api import Locator, Page, sync_playwright

OUT = Path(__file__).resolve().parent.parent / "docs" / "manual" / "img"
# 姿勢は README・サンプル動画と同じ A → B。LIN の失敗例は B と先端の姿勢が同じで手首の形態だけが違う目標
A, B, B_FLIP = [0, -20, 30, 0, 40, 0], [40, -10, 20, 0, 60, 30], [40, -10, 20, 180, -60, -150]


class Manual:
    def __init__(self, page: Page, args: argparse.Namespace):
        self.page, self.args = page, args
        self.viser = page.frame_locator("iframe")

    def open(self):
        """robot-viser の障害物を消してから開き直す（前の操作の状態を残さない）。"""
        self.page.request.post(f"{self.args.viser_api}/obstacles", data={"obstacles": []})
        self.page.request.post(f"{self.args.viser_api}/joints", data={"angles": A})
        self.page.goto(self.args.url)
        self.page.get_by_text("先端 XYZ").first.wait_for()
        # viser が出す「ソフトウェア描画」の案内は操作に関係ないので閉じる
        toast = self.viser.locator(".mantine-Notification-root button")
        toast.first.wait_for(timeout=10_000)
        toast.first.click()
        self.page.wait_for_timeout(1500)

    def shot(self, name: str, *marks: Locator | tuple, top: Locator | None = None):
        """marks（tuple は複数の要素をまとめた範囲）を赤枠で囲んで撮る。top を指定するとその要素が上端に、無ければ最初の赤枠が中央に来るよう設定パネルをスクロールする。"""
        if top: top.evaluate("e => e.scrollIntoView({block: 'start'})")
        elif marks and isinstance(marks[0], Locator): marks[0].evaluate("e => e.scrollIntoView({block: 'center'})")
        self.page.mouse.move(400, 400)  # 数値欄の増減ボタンなど、マウスを載せたときの表示を消す
        self.page.wait_for_timeout(600)  # スクロール・メニュー・3D 描画が落ち着くのを待つ
        # iframe の中の要素も含めて、ページ上の位置で枠を描く
        for mark in marks:
            boxes = [loc.bounding_box() for loc in (mark if isinstance(mark, tuple) else (mark,))]
            x0, y0 = max(min(b["x"] for b in boxes) - 5, 1), max(min(b["y"] for b in boxes) - 5, 1)
            x1, y1 = min(max(b["x"] + b["width"] for b in boxes) + 5, 1279), min(max(b["y"] + b["height"] for b in boxes) + 5, 799)
            self.page.evaluate("""([l, t, w, h]) => { const d = document.createElement('div'); d.className = 'manual-mark';
                Object.assign(d.style, {position: 'fixed', left: `${l}px`, top: `${t}px`, width: `${w}px`, height: `${h}px`, boxSizing: 'border-box',
                    border: '3px solid #e53935', borderRadius: '6px', zIndex: 99999, pointerEvents: 'none'}); document.body.append(d); }""", [x0, y0, x1 - x0, y1 - y0])
        self.page.screenshot(path=OUT / f"{name}.png")
        self.page.evaluate("document.querySelectorAll('.manual-mark').forEach(e => e.remove())")
        print(name)

    def button(self, name: str) -> Locator:
        return self.page.get_by_role("button", name=name, exact=True)

    def card(self, title: str) -> Locator:
        return self.page.locator(".q-card:visible").filter(has=self.page.get_by_text(title, exact=True))

    def field(self, label: str, scope: Locator | None = None) -> Locator:
        return (scope or self.page).locator(".q-field:visible").filter(has=self.page.locator(".q-field__label", has_text=re.compile(f"^{re.escape(label)}$")))

    def set_joints(self, title: str, values: list[float]):
        for i, v in enumerate(values, 1):
            self.field(f"J{i}", self.card(title)).locator("input").fill(str(v))
        self.page.keyboard.press("Tab")
        self.page.wait_for_timeout(500)  # 先端の位置の表示が更新されるのを待つ

    def mode(self, label: str):
        self.page.locator(".q-select").click()
        self.page.locator(".q-menu").get_by_text(label, exact=True).click()
        self.page.wait_for_timeout(500)

    def notify(self, button: Locator, text: str) -> Locator:
        """前の通知を消してからボタンを押し、text を含む通知が出るまで待つ（生成は障害物回避で 10 秒ほどかかる）。"""
        self.page.evaluate("document.querySelectorAll('.q-notification').forEach(e => e.remove())")
        button.click()
        note = self.page.locator(".q-notification").filter(has_text=text).last
        note.wait_for(timeout=120_000)
        self.page.wait_for_timeout(1500)  # 通知が出そろい、グラフの描画が終わるのを待つ
        return note


def steps(m: Manual):
    page, chart = m.page, m.page.locator(".q-card:visible").filter(has=m.button("生成"))

    # ---- 基本の操作 ----
    m.open()
    m.shot("01_start", page.locator(".overflow-y-auto").first, page.locator("iframe"))
    page.locator(".q-select").click()
    m.shot("02_mode", page.locator(".q-menu"))
    page.keyboard.press("Escape")
    m.shot("03_pose", (m.card("開始位置 [deg]").locator(".nicegui-grid"), m.card("開始位置 [deg]").locator(".text-xs")), m.card("目標位置 [deg]").locator(".nicegui-grid"), top=m.card("開始位置 [deg]"))
    m.button("表示").nth(1).click()
    page.wait_for_timeout(1500)  # robot-viser に目標の姿勢が描かれるのを待つ
    m.shot("04_show", m.button("表示").nth(1), page.locator("iframe"))
    m.button("開始⇄目標").click()
    m.shot("05_swap", m.button("開始⇄目標"), m.card("目標位置 [deg]").locator(".nicegui-grid"), m.card("開始位置 [deg]").locator(".nicegui-grid"), top=m.card("開始位置 [deg]"))
    m.button("開始⇄目標").click()

    # ---- PTP ----
    limits = m.card("制限")
    # 再生の途中を撮れるよう、動作時間を指定してゆっくり動かす
    m.field("動作時間 [s]").locator("input").fill("3")
    m.shot("06_limits", limits.locator(".nicegui-grid").nth(0), limits.locator(".nicegui-grid").nth(1), limits.locator(".nicegui-grid").last, top=limits)
    m.notify(m.button("生成"), "の軌道を生成しました")
    m.shot("07_generate", m.button("生成"), chart.locator(".nicegui-echart"), page.locator(".q-notification").last, top=chart)
    # 3D の描画中は撮影に数秒かかり再生が終わってしまうため、途中で Stop を押して止めた姿勢を撮る
    m.button("再生").click()
    page.wait_for_timeout(1500)
    m.viser.get_by_role("button", name="Stop").click()
    m.shot("08_play", m.button("再生"), (m.viser.get_by_text("Time (frame)"), m.viser.get_by_role("button", name="Play")), top=chart)
    with page.expect_download():
        m.button("CSV 保存").click()
    m.shot("09_csv", m.button("CSV 保存"), top=chart)

    # ---- LIN ----
    m.mode("LIN（直線補間・台形速度）")
    m.shot("10_lin", limits.locator(".nicegui-grid").nth(2), limits.locator(".nicegui-grid").last, top=limits)

    # ---- RMP 目標到達 ----
    m.mode("PTP（RMP 目標到達ポリシー）")
    m.notify(m.button("生成"), "の軌道を生成しました")
    m.shot("11_rmp", m.button("生成"), chart.locator(".nicegui-echart"), page.locator(".q-notification").last, top=chart)

    # ---- 経路追従 ----
    m.mode("経路追従（RMP・経由点の折れ線）")
    m.button("経由点を追加").click()
    via = m.card("経由点 [deg]")
    m.shot("12_via", m.button("経由点を追加"), via.locator(".nicegui-grid"), m.button("削除"), top=via)
    m.shot("13_blend", m.field("角の丸め [mm]"), top=limits.locator(".nicegui-grid").nth(2))
    m.notify(m.button("生成"), "の軌道を生成しました")
    m.shot("14_path", m.button("生成"), chart.locator(".nicegui-echart"), page.locator(".q-notification").last, top=chart)

    # ---- 障害物回避（目標到達で、開始と目標の間に置いた球を避ける） ----
    m.mode("PTP（RMP 目標到達ポリシー）")
    avoid = m.card("障害物回避")
    m.button("例を入れる").click()
    page.wait_for_timeout(500)
    m.shot("15_obstacle", m.button("例を入れる"), avoid.locator("textarea"), top=avoid)
    note = m.notify(m.button("viser に送る"), "障害物を robot-viser に送りました")
    page.wait_for_timeout(1000)  # robot-viser に球が描かれるのを待つ
    m.shot("16_send", m.button("viser に送る"), note, page.locator("iframe"), top=avoid)
    page.get_by_text("障害物を避ける", exact=True).click()
    note = m.notify(m.button("生成"), "最小距離")
    m.shot("17_avoid", (page.locator(".q-checkbox"), m.field("影響距離 [mm]")), m.button("生成"), note, top=avoid)

    # ---- 困ったとき（開き直してから、わざと失敗させる） ----
    m.open()
    m.mode("LIN（直線補間・台形速度）")
    m.set_joints("目標位置 [deg]", B_FLIP)
    note = m.notify(m.button("生成"), "一致しません")
    m.shot("18_lin_error", m.card("目標位置 [deg]").locator(".nicegui-grid"), note, top=m.card("目標位置 [deg]"))
    m.open()
    m.mode("PTP（RMP 目標到達ポリシー）")
    page.get_by_text("障害物を避ける", exact=True).click()
    m.card("障害物回避").locator("textarea").fill('[{"type": "sphere", "center": [0.3, 0.8, 1.3], "radius": 0.06,}]')
    note = m.notify(m.button("生成"), "JSON が読めません")
    m.shot("19_json_error", m.card("障害物回避").locator("textarea"), note, top=m.card("障害物回避"))

    # 撮影で置いた障害物を robot-viser から消す
    page.request.post(f"{m.args.viser_api}/obstacles", data={"obstacles": []})


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://localhost:8180", help="robot-motion-app の画面の URL")
    parser.add_argument("--viser-api", default="http://localhost:8000", help="robot-viser-app の API の URL")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    with sync_playwright() as p:
        # 3D ビューア（WebGL）を描くため、Xvfb 上で GPU の代わりに Mesa を使う
        browser = p.chromium.launch(headless=False, args=["--ignore-gpu-blocklist", "--use-angle=gl"])
        page = browser.new_page(viewport={"width": 1280, "height": 800}, accept_downloads=True)
        page.set_default_timeout(30_000)
        steps(Manual(page, args))
        browser.close()


if __name__ == "__main__":
    main()
