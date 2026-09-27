# robot-motion-app

robot-viser-app で再生する「時系列の関節角度軌道」を生成するアプリ。フロントエンドを NiceGUI、バックエンドを FastAPI で作り、構成は robot-viser-app に合わせている。現在の動作モードは **PTP（関節補間・台形速度）** のみ。

## 構成

```
robot-motion-app/
├── compose.yaml              # frontend + backend を Docker で起動（自動起動あり）
├── .dockerignore             # ルートをコンテキストにするビルド用（requirements.txt のみ送る）
├── .devcontainer/            # VSCode Dev Container 設定
├── backend/                  # FastAPI（軌道生成 API :8100）
│   ├── main.py               # API 定義（/modes, /trajectory, /trajectory/csv）
│   ├── planner.py            # 軌道生成（PTP・台形速度）
│   ├── requirements.txt
│   └── Dockerfile
└── frontend/                 # NiceGUI（画面 :8180）
    ├── main.py               # 設定パネル + robot-viser の viser 画面（iframe）
    ├── requirements.txt
    └── Dockerfile
```

### 通信の流れ

```
ブラウザ ──> frontend (NiceGUI :8180) ──HTTP──> backend (FastAPI :8100)              … 軌道の生成
   │                                   └─HTTP──> robot-viser-app backend (:8000)    … 姿勢の表示・軌道の再生
   └── iframe ──> robot-viser-app viser (:8081)                                      … 3D 表示
```

backend は軌道の生成だけを行い、viser には依存しない。robot-viser-app への姿勢・軌道の送信は frontend が行う。
robot-viser-app と同時に動かすため、ポートはずらしている。

| | robot-viser-app | robot-motion-app |
|---|---|---|
| 画面（NiceGUI） | 8080 | 8180 |
| API（FastAPI） | 8000 | 8100 |
| 3D ビューア（viser） | 8081 | なし（robot-viser-app のものを表示） |

frontend の接続先は環境変数で変えられる。

| 環境変数 | 既定値 | 用途 |
|---|---|---|
| `BACKEND_URL` | `http://127.0.0.1:8100` | 軌道生成 API |
| `VISER_API_URL` | `http://127.0.0.1:8000` | robot-viser-app の API（`/joints`, `/trajectory/upload`） |
| `VISER_URL` | `http://{画面を開いたホスト名}:8081` | iframe で表示する viser 画面（ブラウザから直接つなぐ） |

## 実行する

先に robot-viser-app を起動しておく（robot-viser-app で `docker compose up -d`）。

```bash
docker compose up -d --build
```

- 画面: http://localhost:8180
- API: http://localhost:8100（ドキュメント: http://localhost:8100/docs）

compose では frontend に `VISER_API_URL=http://host.docker.internal:8000` を渡し、別の compose で動いている robot-viser-app の API へホスト経由で接続する。

Dev Container で開いた場合は、backend（`--reload` 付き）と frontend が自動で起動する（ログは `/tmp/backend.log`, `/tmp/frontend.log`）。

## 画面

| 項目 | 内容 |
|---|---|
| 動作モード | 生成する動作の種類（現在は PTP のみ） |
| 開始位置 / 目標位置 | J1〜J6 [deg]。「表示」で robot-viser にその姿勢を表示する。「開始⇄目標」で入れ替える |
| 制限 | 関節ごとの最大速度 [deg/s]・最大加速度 [deg/s²]、周期 dt [s]、動作時間 [s]（空欄なら制限内で最短） |
| 生成 | 軌道を生成し、関節角度の時系列グラフを表示する |
| 再生 | 生成した軌道を robot-viser に送って再生する（シーク・停止は viser 画面の Time スライダーと Play/Stop） |
| CSV 保存 | 生成した軌道を CSV でダウンロードする |

## 軌道生成

### PTP（関節補間・台形速度）

全関節に共通の経路パラメータ s（0→1）を台形速度で動かし、`q(t) = q_start + (q_goal - q_start)·s(t)` とする。全関節が同時に加速・減速し、同時に目標へ着く。

- s の速度・加速度の上限は、移動量に対して制限が最も厳しい関節で決まる
- 最高速度に届く移動量なら台形、届かなければ三角の速度波形になる（どちらも制限内で最短時間）
- 動作時間を指定すると、最短時間より長い場合だけその時間にする。このとき加速度は上限のまま、頂点速度を下げて時間を合わせる
- 時刻は 0 から dt 刻みで、終端（動作時間ちょうど）を必ず含む

## API

| メソッド | パス | 内容 |
|---|---|---|
| GET | `/modes` | 選べる動作モードと既定値（最大速度・最大加速度・dt） |
| POST | `/trajectory` | 軌道を生成して JSON（`times`, `angles`, `duration_sec`, `num_points`）で返す |
| POST | `/trajectory/csv` | 軌道を生成して CSV テキストで返す |

リクエスト（`/trajectory`, `/trajectory/csv` 共通）:

```json
{
  "mode": "ptp",
  "start": [0, 0, 0, 0, 0, 0],
  "goal": [30, -30, 30, 0, 45, 0],
  "max_vel": [180, 180, 180, 180, 180, 180],
  "max_acc": [720, 720, 720, 720, 720, 720],
  "dt": 0.01,
  "duration": null
}
```

`max_vel`・`max_acc`・`dt`・`duration` は省略できる（既定値は `/modes` の値、`duration` は最短）。

```bash
curl -X POST http://localhost:8100/trajectory/csv -H 'Content-Type: application/json' \
  -d '{"start":[0,0,0,0,0,0],"goal":[30,-30,30,0,45,0]}' > ptp.csv
```

### CSV 形式

robot-viser-app がそのまま読める「t 形式」。

```
t,joint1,joint2,joint3,joint4,joint5,joint6
0.0000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000
0.0100,0.024000,-0.024000,0.024000,0.000000,0.036000,0.000000
...
```

`t` は秒、`joint1`〜`joint6` は度。
