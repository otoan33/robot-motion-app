# robot-motion-app

robot-viser-app で再生する「時系列の関節角度軌道」を生成するアプリ。フロントエンドを NiceGUI、バックエンドを FastAPI で作り、構成は robot-viser-app に合わせている。動作モードは台形速度の **PTP（関節補間）**・**LIN（先端の直線補間）** と、**RMP（Riemannian Motion Policies）の目標到達ポリシーによる PTP**。

## 構成

```
robot-motion-app/
├── compose.yaml              # frontend + backend を Docker で起動（自動起動あり）
├── .dockerignore             # ルートをコンテキストにするビルド用（requirements.txt のみ送る）
├── .devcontainer/            # VSCode Dev Container 設定
├── backend/                  # FastAPI（軌道生成 API :8100）
│   ├── main.py               # API 定義（/modes, /fk, /trajectory, /trajectory/csv）
│   ├── planner.py            # 軌道生成（台形速度の PTP・LIN）
│   ├── rmp.py                # 軌道生成（RMP の目標到達ポリシーによる PTP）
│   ├── kinematics.py         # URDF からの順運動学・逆運動学
│   ├── assets/arms/robotA/arm.urdf  # robot-viser-app と同じアームの URDF（関節の位置・回転軸だけを使う）
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
| 動作モード | PTP（関節補間）/ LIN（先端の直線補間）/ PTP（RMP 目標到達ポリシー） |
| 開始位置 / 目標位置 | J1〜J6 [deg]。その姿勢での先端（tool0）の位置 [mm]・姿勢 RPY [deg] を下に表示する。「表示」で robot-viser にその姿勢を表示する。「開始⇄目標」で入れ替える |
| 制限 | 関節ごとの最大速度 [deg/s]・最大加速度 [deg/s²]、LIN・RMP のときは先端の速度 [mm/s]・加速度 [mm/s²]・姿勢の角速度 [deg/s]・角加速度 [deg/s²]、周期 dt [s]、動作時間 [s]（PTP・LIN のみ。空欄なら制限内で最短） |
| 生成 | 軌道を生成し、関節角度の時系列グラフを表示する。生成できないとき（LIN で関節の形態が違うなど）は理由を表示する |
| 再生 | 生成した軌道を robot-viser に送って再生する（シーク・停止は viser 画面の Time スライダーと Play/Stop） |
| CSV 保存 | 生成した軌道を CSV でダウンロードする |

## 軌道生成

### PTP（関節補間・台形速度）

全関節に共通の経路パラメータ s（0→1）を台形速度で動かし、`q(t) = q_start + (q_goal - q_start)·s(t)` とする。全関節が同時に加速・減速し、同時に目標へ着く。

- s の速度・加速度の上限は、移動量に対して制限が最も厳しい関節で決まる
- 最高速度に届く移動量なら台形、届かなければ三角の速度波形になる（どちらも制限内で最短時間）
- 動作時間を指定すると、最短時間より長い場合だけその時間にする。このとき加速度は上限のまま、頂点速度を下げて時間を合わせる
- 時刻は 0 から dt 刻みで、終端（動作時間ちょうど）を必ず含む

### LIN（先端の直線補間・台形速度）

開始・目標の関節角度から順運動学で先端（URDF の `tool0`、ハンドなしのフランジ）の姿勢を求め、その間を補間する。

- 位置は直線、姿勢は開始→目標の回転を一定の回転軸まわりに補間し、同じ経路パラメータ s（台形速度）で動かす
- s の上限は、先端の速度・加速度（移動距離に対して）と姿勢の角速度・角加速度（回転角に対して）のうち厳しい方で決まる
- 各時刻の関節角度は、ひとつ前の解を初期値にした逆運動学（減衰最小二乗法）で求める。開始の関節の形態のまま連続にたどる
- 関節の速度・加速度が上限を超える場合は、時間軸を一様に引き延ばして（速度 1/r、加速度 1/r²）上限内に収める。特異点の近くを通る経路はそのぶん長い時間になる
- 生成できない場合は 422 で理由を返す
  - 経路上で逆運動学が解けない（特異点・可動範囲外）
  - 目標の関節角度が、先端の姿勢は同じでも開始と別の形態（手首の反転など）で、直線補間の終点と一致しない（±360° の違いは同じとみなす）

### RMP（目標到達ポリシーによる PTP）

RMPflow の形で、いくつかの空間に置いたポリシー（RMP）を関節空間で合成し、関節の加速度を解いて時間積分する。

- RMP はそれぞれ加速度 `a` と計量 `M = w·I` を持ち、関節空間へヤコビアン `J` で引き戻して `q̈ = (Σ w JᵀJ)⁻¹ Σ w Jᵀa` とする（J̇q̇ の曲率項は省略）
- 置いている RMP（重み w）
  - 先端（tool0）の位置を、目標の関節角度での先端位置へ引き寄せる（1.0）
  - 先端の姿勢を、同じく目標の姿勢へ引き寄せる（0.3）
  - 関節角度を目標の関節角度へ弱く引き寄せる（0.05）。先端が同じ姿勢になる別の形態で止まらず、目標の形態に着くようにする
- 各 RMP は目標到達ポリシー `a = α·s(x_g − x) − β·ẋ`（`s` は RMPflow のソフト正規化）
  - 遠くでは最大速度 `α/β` で目標へ向かい、目標の近く（ソフト正規化の幅 η の内側）では臨界減衰のばね（剛性 `β²/4`）として止まる
  - `β = 最大加速度 / 最大速度`。先端は画面の先端速度・加速度、姿勢は角速度・角加速度、関節は関節の最大速度の最小値と先端と同じ β を使う
- 関節の加速度・速度が上限を超えるときは、向きを保ったまま全体を縮める
- 先端は目標へほぼ直線的に寄っていく（関節・姿勢の RMP との合成のため厳密な直線ではない）。目標へは漸近的に近づくため、関節の誤差と速度がともに 0.01 以下（deg, deg/s）になった時点で終え、最後の点を目標ちょうどにする。動作時間はこの到達までの時間になる
- 60 秒以内に収束しない場合は 422 で理由を返す（手首の反転など、先端では目標に着いても関節の形態が違って止まってしまう場合）
- 障害物回避などのポリシーは、`backend/rmp.py` の `leaves` に RMP（ヤコビアン, 加速度, 重み）を足せば組み込める（robot-viser-app の `/distances` が返す制御点のヤコビアンと距離を使う想定）

## API

| メソッド | パス | 内容 |
|---|---|---|
| GET | `/modes` | 選べる動作モードと既定値（関節・先端の速度と加速度、dt） |
| POST | `/fk` | 関節角度 `{"angles": [6]}` [deg] での先端の位置 [mm] と姿勢 RPY [deg] |
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
  "lin_vel": 250,
  "lin_acc": 1000,
  "rot_vel": 90,
  "rot_acc": 360,
  "dt": 0.01,
  "duration": null
}
```

`mode` は `"ptp"`・`"lin"`・`"rmp"`。`lin_vel`・`lin_acc`（先端 [mm/s]・[mm/s²]）と `rot_vel`・`rot_acc`（姿勢 [deg/s]・[deg/s²]）は LIN・RMP で使い、`duration` は RMP では使わない。`mode` 以外の制限・`dt`・`duration` は省略できる（既定値は `/modes` の値、`duration` は最短）。

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
