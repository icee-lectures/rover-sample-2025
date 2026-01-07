# リモート制御端末（PC）側 ROSノード

## 前提

- Ubuntu 24.04 / ROS 2 Jazzy で動作確認
- ゲームコントローラ利用時は物理マシンでの実行を推奨（仮想環境では認識しない場合あり）
- キーボード入力は pygame でウィンドウを開くためディスプレイ環境が必要

## クイックガイド

### インストール

1. 依存パッケージ等をインストール: `sudo ~/rover-sample-2025/remote/install_dependency.sh` など
2. `rover_ws` をビルド: `cd ~/rover-sample-2025/remote/rover_ws && colcon build --event-handlers console_direct+`
3. ROS2環境変数を設定: `rosenv_default.sh` を `rosenv.sh` にコピーして編集

### 通常使用

#### 実行中のシェルでROS2を読み込む場合

ROS2読込スクリプトを実行: `source ~/rover-sample-2025/rover/start_ROS.sh`

## ディレクトリ・ファイルの説明

### ルートディレクトリ

- `rosenv_default.sh`: ROS 2 関連の環境変数テンプレート（ROS_DOMAIN_ID, Discovery Server など）
  - 編集する場合はコピーして `rosenv.sh` を作成し、そちらを編集してください
- `start_gamepad.sh`: ゲームコントローラ用ローンチを起動
- `start_wasd-controller.sh`: キーボード（WASD）用ローンチを起動
- `start_ROS.sh`: ROS 2 本体とワークスペースの読み込み（rqt 起動や開発時などに使用）

### remote_ws/

ROS2ワークスペース

#### remote_ws/src/

- パッケージ: **gamepad**
  - ノード: `gamepad_publisher.py`（/joy を発行）, `joy_to_cmd_vel.py`（/joy→/cmd_vel 変換）
  - launchファイル: `gamepad.launch.py`（上記2ノードを同時起動）

- パッケージ: **keyboard**
  - ノード: `keyboard_publisher.py`（押下キーを `/keyboard` に配信）, `wasd_controller.py`（`/keyboard`→`/cmd_vel` 変換）
  - launchファイル: `wasd_controller.launch.py`（上記2ノードを同時起動）

### src/

パッケージ化していない補助スクリプト・ユーティリティ

- `clean_ros_ws.sh`: `remote_ws` の build/install/log を削除してクリーンにするスクリプト
- `test_gamepad.py`: ゲームパッド入力の動作確認用スクリプト
- `test_keyboard-monitor.py`: キーボード入力のモニタ用スクリプト

## 使用準備

### シェルスクリプトの実行権限付与

入手直後はスクリプトに実行権限が付いていない場合があります

```bash
cd ~/rover-sample-2025/remote
chmod +x start_gamepad.sh start_wasd-controller.sh
```

### 依存パッケージのインストール

`install_dependency.sh` を実行するか、手動で必要なパッケージをインストールしてください

```bash
cd ~/rover-sample-2025/remote
sudo ./install_dependency.sh
```

### ワークスペースのビルド

```bash
# ビルドするワークスペースへ移動
cd ~/rover-sample-2025/remote/remote_ws

# ワークスペースのビルド
# --event-handlers  console_direct+ : ビルド状況の詳細ログを出力します（省略可）
colcon build --event-handlers console_direct+
```

### 環境変数の設定

`rosenv_default.sh` をコピーして `rosenv.sh` を作成してください。

```bash
cd ~/rover-sample-2025/remote
cp rosenv_default.sh rosenv.sh
```

`rosenv.sh` の例:

```bash
export ROS_DOMAIN_ID=1
# Discovery Server を使う場合のみ設定（しない場合 or 分からない場合はコメントアウトすること）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp # DDSの選択
export ROS_DISCOVERY_SERVER=192.167.1.10:11811 # Discovery Server (ローバー) のIPアドレス
export ROS_SUPER_CLIENT=TRUE # スーパークライアント設定（Discovery Server 越しでもトピック一覧を見られるようにする）
```

- `ROS_DOMAIN_ID`: ローバー側と同じ値にしてください
- Discovery Server を使わない場合は `RMW_IMPLEMENTATION/ROS_DISCOVERY_SERVER/ROS_SUPER_CLIENT` を設定しないでください

## 使用方法

- ゲームコントローラ版とキーボード版があります
- 両方を同時に起動しないでください（`/cmd_vel` が競合します）

### ゲームコントローラの起動

```bash
cd ~/rover-sample-2025/remote
./start_gamepad.sh
```

### キーボード（WASD）の起動

```bash
cd ~/rover-sample-2025/remote
./start_wasd-controller.sh
```
