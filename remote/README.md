# リモート制御端末（PC）側 ROSノード

## 前提

- Ubuntu 24.04 / ROS 2 Jazzy で動作確認
- ゲームコントローラ利用時は物理マシンでの実行を推奨（仮想環境では認識しない場合あり）
- キーボード入力は pygame でウィンドウを開くためディスプレイ環境が必要

## クイックスタート

1. 依存パッケージをインストール
2. `remote_ws` をビルド
3. `rosenv_default.sh` を `rosenv.sh` にコピーして必要に応じて編集
4. ゲームパッド（またはキーボード）ノードを起動

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
cd remote
chmod +x start_gamepad.sh start_wasd-controller.sh
```

### 依存パッケージのインストール

```bash
# ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# pygame（ゲームパッド/キーボード読み取りで使用）
sudo apt install python3-pygame
```

### ワークスペースのビルド

```bash
cd remote/remote_ws
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 環境変数の設定

`rosenv_default.sh` をコピーして `rosenv.sh` を作成してください。

```bash
cd remote
cp rosenv_default.sh rosenv.sh
```

`rosenv.sh` の例:

```bash
export ROS_DOMAIN_ID=1
# Discovery Server を使う場合のみ設定（Fast DDS を使用）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.1.10:11811
export ROS_SUPER_CLIENT=TRUE
```

- `ROS_DOMAIN_ID`: ローバー側と同じ値にしてください
- Discovery Server を使わない場合は `RMW_IMPLEMENTATION/ROS_DISCOVERY_SERVER/ROS_SUPER_CLIENT` を設定しないでください

## 使用方法

- ゲームコントローラ版とキーボード版があります
- 両方を同時に起動しないでください（`/cmd_vel` が競合します）

### ゲームコントローラの起動

```bash
cd remote
./start_gamepad.sh
```

### キーボード（WASD）の起動

```bash
cd remote
./start_wasd-controller.sh
```
