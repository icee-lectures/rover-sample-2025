# rover-sample-2025

学習用ローバーのサンプルプロジェクト

## 概要

このプロジェクトは、ROS 2 Jazzy を使用したローバー（探査車）の制御システムです。Raspberry Pi 5 と Yahboom ROS Control Board v3 を使用して、カメラ映像配信、ArUcoマーカー検出、モーター制御、センサー取得、ゲームパッド操作などの機能を提供します。

## 主な機能

- **カメラ映像配信**: GStreamerを使用したJPEG圧縮画像の低遅延配信
- **ArUcoマーカー検出**: OpenCVを使用したマーカー認識
- **ローバー制御**: cmd_velトピックによるロボット駆動制御
- **センサー統合**: IMU、磁気センサー、バッテリー電圧の取得
- **ゲームパッド操作**: リモートPCからのゲームコントローラーによる遠隔操作
- **Discovery Server**: （オプション）Fast-DDS Discovery Serverによる遠隔ネットワーク通信

## システム構成

```
rover-sample-2025/
├── rover/              # ローバー本体（Raspberry Pi）のROSノード
│   ├── rover_ws/       # ROS 2 ワークスペース
│   │   └── src/
│   │       ├── rover/                # メインパッケージ（ローンチファイル）
│   │       ├── camera_main/          # カメラとArUco検出（C++）
│   │       └── robot_control_board/  # モーター・センサー制御（Python）
│   ├── rosenv.sh                     # 環境変数設定
│   ├── rover.sh                      # ローバー起動スクリプト
│   ├── ros-discovery.sh              # Discovery Server起動スクリプト
│   └── *.service                     # systemdサービスファイル
│
└── rover_remote/       # リモート制御端末（PC）のROSノード
    ├── remote_ws/      # ROS 2 ワークスペース
    │   └── src/
    │       └── gamepad/              # ゲームパッド制御パッケージ
    ├── rosenv.sh                     # 環境変数設定
    ├── start_ROS.sh                  # ROS起動スクリプト
    └── start_gamepad.sh              # ゲームパッド起動スクリプト
```

## 前提条件

### ローバー側（Raspberry Pi 5）
- Ubuntu 24.04
- ROS 2 Jazzy
- Yahboom ROS Control Board v3
- カメラモジュール（/dev/video0）

### リモート制御側（PC）
- Ubuntu 24.04（物理マシン推奨）
- ROS 2 Jazzy
- ゲームコントローラー（USBまたはBluetooth接続）

## セットアップ

### 1. ローバー側のセットアップ

詳細は [rover/README.md](rover/README.md) を参照してください。

```bash
# 依存パッケージのインストール
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-rmw-fastrtps-cpp \
                 ros-jazzy-cv-bridge ros-jazzy-image-transport \
                 ros-jazzy-aruco-msgs libopencv-dev libgstreamer1.0-dev

# ワークスペースのビルド
cd rover/rover_ws
colcon build
source install/setup.bash

# 環境変数の設定（必要に応じて編集）
cd ..
vim rosenv.sh

# systemdサービスの登録（オプション）
./install_discovery_service.sh
./install_rover_service.sh
systemctl --user enable ros-discovery.service rover.service
systemctl --user start ros-discovery.service rover.service
```

### 2. リモート制御側のセットアップ

詳細は [rover_remote/README.md](rover_remote/README.md) を参照してください。

```bash
# 起動スクリプトに実行権限を付与
cd rover_remote
chmod +x start_ROS.sh start_gamepad.sh

# 環境変数の設定（必要に応じて編集）
vim rosenv.sh

# ワークスペースのビルド
cd remote_ws
colcon build
source install/setup.bash
```

## 使用方法

### ローバー側の起動

```bash
# 手動起動の場合
cd rover
./ros-discovery.sh  # Discovery Server（別ターミナル）
./rover.sh          # ローバーノード起動

# systemdサービスを使用する場合は自動起動されます
```

### リモート制御側の起動

```bash
# ゲームコントローラーをPCに接続

# ROS環境の起動
./start_ROS.sh

# ゲームパッド制御の起動（別ターミナル）
./start_gamepad.sh
```

## 主なROSトピック

### ローバーが配信するトピック
- `camera/image_raw/compressed`: カメラ映像（JPEG圧縮）
- `camera/aruco_debug/compressed`: ArUco検出結果の描画付き映像
- `/imu/data_raw`: IMUデータ
- `/imu/mag`: 磁気センサーデータ
- `/voltage`: バッテリー電圧
- `/joint_states`: ジョイント状態
- `/vel_raw`: 生の速度データ

### ローバーが購読するトピック
- `/cmd_vel`: 速度指令（Twist）
- `/RGBLight`: LEDライト制御
- `/Buzzer`: ブザー制御

## トラブルシューティング

### カメラが認識されない
```bash
# カメラデバイスの確認
v4l2-ctl -d /dev/video0 --all

# パーミッションの設定
sudo usermod -aG video $USER
```

### ゲームコントローラーが認識されない
- 仮想環境（VM）ではUSBデバイスの認識に問題がある場合があります
- 物理マシンでUbuntuを実行してください

### ネットワーク通信ができない
- `rosenv.sh`の`ROS_DOMAIN_ID`が一致しているか確認
- 同じサブネット外から制御する場合は`ROS_DISCOVERY_SERVER`の設定を確認
- ファイアウォール設定を確認

## ライセンス

このプロジェクトのライセンスについては [LICENSE](LICENSE) を参照してください。

## 参考リンク

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
