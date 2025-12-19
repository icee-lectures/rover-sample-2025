# ロボット側 ROSノード

## 前提

- Ubuntu24.04, ROS2 Jazzy で動作検証を行っています
- 以下ハードウェアを使用します
  - Raspberry Pi 5
  - Yahboom ROS control Board v3
  - Orbbec Astra Pro Plus
    - 深度が要らなければ普通のWebカメラでも動作可能

## クイックスタート

1. 依存パッケージをインストール
2. `rover_ws` をビルド
3. `rosenv_default.sh` を `rosenv.sh` にコピーして環境変数を設定
4. Discovery Server（必要時）→ Rover ノードを起動

## ディレクトリ・ファイルの説明

### ルートディレクトリ

- `rosenv_default.sh`: ROS2関連の環境変数設定（ROS_DOMAIN_ID, Discovery Serverなど）
  - 編集する場合はコピーし `rosenv.sh` を作成して編集してください
- `rover.sh`: Roverノード起動スクリプト
- `ros-discovery.sh`: Discovery Server 起動スクリプト
- `rover.service`: Rover ノード用 systemd サービスファイル
- `ros-discovery.service`: Discovery Server 用 systemd サービスファイル
- `install_rover_service.sh`: systemd ユーザーサービスとして Rover を登録・削除するスクリプト
- `install_discovery_service.sh`: systemd ユーザーサービスとして Discovery Server を登録・削除するスクリプト
- `start_ROS.sh`: ROS2本体とワークスペースの読み込み（rqt起動や開発時などに使用）

### rover_ws/

ROS 2 ワークスペース

#### rover_ws/src/

- **rover**: ローンチファイルを含むメインパッケージ
  - `rover_launch.py`: 全ノードを起動するローンチファイル
  
- **camera**: カメラとArUcoマーカー検出を担当（C++パッケージ）
  - `camera`: カメラ画像を配信するノード
  - `aruco_detector`: ArUcoマーカーを検出するノード

- **robot_control_board**: Yahboom ROS Control Board v3 を制御（Pythonパッケージ）
  - `driver_node`: モータードライバー、IMU、センサーを制御するノード
    - `Rosmaster_Lib.py`: Yahboom制御ボード用ライブラリ

- **OrbbecSDK_ROS2**: Orbbec社製3Dカメラ用ROS実装
  - 詳細はパッケージ内のREADMEを参照してください

### 便利スクリプト

- `src/clean_ros_ws.sh`: キャッシュ・ビルド生成物のクリーン

```bash
cd rover/src
./clean_ros_ws.sh
```

## 環境変数の設定

`rosenv_default.sh` をコピーして `rosenv.sh` を作成してください。

```bash
cd remote
cp rosenv_default.sh rosenv.sh
```

`rosenv.sh` の例:

```bash
export ROS_DOMAIN_ID=1
# Discovery Server を使う場合のみ設定
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.1.10:11811
export ROS_SUPER_CLIENT=TRUE
```

- `ROS_DOMAIN_ID`: ローバー側とリモート側で同じ値にしてください
- Discovery Serverを使わない場合は `RMW_IMPLEMENTATION`, `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT` をコメントアウト


## 使用準備

### シェルスクリプトの実行権限の確認

入手直後はスクリプトに実行権限が付いていない場合があります

```bash
cd rover
chmod +x rover.sh ros-discovery.sh install_rover_service.sh install_discovery_service.sh
```

### 依存パッケージのインストール

```bash
# ROS 2 Jazzy がインストールされていることを確認
sudo apt update
sudo apt install ros-jazzy-desktop

# Fast-DDS Discovery Server
sudo apt install ros-jazzy-rmw-fastrtps-cpp

# カメラ関連
sudo apt install ros-jazzy-aruco-msgs ros-jazzy-backward-ros ros-jazzy-camera-info-manager \
                 ros-jazzy-compressed-image-transport ros-jazzy-cv-bridge \
                 ros-jazzy-diagnostic-msgs ros-jazzy-diagnostic-updater \
                 ros-jazzy-image-publisher ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
                 ros-jazzy-statistics-msgs \
                 libdw-dev libgflags-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
                 libopencv-dev nlohmann-json3-dev
```

#### Orbbec カメラの udev ルール設定

カメラを認識させるために udev ルールを導入してください

```bash
cd rover/rover_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### ワークスペースのビルド

```bash
# ビルドするワークスペースへ移動
cd rover_ws

# ワークスペースのビルド
# --event-handlers  console_direct+ : ビルド状況の詳細ログを出力します（省略可）
# --cmake-args -DCMAKE_BUILD_TYPE=Release : OrbbecSDK_ROS2がデフォルトでDebugでビルドされるためReleaseを指定しています
colcon build --event-handlers  console_direct+  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 環境変数の設定

`rosenv_default.sh`  をコピーして名前を `rosenv.sh` にしてください

```bash
# rosenv_default.shをコピーしてrosenv.shを作る
cp rosenv_default.sh rosenv.sh
```

`rosenv.sh` の例:

```bash
export ROS_DOMAIN_ID=1
# Discovery Server を使う場合のみ設定（しない場合 or 分からない場合はコメントアウトすること）
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp # DDSの選択
export ROS_DISCOVERY_SERVER=127.0.0.1:11811 # Discovery Server (ローバー) のIPアドレス
export ROS_SUPER_CLIENT=TRUE # スーパークライアント設定（Discovery Server 越しでもトピック一覧を見られるようにする）
```

### systemd サービスの登録（オプション）

- システム起動時に自動的にノードを起動したい場合に設定してください
- 設定する場合はDiscovery ServerサービスとRoverノードサービスを両方有効化してください
  - 起動順序設定がされているため

```bash
# Discovery Server サービスの登録
./install_discovery_service.sh

# Rover ノードサービスの登録
./install_rover_service.sh

# サービスの有効化と起動
systemctl --user enable ros-discovery.service
systemctl --user enable rover.service
systemctl --user start ros-discovery.service
systemctl --user start rover.service

# systemdのユーザーモードが未ログインでも自動起動する設定
loginctl enable-linger $(whoami)
```

サービスを削除する場合：

```bash
./install_discovery_service.sh --remove
./install_rover_service.sh --remove
```

## 使用方法

### 手動起動

#### 1. Discovery Server の起動

```bash
./ros-discovery.sh
```

#### 2. Rover ノードの起動

```bash
./rover.sh
```

### systemd サービス経由での起動

サービスを登録済みの場合：

```bash
# 起動
systemctl --user start ros-discovery.service
systemctl --user start rover.service

# 状態確認
systemctl --user status ros-discovery.service
systemctl --user status rover.service

# ログ確認
journalctl --user -u ros-discovery.service -f
journalctl --user -u rover.service -f

# 停止
systemctl --user stop rover.service
systemctl --user stop ros-discovery.service
```
