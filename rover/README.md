# ロボット側 ROSノード

## 前提

- Ubuntu24.04, ROS2 Jazzy で動作検証を行っています
- 以下ハードウェアを使用します
    - Raspberry Pi 5
    - Yahboom ROS control Board v3

## ディレクトリ・ファイルの説明

### ルートディレクトリ

- `rosenv.sh`: ROS 2 環境変数の設定スクリプト（ROS_DOMAIN_ID, Discovery Server設定など）
- `rover.sh`: Roverノード起動スクリプト
- `ros-discovery.sh`: Fast-DDS Discovery Server 起動スクリプト
- `install_rover_service.sh`: systemd ユーザーサービスとして Rover を登録・削除するスクリプト
- `install_discovery_service.sh`: systemd ユーザーサービスとして Discovery Server を登録・削除するスクリプト
- `rover.service`: Rover ノード用 systemd サービスファイル
- `ros-discovery.service`: Discovery Server 用 systemd サービスファイル

### rover_ws/

ROS 2 ワークスペース

#### rover_ws/src/

- **rover**: ローンチファイルを含むメインパッケージ
  - `rover_launch.py`: 全ノードを起動するローンチファイル
  
- **camera_main**: カメラとArUcoマーカー検出を担当（C++パッケージ）
  - `camera`: カメラ画像を配信するノード
  - `aruco_detector`: ArUcoマーカーを検出するノード
  - 依存: `rclcpp`, `sensor_msgs`, `aruco_msgs`, `opencv2`

- **robot_control_board**: Yahboom ROS Control Board v3 を制御（Pythonパッケージ）
  - `driver_node`: モータードライバー、IMU、センサーを制御するノード
  - `Rosmaster_Lib.py`: Yahboom制御ボード用ライブラリ
  - トピック:
    - 購読: `/cmd_vel`, `/RGBLight`, `/Buzzer`
    - 発行: `/voltage`, `/joint_states`, `/vel_raw`, `/imu/data_raw`, `/imu/mag`

## 使用準備

### 1. 依存パッケージのインストール

```bash
# ROS 2 Jazzy がインストールされていることを確認
sudo apt update
sudo apt install ros-jazzy-desktop

# Fast-DDS Discovery Server
sudo apt install ros-jazzy-rmw-fastrtps-cpp

# カメラとOpenCV関連
sudo apt install ros-jazzy-cv-bridge ros-jazzy-image-transport
sudo apt install libopencv-dev

# ArUco関連
sudo apt install ros-jazzy-aruco-msgs
```

### 2. ワークスペースのビルド

```bash
cd rover_ws
colcon build
```

### 3. 環境変数の設定

`rosenv.sh` に以下の設定があります：
- `ROS_DOMAIN_ID`: ROSドメインID
- 遠隔制御に使う環境変数（しない場合 or 分からない場合はコメントアウトすること）
    - `RMW_IMPLEMENTATION`: DDSの選択
    - `ROS_DISCOVERY_SERVER`: Discovery Server のアドレス
    - `ROS_SUPER_CLIENT`: スーパークライアント設定（Discovery Server 越しでもトピック一覧を見られるようにする）

必要に応じて編集してください。

### 4. systemd サービスの登録（オプション）

システム起動時に自動的にノードを起動したい場合：

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

または直接：
```bash
source rosenv.sh
source /opt/ros/jazzy/setup.bash
fastdds discovery -i 0 -p 11811
```

#### 2. Rover ノードの起動

別のターミナルで：
```bash
./rover.sh
```

または直接：
```bash
source rosenv.sh
source /opt/ros/jazzy/setup.bash
source rover_ws/install/setup.bash
ros2 launch rover rover_launch.py
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

### ノードの確認

```bash
source rosenv.sh
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
```

### トピックのテスト

Roverを動かすテスト：
```bash
source rosenv.sh
source /opt/ros/jazzy/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```


