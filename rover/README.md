# ロボット側 ROSノード

## 前提

- Ubuntu24.04, ROS2 Jazzy で動作検証を行っています
- 以下ハードウェアを使用します
  - Raspberry Pi 5
  - Yahboom ROS control Board v3
  - Orbbec Astra Pro Plus
    - 深度が要らなければ普通のWebカメラでも動作可能

## クイックガイド

### インストール

1. 依存パッケージ等をインストール: `sudo ~/rover-sample-2025/rover/install_dependency.sh` など
2. `rover_ws` をビルド: `cd ~/rover-sample-2025/rover/rover_ws && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release`
3. ROS2環境変数を設定: `rosenv_default.sh` を `rosenv.sh` にコピーして編集
4. 自動起動設定: `~/rover-sample-2025/rover/install_rover_service.sh`

### 通常使用

自動起動設定をしている場合は電源ONで自動起動します

#### 実行中のシェルでROS2を読み込む場合

ROS2読込スクリプトを実行: `source ~/rover-sample-2025/rover/start_ROS.sh`

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
  - `camera_fallback`: LTE回線越しなど通信帯域が限られた環境でも使える低ビットレート画像を配信するノード
  - `aruco_detector`: ArUcoマーカーを検出するノード

- **robot_control_board**: Yahboom ROS Control Board v3 を制御（Pythonパッケージ）
  - `driver_node`: モータードライバー、IMU、センサーを制御するノード
    - `Rosmaster_Lib.py`: Yahboom制御ボード用ライブラリ

- **OrbbecSDK_ROS2**: Orbbec社製3Dカメラ用ROS実装
  - 詳細はパッケージ内のREADMEを参照してください

### doc/

参考資料やマニュアル

### src/

テストやメンテナンス用スクリプト

- `src/clean_ros_ws.sh`: キャッシュ・ビルド生成物のクリーン
- `src/yahboom_rosboard_v3_driver.py`: Yahboom ROS robot control board の独自修正版ドライバ
  - 単体で実行 (`python3 src/yahboom_rosboard_v3_driver.py`) すると現在のセンサ情報の確認ができる
- `src/test_list-orbbec-cam.sh`: Orbbec社のカメラが接続されているかどうか確認する
- `rover/src/ros2_v4l2_camera_pub.sh`: [v4l2_camera](https://docs.ros.org/en/jazzy/p/v4l2_camera/)でWebカメラ等の映像をとりあえずパブリッシュする

```bash
cd rover/src
./clean_ros_ws.sh
```

## 使用準備

### シェルスクリプトの実行権限の確認

入手直後はスクリプトに実行権限が付いていない場合があります

```bash
cd ~/rover-sample-2025/rover
chmod +x rover.sh ros-discovery.sh install_rover_service.sh install_discovery_service.sh install_dependency.sh
```

### 依存パッケージのインストール

`install_dependency.sh` を実行するか、手動で必要なパッケージをインストールしてください

```bash
cd ~/rover-sample-2025/rover
sudo ./install_dependency.sh
```

#### Orbbec カメラの udev ルール設定

カメラを認識させるために udev ルールを導入してください

```bash
cd ~/rover-sample-2025/rover/rover_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### ワークスペースのビルド

#### 全体のビルド

```bash
# ビルドするワークスペースへ移動
cd ~/rover-sample-2025/rover/rover_ws

# ワークスペースのビルド
# --event-handlers  console_direct+ : ビルド状況の詳細ログを出力します（省略可）
# --cmake-args -DCMAKE_BUILD_TYPE=Release : OrbbecSDK_ROS2がデフォルトでDebugでビルドされるためReleaseを指定しています
colcon build --event-handlers  console_direct+  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 特定パッケージのビルド

例: camera パッケージのビルド

```bash
# ビルドするワークスペースへ移動
cd ~/rover-sample-2025/rover/rover_ws

# 普通のビルド
colcon build --event-handlers  console_direct+ --packages-select camera

# クリーンビルド
# CMakeLists.txt や pakcage.xml を編集した場合はクリーンビルドしたほうが無難
colcon build --event-handlers  console_direct+ --packages-select camera --cmake-clean-cache
```

### 環境変数の設定

`rosenv_default.sh`  をコピーして名前を `rosenv.sh` にしてください

```bash
cd ~/rover-sample-2025/rover
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

- `ROS_DOMAIN_ID`: ローバー側とリモート側で同じ値にしてください
- Discovery Serverを使わない場合は `RMW_IMPLEMENTATION`, `ROS_DISCOVERY_SERVER`, `ROS_SUPER_CLIENT` をコメントアウト

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

## トピック

### パブリッシュするトピック

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/aruco/markers` | `aruco_msgs/msg/MarkerArray` | 検出されたArUcoマーカーの情報 |
| `/aruco/debug/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | ArUco検出デバッグ画像（圧縮） |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | カラーカメラの生画像 |
| `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | カラーカメラの圧縮画像 |
| `/camera/color/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | カラーカメラの圧縮深度画像 |
| `/camera/color/image_raw/theora` | `theora_image_transport/msg/Packet` | カラーカメラのTheora圧縮画像 |
| `/camera/color/image_raw/zstd` | `sensor_msgs/msg/CompressedImage` | カラーカメラのZstd圧縮画像 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | カラーカメラのキャリブレーション情報 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度カメラの生画像 |
| `/camera/depth/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 深度カメラの圧縮画像 |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 深度カメラの圧縮深度画像 |
| `/camera/depth/image_raw/theora` | `theora_image_transport/msg/Packet` | 深度カメラのTheora圧縮画像 |
| `/camera/depth/image_raw/zstd` | `sensor_msgs/msg/CompressedImage` | 深度カメラのZstd圧縮画像 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度カメラのキャリブレーション情報 |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | 深度カメラのポイントクラウド |
| `/camera/depth_filter_status` | `std_msgs/msg/String` | 深度フィルターの状態 |
| `/camera/depth_to_color` | `orbbec_camera_msgs/msg/Extrinsics` | 深度・カラーカメラ間の変換行列 |
| `/camera/depth_to_ir` | `orbbec_camera_msgs/msg/Extrinsics` | 深度・赤外線カメラ間の変換行列 |
| `/camera/ir/image_raw` | `sensor_msgs/msg/Image` | 赤外線カメラの生画像 |
| `/camera/ir/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 赤外線カメラの圧縮画像 |
| `/camera/ir/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | 赤外線カメラの圧縮深度画像 |
| `/camera/ir/image_raw/theora` | `theora_image_transport/msg/Packet` | 赤外線カメラのTheora圧縮画像 |
| `/camera/ir/image_raw/zstd` | `sensor_msgs/msg/CompressedImage` | 赤外線カメラのZstd圧縮画像 |
| `/camera/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | 赤外線カメラのキャリブレーション情報 |
| `/camera_fallback/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 低ビットレートカラー画像（圧縮） |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | IMUセンサーのデータ |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | 磁気センサーのデータ |
| `/joint_states` | `sensor_msgs/msg/JointState` | ジョイント（モーター）の状態 |
| `/tf` | `tf2_msgs/msg/TFMessage` | トランスフォーム情報（動的） |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | トランスフォーム情報（静的） |
| `/voltage` | `std_msgs/msg/Float32` | 電源電圧 |

### サブスクライブするトピック

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 速度指令（線速度・角速度） |
| `/vel_raw` | `geometry_msgs/msg/Twist` | 生の速度指令 |
| `/Buzzer` | `std_msgs/msg/Bool` | ブザー制御（True: 鳴動、False: 停止） |
| `/RGBLight` | `std_msgs/msg/Int32` | RGB LED制御 |
| `/pwmservo` | `std_msgs/msg/Float32MultiArray` | PWM サーボ制御 (4ch) |
