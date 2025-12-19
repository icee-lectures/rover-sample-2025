# rover_ws マニュアル

## 概要

- ROS 2 ワークスペース。カメラから JPEG 圧縮画像を配信する `camera` と、Rostmaster ベースの車両制御ノード `robot_control_board` を含む。
- 主なトピック: `camera/image_raw/compressed`, `camera/aruco_debug/compressed`, `/cmd_vel`, `/imu/data_raw`, `/imu/mag`, `/vel_raw`, `/joint_states` など。

## 前提環境

- ROS 2 Jazzy (`colcon` と `rosdep` が利用可能な状態)
- 依存ライブラリ: GStreamer 1.0, gstreamer-app/base, OpenCV (ArUco 含む), v4l2 カメラデバイス `/dev/video0`。
- Python 3.10 以上推奨。

## パッケージ構成

- `src/camera` (C++/ament_cmake)
  - ノード `camera`: GStreamer パイプラインで `/dev/video0` の JPEG をそのまま `camera/image_raw/compressed` へ配信。
  - ノード `aruco_detector`: JPEG を OpenCV でデコードし ArUco マーカー検出。デバッグ画像を `camera/aruco_debug/compressed` へ（パラメータで無効化可）。
- `src/robot_control_board` (Python/ament_python)
  - ノード `driver_node`: Rosmaster ボード制御。`cmd_vel` などから駆動、IMU・磁気・バッテリ・ジョイントステートを配信。

## セットアップ

1. ROS 環境を読み込む

    ```bash
    source /opt/ros/<distro>/setup.bash
    ```

2. 依存解決 (必要に応じて)

    ```bash
    rosdep install -r --from-paths src --ignore-src -y
    ```

3. ビルド

    ```bash
    colcon build --symlink-install
    ```

4. オーバーレイ読み込み

    ```bash
    source install/setup.bash
    ```

## 実行手順

- カメラ配信ノード

    ```bash
    ros2 run camera camera
    ```

- デフォルトで `/dev/video0` を 1280x720 30fps JPEG で配信。

- ArUco 検出ノード
  
    ```bash
    ros2 run camera aruco_detector --ros-args \
        -p dictionary_id:=0 \
        -p publish_debug_image:=true
    ```

- `dictionary_id` は OpenCV 定義の ID (例: 0 は `DICT_4X4_50`)。
- `publish_debug_image` を false にするとデバッグ画像を止めて CPU 使用率を抑える。

- 車両制御ノード

    ```bash
    ros2 run robot_control_board driver_node --ros-args \
        -p car_type:=X1 \
        -p imu_link:=imu_link \
        -p Prefix:="" \
        -p xlinear_limit:=1.0 \
        -p ylinear_limit:=1.0 \
        -p angular_limit:=1.0 \
        -p nav_use_rotvel:=false
    ```

- `cmd_vel` を購読し、RGB ライト制御は `RGBLight`、ブザー制御は `Buzzer` (Bool) で受信。
- IMU/磁気/バッテリ/ジョイント状態をそれぞれ `/imu/data_raw`, `/imu/mag`, `voltage`, `joint_states` に配信。

## 主なトピック

- `camera/image_raw/compressed` (sensor_msgs/CompressedImage): JPEG カメラ画像。
- `camera/aruco_debug/compressed` (sensor_msgs/CompressedImage): 検出結果の描画付き JPEG (オプション)。
- `cmd_vel` (geometry_msgs/Twist): 車両速度指令。
- `RGBLight` (std_msgs/Int32), `Buzzer` (std_msgs/Bool): ライト・ブザー制御。
- `/imu/data_raw` (sensor_msgs/Imu), `/imu/mag` (sensor_msgs/MagneticField), `voltage` (std_msgs/Float32), `edition` (std_msgs/Float32), `joint_states` (sensor_msgs/JointState), `/vel_raw` (geometry_msgs/Twist)。

## トラブルシュートのヒント

- カメラが認識されない場合: `v4l2-ctl -d /dev/video0 --all` で状態確認し、`test/ros2_camera_pub.sh` で動作確認。
- GStreamer/OpenCV が見つからない場合: 開発パッケージ (例: `libgstreamer1.0-dev`, `libopencv-dev`) をインストールし、再ビルド。
- パーミッション問題: `/dev/video0` へのアクセス権を確認し、必要なら `udev` 設定または `sudo usermod -aG video <ユーザ>`。

## 開発メモ

- `camera` は GStreamer で JPEG をデコードせず配信する構成のため CPU 使用率が低い。OpenCV で処理するノードを増やす場合は QoS を `SensorDataQoS` 相当で揃えると取りこぼしを減らせる。
- `aruco_detector` の辞書やパラメータは launch ファイル未提供のため、必要に応じて `--ros-args` で指定する。
