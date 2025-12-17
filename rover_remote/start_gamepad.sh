#! /usr/bin/env bash

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS環境変数を設定
source $SCRIPT_DIR/rosenv.sh

# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash

# remote_wsのセットアップ
source $SCRIPT_DIR/remote_ws/install/setup.bash

# ゲームパッドノードの起動
ros2 launch gamepad gamepad.launch.py
