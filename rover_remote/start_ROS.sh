#! /usr/bin/env bash

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS環境変数を設定
source $SCRIPT_DIR/rosenv.sh

# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash

# ROS2デーモンの再起動
ros2 daemon stop
ros2 daemon start

