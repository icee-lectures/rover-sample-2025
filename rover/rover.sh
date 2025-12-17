#!/usr/bin/env bash

# スクリプトのディレクトリとワークスペースのパスを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/rover_ws"

# 環境変数の設定
if [ -f "$SCRIPT_DIR/rosenv.sh" ]; then
    source $SCRIPT_DIR/rosenv.sh
else
    source $SCRIPT_DIR/rosenv_default.sh
fi

# Discovery Server が立ち上がるまで少し待つ
sleep 2

# ROS 2 環境のセットアップ
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# ノードの起動
ros2 launch rover rover_launch.py
