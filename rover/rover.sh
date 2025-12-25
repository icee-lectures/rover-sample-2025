#!/usr/bin/env bash

# スクリプトのディレクトリとワークスペースのパスを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/rover_ws"

# 共通の ROS セットアップを実行（同一シェルで反映させるため source）
source "$SCRIPT_DIR/start_ROS.sh" || {
    echo -e "ROS 環境のセットアップに失敗しました。"
    exit 1
}

# 起動前の待機時間（必要に応じて調整）
sleep 10

# ROS2 デーモンの再起動
ros2 daemon stop
ros2 daemon start

# ノードの起動
ros2 launch rover rover_launch.py
