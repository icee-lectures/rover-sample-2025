#! /usr/bin/env bash

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 共通の ROS セットアップを実行（同一シェルで反映させるため source）
source "$SCRIPT_DIR/start_ROS.sh" || {
    echo -e "ROS 環境のセットアップに失敗しました。"
    exit 1
}

# WASDコントローラーノードの起動
ros2 launch keyboard wasd_controller.launch.py
