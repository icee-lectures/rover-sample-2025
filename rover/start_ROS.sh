#! /usr/bin/env bash

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS環境変数を設定
if [ -f "$SCRIPT_DIR/rosenv.sh" ]; then
    source $SCRIPT_DIR/rosenv.sh
else
    source $SCRIPT_DIR/rosenv_default.sh
fi

# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash
