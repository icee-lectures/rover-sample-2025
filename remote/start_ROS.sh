#! /usr/bin/env bash

# スクリプトのディレクトリを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/remote_ws"

# ROS環境変数を設定
if [ -f "$SCRIPT_DIR/rosenv.sh" ]; then
    source $SCRIPT_DIR/rosenv.sh
else
    source $SCRIPT_DIR/rosenv_default.sh
fi

# ROS2のセットアップ
source /opt/ros/jazzy/setup.bash

# ワークスペースのセットアップ
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "ワークスペースのセットアップファイルが見つかりません: $WS_DIR/install/setup.bash"
fi
