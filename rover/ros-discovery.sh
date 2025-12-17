#!/usr/bin/env bash
set -euo pipefail

echo "[startROS] start: $(date)"

# スクリプトのディレクトリパスを取得
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 環境変数の設定
if [ -f "$SCRIPT_DIR/rosenv.sh" ]; then
    source $SCRIPT_DIR/rosenv.sh
else
    source $SCRIPT_DIR/rosenv_default.sh
fi

# ROS2読み込み
set +u
source /opt/ros/jazzy/setup.bash
set -u

# ネットワーク接続の待機
sleep 10

# Discovery Server 起動
fastdds discovery -i 0 -p 11811
