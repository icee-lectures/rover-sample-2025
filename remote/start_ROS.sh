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

# オプション解析 (--build で強制ビルド)
FORCE_BUILD=0
for arg in "$@"; do
    case "$arg" in
        --build)
            FORCE_BUILD=1
            ;;
    esac
done

# ワークスペースのセットアップ
if [ -f "$WS_DIR/install/setup.bash" ] && [ "$FORCE_BUILD" -ne 1 ]; then
    source "$WS_DIR/install/setup.bash"
else
    if [ "$FORCE_BUILD" -eq 1 ]; then
        echo -e "--build 指定のため colcon build を実行します: $WS_DIR"
    else
        echo -e "ワークスペースのセットアップファイルが見つかりません: $WS_DIR/install/setup.bash"
        echo -e "colcon build を実行します: $WS_DIR"
    fi
    if command -v colcon >/dev/null 2>&1; then
        pushd "$WS_DIR" >/dev/null
        colcon build --event-handlers console_direct+
        BUILD_EXIT_CODE=$?
        popd >/dev/null
        if [ $BUILD_EXIT_CODE -eq 0 ] && [ -f "$WS_DIR/install/setup.bash" ]; then
            source "$WS_DIR/install/setup.bash"
        else
            echo -e "ビルドに失敗したかセットアップファイルが見つかりません。"
            return 1 2>/dev/null || exit 1
        fi
    else
        echo -e "colcon が見つかりません。ROS2 Jazzy が正しくインストールされているか確認してください。"
        return 1 2>/dev/null || exit 1
    fi
fi
