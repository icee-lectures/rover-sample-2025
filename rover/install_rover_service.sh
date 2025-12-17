#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME=rover.service
TEMPLATE=rover.service
TARGET_DIR="$HOME/.config/systemd/user"
TARGET_SERVICE="$TARGET_DIR/$SERVICE_NAME"

# このスクリプト自身のあるディレクトリ
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  echo "Usage:"
  echo "  $0           # install service"
  echo "  $0 --remove  # remove service"
}

# --- オプション解析 ---
if [[ $# -gt 1 ]]; then
  usage
  exit 1
fi

if [[ $# -eq 1 && "$1" != "--remove" ]]; then
  usage
  exit 1
fi

# --- remove 処理 ---
if [[ $# -eq 1 && "$1" == "--remove" ]]; then
  echo "[remove] stop & disable $SERVICE_NAME"

  systemctl --user stop "$SERVICE_NAME" 2>/dev/null || true
  systemctl --user disable "$SERVICE_NAME" 2>/dev/null || true

  if [[ -f "$TARGET_SERVICE" ]]; then
    rm "$TARGET_SERVICE"
    echo "[remove] removed $TARGET_SERVICE"
  else
    echo "[remove] service file not found"
  fi

  systemctl --user daemon-reload
  echo "[remove] done"
  exit 0
fi

# --- install 処理 ---
echo "[install] SCRIPT_DIR = $SCRIPT_DIR"
mkdir -p "$TARGET_DIR"

sed "s|__SCRIPT_DIR__|$SCRIPT_DIR|g" \
  "$TEMPLATE" > "$TARGET_SERVICE"

echo "[install] installed to $TARGET_SERVICE"

systemctl --user daemon-reload

echo "[install] done"
echo "次に以下を実行してください:"
echo "  systemctl --user enable $SERVICE_NAME"
echo "  systemctl --user start  $SERVICE_NAME"
