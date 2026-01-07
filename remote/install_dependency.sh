#! /usr/bin/env bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "このスクリプトはsudo権限で実行する必要があります。"
  echo "使用方法: sudo $0"
  exit 1
fi

apt update

# パッケージインストール
apt install -y \
  python3-pygame

# ROS2関連パッケージインストール
apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-ffmpeg-image-transport \
  ros-jazzy-ffmpeg-image-transport-tools
