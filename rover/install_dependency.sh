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
  libdw-dev \
  libgflags-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libopencv-dev \
  nlohmann-json3-dev

# ROS2関連パッケージインストール
apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-aruco-msgs \
  ros-jazzy-backward-ros \
  ros-jazzy-camera-info-manager \
  ros-jazzy-compressed-image-transport \
  ros-jazzy-cv-bridge \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-diagnostic-updater \
  ros-jazzy-image-publisher \
  ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-topic-tools \
  ros-jazzy-ffmpeg-image-transport \
  ros-jazzy-ffmpeg-image-transport-tools \
  ros-jazzy-statistics-msgs \
  ros-jazzy-rmw-fastrtps-cpp

# OrbbecSDKの入手とインストール
pushd ~ >/dev/null
wget https://github.com/orbbec/OrbbecSDK/releases/download/v1.10.18/OrbbecSDK_v1.10.18_arm64.deb
dpkg -i OrbbecSDK_v1.10.18_arm64.deb
popd >/dev/null
