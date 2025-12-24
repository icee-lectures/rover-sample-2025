#!/usr/bin/env bash

# /camera_encoded/ffmpeg をデコードしてパブリッシュ
ros2 run image_transport republish --ros-args \
  -p in_transport:=ffmpeg \
  -p out_transport:=raw \
  --remap in/ffmpeg:=/camera_encoded/ffmpeg \
  --remap out:=/camera_decoded/image_raw
