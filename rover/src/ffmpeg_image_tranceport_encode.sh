#!/usr/bin/env bash

# /camera/color/image_raw をエンコードしてパブリッシュ
ros2 run image_transport republish --ros-args \
  -p in_transport:=raw \
  -p out_transport:=ffmpeg \
  --remap in:=/camera/color/image_raw \
  --remap out/ffmpeg:=/camera_encoded/ffmpeg \
  -p out.ffmpeg.encoder:=libx264 \
  -p out.ffmpeg.bit_rate:=200000 \
  -p out.ffmpeg.qmax:=30 \
  -p out.ffmpeg.gop_size:=30 \
  -p out.ffmpeg.pixel_format:=yuv420p \
  -p out.ffmpeg.encoder_av_options:="preset:ultrafast,profile:baseline,tune:zerolatency,crf:45"
