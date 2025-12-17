#!/bin/bash

v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --list-formats-ext

ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -p video_device:=/dev/video0 \
             -p image_size:="[1280,960]" \
             -p pixel_format:="YUYV" \
             -p output_encoding:="rgb8" \
             -p qos_reliability:="best_effort" \
             -p qos_history:="keep_last" \
             -p qos_depth:=5 \
             -p power_line_frequency:=2 \
             -p auto_exposure:=3 \
             -p exposure_time_absolute:=10 \
             -p gain:=0
