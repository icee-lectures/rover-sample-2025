#!/usr/bin/env bash

# 変更する場合はこのファイルをコピーし rosenv.sh という名前で保存してください

export ROS_DOMAIN_ID=0 # ROS2のドメインID (各チームのIDに合わせて変更すること)

# 遠隔地のROS2ノードと通信する場合（操作PCがインターネット越しなど）は、以下の環境変数を設定してください
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export ROS_DISCOVERY_SERVER=127.0.0.1:11811
# export ROS_SUPER_CLIENT=TRUE
