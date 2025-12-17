#! /usr/bin/env bash

export ROS_DOMAIN_ID=0 # ROS2のドメインID (各チームのIDに合わせて変更すること)

# 遠隔地のROS2ノードと通信する場合（ローバーをインターネット越しに操作するなど）は、以下の環境変数を設定してください
# export ROS_DISCOVERY_SERVER=xxx.xxx.xxx.xxx:11811 # ROS2ディスカバリサーバのアドレス
# export ROS_SUPER_CLIENT=TRUE # ROS2ディスカバリサーバ越しでも topic が見えるようにする
