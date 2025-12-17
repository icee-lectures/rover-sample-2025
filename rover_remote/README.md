# リモート制御端末（PC）側 ROSノード

## 前提

- Ubuntu24.04, ROS2 Jazzy で動作検証を行っています
- ゲームコントローラを使う場合は、物理マシンでUbuntuを実行する必要があります
  - 仮想環境でゲームコントローラを認識できないため

## ディレクトリ・ファイルの説明

- **remote_ws/** : ROS2ワークスペース
- **src/** : ワークスペースに入れてパッケージ化していないROS2ノードやその他のプログラム
- **rosenv.sh** : ROS2の環境変数設定
- **start_ROS** : ROS2の立ち上げ
- **start_gamepad.sh** : ゲームコントローラで制御するROS2ノードを立ち上げ

## 使用準備

- 起動スクリプトに実行権限を付加してください
    ``` bash
    chmod +x start_ROS.sh start_gamepad.sh
    ```
- 環境変数設定(`rosenv.sh`)を編集してください
  - `ROS_DOMAIN_ID` を各グループで適切に設定しないと混信します
  - `ROS_DISCOVERY_SERVER` や `ROS_SUPER_CLIENT` は同じサブネットの外から制御する場合のみ必要です
    - わからない場合はとりあえずコメントアウトしてください
- remote_ws のビルドを行ってください
    ``` bash
    cd remote_ws
    colcon build
    ```

## 使用方法

1. ゲームコントローラをPCに接続
   - 仮想OSだと認識できないので、物理マシンでUbuntuを実行する必要があります 
2. `start_ROS.sh` を実行する
    ``` bash
    ./start_ROS.sh
    ```
3. `start_gamepad.sh` を実行する
    ``` bash
    ./start_gamepad.sh
    ```


