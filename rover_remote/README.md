# リモート制御端末（PC）側 ROSノード

## 前提

- Ubuntu24.04, ROS2 Jazzy で動作検証を行っています
- ゲームコントローラを使う場合は、物理マシンでUbuntuを実行する必要があります
  - 仮想環境でゲームコントローラを認識できないため

## ディレクトリ・ファイルの説明

### ルートディレクトリ

- `rosenv.sh`: ROS 2 環境変数の設定スクリプト
- `start_ROS.sh`: ROS 2 起動スクリプト
- `start_gamepad.sh`: ゲームコントローラで制御するノード起動スクリプト

### remote_ws/

ROS 2 ワークスペース

#### remote_ws/src/

- **gamepad**: ゲームコントローラ入力を処理（Pythonパッケージ）
  - `gamepad_publisher.py`: ゲームコントローラの入力を発行するノード
  - `joy_to_cmd_vel.py`: ゲームコントローラ入力を速度指令に変換するノード
  - `gamepad.launch.py`: ゲームコントローラ関連ノード起動用ローンチファイル

### src/

ワークスペースに入れてパッケージ化していないROS 2 ノードやその他のプログラム

- `ros2_gamepad_pub.py`: ゲームコントローラ入力発行スクリプト
- `ros2_joy_to_cmd_vel.py`: 速度指令変換スクリプト

## 使用準備

### 0. シェルスクリプトの実行権限付与

入手直後はスクリプトに実行権限が付いていない場合があります。最初に権限を付与してください。

```bash
cd rover_remote
chmod +x start_ROS.sh start_gamepad.sh
```

### 1. 依存パッケージのインストール

```bash
# ROS 2 Jazzy がインストールされていることを確認
sudo apt update
sudo apt install ros-jazzy-desktop

# ゲームコントローラ関連
sudo apt install ros-jazzy-joy
```

### 2. ワークスペースのビルド

```bash
cd remote_ws
colcon build
```

### 3. 環境変数の設定

`rosenv_default.sh`  をコピーして名前を `rosenv.sh` にしてください 

```bash
cp rosenv_default.sh rosenv.sh
```

以下の設定があります：
- `ROS_DOMAIN_ID`: ROSドメインID
- 遠隔制御に使う環境変数（しない場合 or 分からない場合はコメントアウトすること）
    - `RMW_IMPLEMENTATION`: DDSの選択
    - `ROS_DISCOVERY_SERVER`: Discovery Server のアドレス
    - `ROS_SUPER_CLIENT`: スーパークライアント設定

必要に応じて編集してください。

## 使用方法

### ゲームコントローラノードの起動

```bash
./start_gamepad.sh
```

または直接：
```bash
source rosenv.sh
source /opt/ros/jazzy/setup.bash
source remote_ws/install/setup.bash
ros2 launch gamepad gamepad.launch.py
```
