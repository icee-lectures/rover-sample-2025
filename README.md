# rover-sample-2025

学習用ローバーのサンプルプロジェクト

## 概要

このプロジェクトは、ROS 2 Jazzy を使用したローバー（探査車）の制御システム構成のサンプルです。
Raspberry Pi 5 と Yahboom ROSMASTER (Yahboom ROS robot control board V3.0) を使用して、カメラ映像配信、ArUcoマーカー検出、モーター制御、センサー取得、ゲームパッド操作などの機能を提供します。

## 主な機能

- **カメラ映像配信**: Webカメラ等からの映像（MJPEG）のリアルタイム送信
- **深度センサ画像配信**: Orbbec Astra Pro Plus からの深度情報を画像化して送信
- **ArUcoマーカー検出**: カメラ画像からのArUroマーカー認識結果を送信
- **ローバー操作**: ゲームパッドやキーボードによるロボット駆動制御
- **センサ読み取り**: IMU、磁気センサー、バッテリー電圧の取得
- **遠隔操作**: （オプション）VPN等を利用した遠隔ネットワーク通信のためのFast-DDS Discovery Server実装

## 構成

```txt
rover-sample-2025/
├── rover/        # ローバー本体（Raspberry Pi）用環境
└── remote/       # リモート制御端末（PC）用環境
```

## 前提条件

### ローバー側（Raspberry Pi 5）

- Ubuntu 24.04
- ROS 2 Jazzy
- Yahboom ROS robot control board V3.0
- Orbbec Astra Pro Plus (3Dカメラ)

### リモート制御側（PC）

- Ubuntu 24.04（物理マシン推奨）
- ROS 2 Jazzy
- ゲームコントローラー

## ライセンス

このプロジェクトのライセンスについては [LICENSE](LICENSE) を参照してください。

## 参考リンク

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ArUco Marker Detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
