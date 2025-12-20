# Yahboom Rosboard V3 Driver API リファレンス

対象ファイル: [rover/src/yahboom_rosboard_v3_driver.py](rover/src/yahboom_rosboard_v3_driver.py)

## 概要

- Yahboom Rosmaster V3 系ボード向けのPythonドライバです。
- シリアル経由で車両（X3/X3Plus/X1/R2）やアーム、RGB、ブザー、IMU 等を制御・取得します。
- 受信スレッドによる自動レポートでセンサ値・速度などが周期更新されます（デフォルト有効）。

## 依存関係・前提

- 依存: `pyserial`（`import serial`）、`struct`、`threading`、`time`。
- 通信: デフォルトは `/dev/ttyUSB0`、ボーレート 115200。
- 権限: Linux では `dialout` グループや `udev` 設定でポートアクセス権を付与してください。

## クラス: Rosmaster

### 初期化

- コンストラクタ: `Rosmaster(car_type=4, com="/dev/ttyUSB0", delay=0.002, debug=False)`
  - シリアルをオープンし、内部状態を初期化します。
  - アーム用バスサーボのトルクを有効化します（初回角度読取の安定化）。
  - `car_type` は下記の車両タイプ定数参照。

### 受信スレッド

- `create_receive_threading()`
  - 受信ループをデーモンスレッドで開始し、MCUからの自動レポートやリクエスト応答を処理します。

### 自動レポート制御

- `set_auto_report_state(enable, forever=False)`
  - MCUの自動送信を有効/無効化します。`enable=True` で 10ms間隔のパケット送信（4種類を順繰りに送るため各40ms更新）。
  - `forever=True` でフラッシュに永続保存（MCU側動作に時間がかかるため若干の待ちが入ります）。

## センサ・状態取得

- `get_accelerometer_data()` → `(ax, ay, az)`
  - 加速度[m/s^2]相当（MPU9250時は±2g換算）。
- `get_gyroscope_data()` → `(gx, gy, gz)`
  - 角速度[rad/s]相当（MPU9250時は±500 dps換算）。
- `get_magnetometer_data()` → `(mx, my, mz)`
  - 地磁気の生値。
- `get_imu_attitude_data(ToAngle=True)` → `(roll, pitch, yaw)`
  - `ToAngle=True` で角度[deg]、`False` でラジアン。
- `get_motion_data()` → `(vx, vy, vz)`
  - 車両速度（シリアル報告のスケーリング済み値）。
- `get_battery_voltage()` → `vol`
  - バッテリ電圧[V]。
- `get_motor_encoder()` → `(m1, m2, m3, m4)`
  - 4輪のエンコーダカウント。
- `get_motion_pid()` → `[kp, ki, kd]`
  - 現在のモーションPIDを取得。タイムアウト時は `[-1, -1, -1]`。
- `get_car_type_from_machine()` → `car_type`
  - MCUから車両タイプを取得。タイムアウト時は `-1`。
- `get_version()` → `version`
  - MCUのバージョン（例: 1.1）。取得失敗時は `-1`。
- `clear_auto_report_data()`
  - 自動レポートで蓄積した内部キャッシュをゼロクリア。

## 車両制御

- `set_motor(speed_1, speed_2, speed_3, speed_4)`
  - PWM直指定で各モータの速度を操作。範囲 `[-100, 100]`、`127` は「変更なし」。
- `set_car_run(state, speed, adjust=False)`
  - プリセット動作: `state` in `{0:停止, 1:前進, 2:後退, 3:左, 4:右, 5:左回転, 6:右回転, 7:停車}`。
  - `speed` は `[-100, 100]`。`adjust=True` でジャイロ補正フラグ付与（実機未有効）。
- `set_car_motion(v_x, v_y, v_z)`
  - 速度指令。車種により許容範囲が異なります。
    - X3: `v_x,v_y ∈ [-1.0, 1.0]`, `v_z ∈ [-5, 5]`
    - X3Plus: `v_x,v_y ∈ [-0.7, 0.7]`, `v_z ∈ [-3.2, 3.2]`
    - R2/R2L: `v_x ∈ [-1.8, 1.8]`, `v_y ∈ [-0.045, 0.045]`, `v_z ∈ [-3, 3]`
- `set_pid_param(kp, ki, kd, forever=False)`
  - モーションPIDを設定。各値は `[0, 10.00]`。`forever=True` でMCUフラッシュへ保存（遅延あり）。
- `set_car_type(car_type)`
  - 車種設定（整数）。`str(car_type).isdigit()` による検証を通過した入力のみ受理。MCU側にも送信し `0x5F`（永続）を併送。
- `reset_car_state()`
  - 停止・ライトオフ・ブザーオフにリセット。
- `reset_flash_value()`
  - MCUフラッシュ保存値を工場出荷値へリセット。

## PWMサーボ制御（1〜4）

- `set_pwm_servo(servo_id, angle)`
  - `servo_id ∈ [1,4]`、`angle ∈ [0,180]`。
- `set_pwm_servo_all(a1, a2, a3, a4)`
  - 同時制御。範囲外は `255`（変更なし）を送信。

## バスサーボ（UART）制御（アーム 1〜6 他）

- `set_uart_servo(servo_id, pulse_value, run_time=500)`
  - `servo_id ∈ [1,250]` または `254`（全サーボ）。`pulse_value ∈ [96,4000]`、`run_time ∈ [0,2000]` [ms]。
- `set_uart_servo_angle(s_id, s_angle, run_time=500)`
  - アームの物理角度で指定。各軸許容範囲:
    - 1〜4: `[0,180]`, 5: `[0,270]`, 6: `[0,180]`。
- `set_uart_servo_id(servo_id)`
  - バスサーボのID設定。複数接続時は同一ID化に注意（単体接続時のみ実行推奨）。
- `set_uart_servo_torque(enable)`
  - トルクON/OFF（ONでコマンド制御可能、OFFで手動回転可能）。
- `set_uart_servo_ctrl_enable(enable)`
  - アーム制御プロトコルの送信自体を有効/無効。
- `set_uart_servo_angle_array(angle_s=[90,90,90,90,90,180], run_time=500)`
  - 6軸一括角度指令。範囲チェックと内部パルス変換を行います。
- `set_uart_servo_offset(servo_id)` → `state`
  - 中点偏差設定。`servo_id=0` で全軸を工場出荷値へ復元。応答ステータスを返却。
- `get_uart_servo_value(servo_id)` → `(read_id, pulse)`
  - 現在パルス位置の読取。タイムアウト時は `(-1, -1)`、例外時は `(-2, -2)`。
- `get_uart_servo_angle(s_id)` → `angle` / `-1`（範囲外）
  - 現在角度の読取。例外時は `-2`。
- `get_uart_servo_angle_array()` → `[a1..a6]`
  - 6軸角度の一括読取。例外時は `[-2, -2, -2, -2, -2, -2]`。

## Ackermann（R2系）ステアリング

- `set_akm_default_angle(angle, forever=False)`
  - 前輪のデフォルト角度設定。`angle ∈ [60,120]`。
- `get_akm_default_angle()` → `angle` / `-1`
  - 前輪デフォルト角度読取。
- `set_akm_steering_angle(angle, ctrl_car=False)`
  - デフォルト角度に対する相対ステア角。`angle ∈ [-45,45]`。`ctrl_car=True` で左右モータ速度も同時変更。

## 車両タイプ定数

- `CARTYPE_X3 = 0x01`
- `CARTYPE_X3_PLUS = 0x02`
- `CARTYPE_X1 = 0x04`
- `CARTYPE_R2 = 0x05`

## 使用例

```python
from rover.src.yahboom_rosboard_v3_driver import Rosmaster
import time

bot = Rosmaster(car_type=4, com="/dev/ttyUSB0", debug=True)
bot.create_receive_threading()
bot.set_auto_report_state(True)

# 車両を前進させる
bot.set_car_run(state=1, speed=50)
time.sleep(1.0)
bot.set_car_run(state=0, speed=0)

# IMU姿勢を取得（度）
roll, pitch, yaw = bot.get_imu_attitude_data(ToAngle=True)
print("roll=", roll, "pitch=", pitch, "yaw=", yaw)

# アームの各軸角度を一括設定
bot.set_uart_servo_angle_array([90, 90, 90, 90, 90, 180], run_time=800)

# バッテリ電圧表示
print("battery(V):", bot.get_battery_voltage())
```

## 注意事項・設計メモ

- 自動レポート有効時、センサ値は数十ms単位で更新されます。明示的な読取関数は内部でリクエスト送信し短時間待機します。
- 一部の永続操作（PID、デフォルト角度等）はMCUフラッシュに書き込みを伴い、遅延が挿入されます。
- 範囲外入力は無効化または「変更なし（127）」として送信される場合があります。
- デストラクタでシリアルをクローズします。長時間の運用では例外時のクローズに留意してください。

## 動作確認（スクリプト起動）

- 単体テスト実行（IMUや電圧を定期表示）:

```bash
python3 rover/src/yahboom_rosboard_v3_driver.py
```

- 依存関係インストール（未導入の場合）:

```bash
python3 -m pip install pyserial
```

---
このドキュメントはコード実装を元に生成されています。詳細はソース: [rover/src/yahboom_rosboard_v3_driver.py](rover/src/yahboom_rosboard_v3_driver.py) を参照してください。
