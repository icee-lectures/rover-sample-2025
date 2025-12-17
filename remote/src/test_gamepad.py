import os
import pygame

# スティックの値が思い通りにならない場合は以下コマンドでキャリブレーションを試みる
# jscal -c /dev/input/js0

# 特定のジョイスティックデバイスを指定（必要に応じて変更）
os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"

# Pygameの初期化
pygame.init()

# ジョイスティックの初期化
pygame.joystick.init()

# 接続されている数を確認
joystick_count = pygame.joystick.get_count()
print(f"検出されたジョイスティック数: {joystick_count}")

if joystick_count > 0:
    # 0番目のジョイスティックを取得
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"使用デバイス: {joy.get_name()}")
    print(f"軸の数: {joy.get_numaxes()}")
    print(f"ボタンの数: {joy.get_numbuttons()}")
    print(f"ハット（十字キー）の数: {joy.get_numhats()}")
    print("=" * 50)

if joystick_count > 0:
    # 0番目のジョイスティックを取得
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"使用デバイス: {joy.get_name()}")
    print(f"軸の数: {joy.get_numaxes()}")
    print(f"ボタンの数: {joy.get_numbuttons()}")
    print(f"ハット（十字キー）の数: {joy.get_numhats()}")
    print("=" * 50)

    try:
        while True:
            # イベント処理
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"[ボタン] ボタン {event.button} が押されました")
                elif event.type == pygame.JOYHATMOTION:
                    # 十字キー
                    if event.value != (0, 0):
                        print(f"[十字キー] {event.value}")
                elif event.type == pygame.JOYAXISMOTION:
                    # スティック（デッドゾーン0.1）
                    if abs(event.value) > 0.1:
                        # イベント発生時に全軸の値を表示
                        all_axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
                        print(f"[スティック] {[f'{v:.2f}' for v in all_axes]}")
            
            pygame.time.delay(50)
    except KeyboardInterrupt:
        print("終了します")
else:
    print("ジョイスティックが見つかりません。")

pygame.quit()
