"""
ROS2ノード: ゲームコントローラー情報をパブリッシュ
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import sys
import os


class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('gamepad_publisher')
        
        # パブリッシャーの作成
        self.publisher = self.create_publisher(Joy, 'joy', 10)
        
        # Pygameの初期化
        os.environ["SDL_JOYSTICK_DEVICE"] = "/dev/input/js0"
        pygame.init()
        pygame.joystick.init()
        
        # コントローラーの接続確認
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.get_logger().error('ゲームコントローラーが接続されていません')
            sys.exit(1)
        
        # 最初のコントローラーを初期化
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # コントローラー情報をログに出力
        self.get_logger().info(f'コントローラー名: {self.joystick.get_name()}')
        self.get_logger().info(f'軸の数: {self.joystick.get_numaxes()}')
        self.get_logger().info(f'ボタンの数: {self.joystick.get_numbuttons()}')
        self.get_logger().info(f'ハットスイッチの数: {self.joystick.get_numhats()}')
        
        # タイマーの作成（50Hz = 0.02秒）
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('ゲームコントローラーパブリッシャーを起動しました')
    
    def timer_callback(self):
        """タイマーコールバック: コントローラーの状態を読み取ってパブリッシュ"""
        # Pygameイベント処理
        pygame.event.pump()
        
        # Joyメッセージの作成
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = 'gamepad'
        
        # 軸の値を取得
        joy_msg.axes = []
        for i in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(i)
            joy_msg.axes.append(float(axis_value))
        
        # ボタンの状態を取得
        joy_msg.buttons = []
        for i in range(self.joystick.get_numbuttons()):
            button_value = self.joystick.get_button(i)
            joy_msg.buttons.append(int(button_value))
        
        # ハットスイッチの状態をボタンとして追加
        # ハットスイッチは (x, y) のタプルで返される
        # 上下左右を個別のボタンとして扱う
        for i in range(self.joystick.get_numhats()):
            hat_value = self.joystick.get_hat(i)
            # 左: -1, 右: 1, 上: 1, 下: -1
            joy_msg.buttons.append(int(hat_value[0] == -1))  # 左
            joy_msg.buttons.append(int(hat_value[0] == 1))   # 右
            joy_msg.buttons.append(int(hat_value[1] == 1))   # 上
            joy_msg.buttons.append(int(hat_value[1] == -1))  # 下
        
        # パブリッシュ
        self.publisher.publish(joy_msg)
    
    def destroy_node(self):
        """ノード終了時の処理"""
        self.joystick.quit()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        gamepad_publisher = GamepadPublisher()
        rclpy.spin(gamepad_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'gamepad_publisher' in locals():
            gamepad_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
