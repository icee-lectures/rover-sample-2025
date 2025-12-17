"""
ROS2ノード: キーボード入力をパブリッシュ
WASDキーで移動速度を制御
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        
        # パブリッシャーの作成
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # キーボード入力の設定
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # 速度設定
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        
        # 現在の速度
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # タイマーの作成（50Hz = 0.02秒）
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('キーボードパブリッシャーを起動しました')
        self.get_logger().info('操作方法: W(前進), S(後退), A(左回転), D(右回転), Q(停止)')
    
    def __del__(self):
        """終了時にキーボード設定を復元"""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        except:
            pass
    
    def timer_callback(self):
        """タイマーコールバック: キーボード入力を読み取ってパブリッシュ"""
        # キーボード入力を読み取る
        self.read_keyboard()
        
        # Twistメッセージの作成
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.angular.z = self.angular_vel
        
        # パブリッシュ
        self.publisher.publish(twist_msg)
    
    def read_keyboard(self):
        """キーボード入力を読み取る"""
        import select
        
        # ノンブロッキングでキー入力を読み取る
        if select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            
            if key == 'w' or key == 'W':
                self.linear_vel = self.linear_speed
                self.angular_vel = 0.0
            elif key == 's' or key == 'S':
                self.linear_vel = -self.linear_speed
                self.angular_vel = 0.0
            elif key == 'a' or key == 'A':
                self.linear_vel = 0.0
                self.angular_vel = self.angular_speed
            elif key == 'd' or key == 'D':
                self.linear_vel = 0.0
                self.angular_vel = -self.angular_speed
            elif key == 'q' or key == 'Q' or key == '\x03':  # Ctrl+C
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.get_logger().info('終了します')
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    keyboard_publisher = KeyboardPublisher()
    
    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
