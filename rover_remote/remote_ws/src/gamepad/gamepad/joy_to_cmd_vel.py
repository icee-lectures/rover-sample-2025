"""
ROS2ノード: Joyトピックをサブスクライブしてcmd_velトピックへTwistメッセージを送信
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        
        # Joyトピックのサブスクライバーを作成
        self.joy_subscriber = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # cmd_velトピックのパブリッシャーを作成
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('Joy to Cmd_vel ノードを起動しました')
    
    def joy_callback(self, joy_msg: Joy):
        """Joyメッセージを受け取ってcmd_velを送信"""
        # Twistメッセージの作成
        twist_msg = Twist()
        
        # linear.x に -joy_msg.axes[3] を割り当て
        if len(joy_msg.axes) > 3:
            linear_value = -joy_msg.axes[3]
            # 絶対値が0.01以下の場合は0.0とする
            twist_msg.linear.x = 0.0 if abs(linear_value) <= 0.01 else linear_value
        else:
            twist_msg.linear.x = 0.0
        
        # angular.z に -joy_msg.axes[2] を割り当て
        if len(joy_msg.axes) > 2:
            angular_value = -joy_msg.axes[2]
            # 絶対値が0.01以下の場合は0.0とする
            twist_msg.angular.z = 0.0 if abs(angular_value) <= 0.01 else angular_value * 5.0
        else:
            twist_msg.angular.z = 0.0
        
        # cmd_velトピックにパブリッシュ
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        joy_to_cmd_vel = JoyToCmdVel()
        rclpy.spin(joy_to_cmd_vel)
    except KeyboardInterrupt:
        pass
    finally:
        if 'joy_to_cmd_vel' in locals():
            joy_to_cmd_vel.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
