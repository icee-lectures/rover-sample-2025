"""/keyboard を購読し /cmd_vel に速度指令を配信するノード."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Twist


class WasdController(Node):
    def __init__(self) -> None:
        super().__init__('wasd_controller')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'keyboard',
            self._on_keys,
            10,
        )

        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s

        self.get_logger().info('Started wasd_controller (subscribing to /keyboard)')

    def _on_keys(self, msg: UInt8MultiArray) -> None:
        twist = Twist()
        twist.linear.x, twist.angular.z = self._compute_velocities(msg.data)
        self.publisher.publish(twist)

    def _compute_velocities(self, vk_codes: list[int]) -> tuple[float, float]:
        # Windows 仮想キーコード
        # WASD: W=0x57, A=0x41, S=0x53, D=0x44
        # 矢印キー: Up=0x26, Down=0x28, Left=0x25, Right=0x27
        # Q=0x51 (終了)
        
        if 0x51 in vk_codes:  # Q キー
            return 0.0, 0.0

        forward = (0x57 in vk_codes)  # W
        back = (0x53 in vk_codes)     # S
        left = (0x41 in vk_codes)     # A
        right = (0x44 in vk_codes)    # D
        
        # 矢印キーでも操作可能
        forward_arrow = (0x26 in vk_codes)  # 上矢印
        back_arrow = (0x28 in vk_codes)     # 下矢印
        left_arrow = (0x25 in vk_codes)     # 左矢印
        right_arrow = (0x27 in vk_codes)    # 右矢印

        linear = 0.0
        angular = 0.0

        # 前進/後退
        if (forward or forward_arrow) and not (back or back_arrow):
            linear = self.linear_speed
        elif (back or back_arrow) and not (forward or forward_arrow):
            linear = -self.linear_speed

        # 左回転/右回転
        if (left or left_arrow) and not (right or right_arrow):
            angular = self.angular_speed
        elif (right or right_arrow) and not (left or left_arrow):
            angular = -self.angular_speed

        return linear, angular


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WasdController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
