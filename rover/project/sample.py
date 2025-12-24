#!/usr/bin/env python3
"""
ArUcoマーカー追跡ノード

id=0のArUcoマーカーを見つけたら、それに向かって走る制御を行う。
"""

import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class ArucoTraceNode(Node):
    """ArUcoマーカーを追跡して走るノード"""

    def __init__(self):
        super().__init__("aruco_trace")
        
        # パラメータの定義
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("linear_speed", 0.5)  # m/s
        self.declare_parameter("angular_speed", 1.0)  # rad/s
        self.declare_parameter("center_threshold", 50)  # ピクセル
        
        self.marker_id = self.get_parameter("marker_id").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.center_threshold = self.get_parameter("center_threshold").value
        
        # サブスクライバー：ArUcoマーカー情報
        self.marker_subscriber = self.create_subscription(
            MarkerArray,
            "/aruco/markers",
            self.marker_callback,
            10
        )
        
        # パブリッシャー：速度指令
        self.vel_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )
        
        # タイマー：マーカーが見つからない場合の停止処理
        self.timer = self.create_timer(0.5, self.timeout_callback)
        self.last_marker_time = self.get_clock().now()
        
        self.get_logger().info(
            f"ArUco Trace Node started (marker_id={self.marker_id})"
        )

    def marker_callback(self, msg: MarkerArray) -> None:
        """ArUcoマーカー情報のコールバック"""
        self.last_marker_time = self.get_clock().now()
        
        # id=0のマーカーを探す
        target_marker = None
        for marker in msg.markers:
            if marker.id == self.marker_id:
                target_marker = marker
                break
        
        if target_marker is None:
            # マーカーが見つからない場合は停止
            self.stop_robot()
            return
        
        # マーカーの中心座標を計算
        corners = target_marker.corners.data
        if len(corners) < 4:
            self.stop_robot()
            return
        
        # 4つのコーナーから中心を計算
        center_x = sum(corners[i].x for i in range(4)) / 4.0
        center_y = sum(corners[i].y for i in range(4)) / 4.0
        
        # カメラ画像の中心（仮定：解像度 640x480）
        image_center_x = 320.0
        image_center_y = 240.0
        
        # 画像中心からのオフセット
        offset_x = center_x - image_center_x
        offset_y = center_y - image_center_y
        
        self.get_logger().debug(
            f"Marker id={target_marker.id} at ({center_x:.1f}, {center_y:.1f})"
        )
        
        # 速度指令を計算
        twist = Twist()
        
        # X方向のオフセット（回転制御）
        if abs(offset_x) > self.center_threshold:
            # マーカーが中心より左にある場合は左回転
            twist.angular.z = self.angular_speed if offset_x > 0 else -self.angular_speed
        else:
            # マーカーが中心近くにある場合は直進
            twist.angular.z = 0.0
        
        # Y方向のオフセット（前進・後進制御）
        # Y座標が小さい（画像上部）ほど遠い、大きい（画像下部）ほど近い
        if center_y < image_center_y:
            # マーカーが画像上部（遠い）→前進
            twist.linear.x = self.linear_speed
        elif center_y > image_center_y + self.center_threshold:
            # マーカーが画像下部（近い）→後退
            twist.linear.x = -self.linear_speed * 0.5
        else:
            # マーカーが画像中央付近→停止
            twist.linear.x = 0.0
        
        # 速度指令を発行
        self.vel_publisher.publish(twist)

    def timeout_callback(self) -> None:
        """タイムアウトコールバック：マーカーが見つからなくなった場合の処理"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_marker_time).nanoseconds / 1e9
        
        # 1秒以上マーカーが見つからない場合は停止
        if time_diff > 1.0:
            self.stop_robot()

    def stop_robot(self) -> None:
        """ロボットを停止"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTraceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
