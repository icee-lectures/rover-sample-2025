#!/usr/bin/env python3
"""
ArUcoマーカー追跡ノード

id=0のArUcoマーカーを見つけたら、それに向かって走る制御を行う。
"""

import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class ArucoTraceNode(Node):
    """ArUcoマーカーを追跡して走るノード"""

    def __init__(self):
        super().__init__("aruco_trace")
        
        # パラメータの定義
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("linear_speed", 1.0)
        self.declare_parameter("angular_speed", 5.0)
        self.declare_parameter("distance_threshold", 1.0)
        self.declare_parameter("distance_tolerance", 0.1)
        
        self.marker_id = self.get_parameter("marker_id").value
        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.distance_threshold = self.get_parameter("distance_threshold").value
        self.distance_tolerance = self.get_parameter("distance_tolerance").value
        
        # サブスクライバー：ArUcoマーカー情報
        self.marker_subscriber = self.create_subscription(
            ArucoDetection,
            "/aruco_detections",
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

    def marker_callback(self, msg: ArucoDetection) -> None:
        """ArUcoマーカー情報のコールバック"""
        self.last_marker_time = self.get_clock().now()
        
        # marker_idと一致するマーカーを探す
        target_marker = None
        for marker in msg.markers:
            if marker.marker_id == self.marker_id:
                target_marker = marker
                break
        
        if target_marker is None:
            # マーカーが見つからない場合は停止
            self.stop_robot()
            return
        
        # マーカーの位置情報から3D座標を取得
        # pose には位置(x, y, z)と回転(quaternion)が含まれる
        pose = target_marker.pose
        
        # カメラ座標系での位置
        # x: 右方向, y: 下方向, z: カメラ前方向
        marker_x = pose.position.x
        marker_y = pose.position.y
        marker_z = pose.position.z
        
        self.get_logger().debug(
            f"Marker id={target_marker.marker_id} at ({marker_x:.3f}, {marker_y:.3f}, {marker_z:.3f})"
        )
        
        # 速度指令を計算
        twist = Twist()
        
        # X軸方向のオフセット（回転制御）
        # marker_xが0に近いほど中心に位置している
        if abs(marker_x) > 0.05:  # 中心からのずれが大きい場合
            # マーカーが右にある場合は左に回転（正の角速度は右旋回）
            twist.angular.z = -self.angular_speed if marker_x > 0 else self.angular_speed
        else:
            # マーカーが中心近くにある場合は回転しない
            twist.angular.z = 0.0
        
        # Z軸方向（前進・後進制御）
        # marker_zが大きいほど遠い、小さいほど近い（30cm=1.0、170cm=5.0）
        if marker_z < self.distance_threshold - self.distance_tolerance:
            # マーカーが近い→後退
            twist.linear.x = -self.linear_speed
        elif marker_z > self.distance_threshold + self.distance_tolerance:
            # マーカーが遠い→前進
            twist.linear.x = self.linear_speed
        else:
            # マーカーが適切な距離→停止
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
