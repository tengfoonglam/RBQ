#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odometry_to_tf')
        
        # TF broadcaster 초기화
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry 토픽 구독
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/rbq/stateEstimation/odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry to TF converter started')
  
    def is_valid_quaternion(self, x, y, z, w):
        """quaternion 값이 유효한지 검증"""
        import math
        # NaN 체크
        if math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isnan(w):
            return False
        # 무한대 체크
        if math.isinf(x) or math.isinf(y) or math.isinf(z) or math.isinf(w):
            return False
        return True
    
    def odom_callback(self, msg):
        # quaternion 값 검증
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 유효하지 않은 quaternion이면 기본값 사용
        if not self.is_valid_quaternion(qx, qy, qz, qw):
            self.get_logger().warn('Invalid quaternion detected, using default orientation')
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0  # 단위 quaternion
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'local_world'
        t.child_frame_id = 'trunk'

        # 위치 값도 검증
        t.transform.translation.x = msg.pose.pose.position.x if not math.isnan(msg.pose.pose.position.x) else 0.0
        t.transform.translation.y = msg.pose.pose.position.y if not math.isnan(msg.pose.pose.position.y) else 0.0
        t.transform.translation.z = msg.pose.pose.position.z if not math.isnan(msg.pose.pose.position.z) else 0.0

        # 검증된 quaternion 사용
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        
        # 디버그 정보 (선택사항)
        self.get_logger().debug(f'Published TF: local_world -> trunk')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 
