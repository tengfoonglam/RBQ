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
            '/rbq/slam_odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odometry to TF converter started')
  
  
    
    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'imu_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 원래 quaternion 값 그대로 사용
        t.transform.rotation.x = -msg.pose.pose.orientation.z
        t.transform.rotation.y = -msg.pose.pose.orientation.y
        t.transform.rotation.z = -msg.pose.pose.orientation.x
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

        
        # 디버그 정보 (선택사항)
        self.get_logger().debug(f'Published TF: camera_init -> imu_link')


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
