#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceWrapper(Node):
    def __init__(self):
        super().__init__('imu_covariance_wrapper')
        
        self.sub = self.create_subscription(Imu, '/bluerov2/mpu', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.get_logger().info('Imu cov node active')
    
    def imu_callback(self, msg):
        msg.header.frame_id = 'bluerov2/mpu'
        
        msg.orientation_covariance[0] = 0.1
        msg.orientation_covariance[4] = 0.1
        msg.orientation_covariance[8] = 0.1
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
