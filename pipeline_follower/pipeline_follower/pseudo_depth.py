#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import random

class PseudoPressureDepth(Node):
    def __init__(self):
        super().__init__('pseudo_depth_node')
        
        self.sub = self.create_subscription(Odometry, '/bluerov2/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/bluerov2/depth', 10)
        self.noise = 0.1     
        self.get_logger().info('Pseudo pressure depth node running')
        
    def odom_callback(self, msg):
        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.header.frame_id = 'world'
        
        # Extraction of depth
        z_pos = msg.pose.pose.position.z + random.gauss(0, self.noise)
        depth_msg.pose.pose.position.z = z_pos
        depth_msg.pose.pose.orientation.w = 1.0

        
        # Covariance values
        cov = [0.0,     0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.1,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    0.0]
        depth_msg.pose.covariance = cov
        self.pub.publish(depth_msg)
         
def main(args=None):
    rclpy.init(args=args)
    node = PseudoPressureDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
