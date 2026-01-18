#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import random

class PseudoDVLNode(Node):
    def __init__(self):
        #Initialize node
        super().__init__('pseudo_dvl_node')
        
        self.sub = self.create_subscription(Odometry, '/bluerov2/odom', self.odom_callback, 10)
        self.dvl_pub = self.create_publisher(TwistWithCovarianceStamped, '/bluerov2/dvl', 10)
        
        self.noise_std = 0.01
        self.get_logger().info('Pseudo DVL node initialized')
    
    def odom_callback(self, msg):
        dvl_msg = TwistWithCovarianceStamped()
        dvl_msg.header = msg.header
        dvl_msg.header.frame_id = 'bluerov2/base_link'
        
        # Extract linear velocities and add Gaussian noise
        vx = msg.twist.twist.linear.x + random.gauss(0, self.noise_std)
        vy = msg.twist.twist.linear.y + random.gauss(0, self.noise_std)
        vz = msg.twist.twist.linear.z + random.gauss(0, self.noise_std)
        
        dvl_msg.twist.twist.linear.x = vx #if abs(vx) > 0.02 else 0.0
        dvl_msg.twist.twist.linear.y = vy #if abs(vy) > 0.02 else 0.0
        dvl_msg.twist.twist.linear.z = vz #if abs(vz) > 0.02 else 0.0
        
        dvl_msg.twist.twist.angular.x = msg.twist.twist.angular.x
        dvl_msg.twist.twist.angular.y = msg.twist.twist.angular.y
        dvl_msg.twist.twist.angular.z = msg.twist.twist.angular.z
        
        # Covariance values
        cov = [0.1, 0.0,    0.0,    0.0,    0.0,    0.0,
                0.0,    0.1, 0.0,    0.0,    0.0,    0.0,
                0.0,    0.0,    0.1, 0.0,    0.0,    0.0,
                0.0,    0.0,    0.0,    9999.0, 0.0,    0.0,
                0.0,    0.0,    0.0,    0.0,    9999.0, 0.0,
                0.0,    0.0,    0.0,    0.0,    0.0,    9999.0]
        dvl_msg.twist.covariance = cov
        
        self.dvl_pub.publish(dvl_msg)

def main(args = None):
    rclpy.init(args=args)
    node = PseudoDVLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
