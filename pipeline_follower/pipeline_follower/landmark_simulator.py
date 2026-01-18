#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import numpy as np
import time
from scipy.optimize import least_squares



class AcousticPositioningNode(Node):
    def __init__(self):
        super().__init__('acoustic_positioning_node')
        
        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('noise_std_dev', 0.05)
        self.declare_parameter('delay_sec', 0.2)
        
        self.update_rate = self.get_parameter('update_rate_hz').value
        self.noise_std = self.get_parameter('noise_std_dev').value
        self.delay = self.get_parameter('delay_sec').value

        # Beacon parameters
        self.landmarks = np.array([
            [-2.5, -2.5, -8.0],
            [-2.5, 4.0, -8.0],
            [10.0, 4.0, -8.0],
            [10.0, -2.5, -8.0]
        ])

        # Subscribers and publishers
        self.sub = self.create_subscription(Odometry, '/bluerov2/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/bluerov2/acoustic_pose', 10)

        # Periodic timer to simulate beacon updates
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        self.latest_odom = None

        self.get_logger().info('Landmark acoustic beacon simulator started')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self):
        if self.latest_odom is None:
            return
            
        # Simulate delay
        time.sleep(self.delay)
        
        now = self.get_clock().now()
        
        # Extract true position
        pose = self.latest_odom.pose.pose
        true_pos = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        
        distances = np.linalg.norm(self.landmarks - true_pos, axis=1)
        noisy_ranges = distances + np.random.normal(0, self.noise_std, size=len(distances))
        
        # Trilateration
        estimated_pos = self.solve_trilateration(self.landmarks, noisy_ranges)
        if estimated_pos is None:
            self.get_logger().warn('Trilateration failed')
            return
        
        # Publish estimated position
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.pose.position.x = estimated_pos[0]
        pose_msg.pose.pose.position.y = estimated_pos[1]
        pose_msg.pose.pose.position.z = estimated_pos[2]
        
        # Covariances
        pose_msg.pose.covariance = [0.5, 0.0,  0.0,  0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0,  0.0, 0.0, 0.0,
                                    0.0, 0.0,  0.5, 0.0, 0.0, 0.0,
                                    0.0, 0.0,  0.0,  0.0, 0.0, 0.0,
                                    0.0, 0.0,  0.0,  0.0, 0.0, 0.0,
                                    0.0, 0.0,  0.0,  0.0, 0.0, 0.0]
        
        self.pub.publish(pose_msg)
        
    def solve_trilateration(self, landmarks, ranges):
        def residuals(pos):
            return np.linalg.norm(landmarks-pos, axis=1) - ranges
        
        x0 = np.mean(landmarks, axis=0)
        result = least_squares(residuals, x0)
        
        if result.success:
            return result.x
        else:
            return None
            
def main(args=None):
    rclpy.init(args=args)
    node = AcousticPositioningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
