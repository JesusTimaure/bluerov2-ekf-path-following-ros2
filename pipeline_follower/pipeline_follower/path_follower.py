#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        self.distance_thr = 0.25
        
        self.current_pose = None
        self.current_target = None
        self.path = []
        self.current_index = 0
        self.waiting_for_reach = False
        
        # QoS Settings
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        cmd_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        
        
        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/bluerov2/ekf_odom', self.odom_callback, 10)
        self.sub_path = self.create_subscription(Path, '/bluerov2/path', self.path_callback, qos)
        
        # Publisher
        self.cmd_pub = self.create_publisher(PoseStamped, '/bluerov2/cmd_pose', cmd_qos)
        
        self.get_logger().info('Path follower node initialized')
        self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.path and not self.waiting_for_reach:
            self.get_logger().info('Odometry received; sending first waypoint')
            self.send_next_waypoint()
    
    def path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn('Received empty path')
            return
        
        self.get_logger().info('Path received')
        self.path = msg.poses
        self.current_index = 0
        self.waiting_for_reach = False
        
        if self.current_pose is not None:
            self.send_next_waypoint()
        else:
            self.get_logger().warn('Path received but waiting for odometry')
    
    def control_loop(self):
        if self.current_pose is None or not self.waiting_for_reach or self.current_target is None:
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.current_target.header.frame_id
        pose.pose = self.current_target.pose
        self.cmd_pub.publish(pose)
            
        self.check_proximity()
    
    def check_proximity(self):
        if self.current_index >= len(self.path):
            self.get_logger().info('Trajectory completed')
            self.waiting_for_reach = False
            self.current_target = None
            return
        
        target = self.path[self.current_index].pose
        dx = target.position.x - self.current_pose.position.x
        dy = target.position.y - self.current_pose.position.y
        dz = target.position.z - self.current_pose.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist < self.distance_thr:
            self. current_index += 1
            self.send_next_waypoint()
    
    def send_next_waypoint(self):
        if self.current_pose is None:
            self.get_logger().warn('Current pose not yet received; cannot send waypoint')
            return
        
        if self.current_index < len(self.path):
            self.current_target = self.path[self.current_index]
            self.get_logger().info(f'Sending waypoint {self.current_index + 1}/{len(self.path)}')
            self.waiting_for_reach = True
        else:
            self.get_logger().info('All waypoints were reached')
            self.current_target = None
            self.waiting_for_reach = False
            
def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
