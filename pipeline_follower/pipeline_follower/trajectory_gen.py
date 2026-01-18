import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import interp1d

class InterpolatedTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('interpolated_trajectory_publisher')
        
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.publisher_ = self.create_publisher(Path, '/bluerov2/path', qos)

        self.path = self.generate_path()
        #self.timer = self.create_timer(1.0, self.publish_path)
        self.publish_path()

    def generate_path(self):
        path = Path()
        path.header.frame_id = 'world'

        # Define waypoints
        waypoints = np.array([
            [-8.0, -2.0, -5.0],
            [-8.0, 4.0, -5.0],
            [-1.0, 6.0, -5.0],
            [5.0, 7.0, -5.0],
            [5.0, 10.5, -5.0],
            [5.0, 10.5, -9.0],
            [10.0, 10.5, -9.0],
            [14.0, 5.5, -9.0],
            [14.0, 1.0, -9.0],
            [9.0, -5.0, -9.0],
            [5.0, -5.0, -9.0],
            [5.0, -5.0, -5.0],
            [-8.0, -2.0, -5.0]
        ])

        # Generate cumulative distance
        distances = np.cumsum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))
        distances = np.insert(distances, 0, 0)

        # Interpolation functions
        fx = interp1d(distances, waypoints[:, 0], kind='linear')
        fy = interp1d(distances, waypoints[:, 1], kind='linear')
        fz = interp1d(distances, waypoints[:, 2], kind='linear')

        interp_distances = np.linspace(0, distances[-1], num=20)

        for d in interp_distances:
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = float(fx(d))
            pose.pose.position.y = float(fy(d))
            pose.pose.position.z = float(fz(d))
            path.poses.append(pose)

        return path

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path)
        #self.get_logger().info('Path published')

def main(args=None):
    rclpy.init(args=args)
    node = InterpolatedTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

