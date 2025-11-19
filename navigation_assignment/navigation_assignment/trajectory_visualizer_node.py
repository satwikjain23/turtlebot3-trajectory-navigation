#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

class TrajectoryVisualizer(Node):

    def __init__(self):
        super().__init__("trajectory_visualizer")

        # Subscribers
        self.create_subscription(PoseArray, "/trajectory", self.trajectory_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publishers
        self.traj_pub = self.create_publisher(Path, "/visualized_trajectory", 10)
        self.robot_path_pub = self.create_publisher(Path, "/robot_path", 10)

        # Path storage
        self.robot_path = Path()
        self.robot_path.header.frame_id = "map"

        self.get_logger().info("Trajectory Visualizer Node Started.")

    # ----------------------------
    # 1. Visualize Target Trajectory
    # ----------------------------
    def trajectory_callback(self, msg: PoseArray):
        path_msg = Path()
        path_msg.header.frame_id = "map"

        for pose in msg.poses:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = pose.position.x
            ps.pose.position.y = pose.position.y
            path_msg.poses.append(ps)

        self.traj_pub.publish(path_msg)

    # ----------------------------
    # 2. Record Robot's Actual Path
    # ----------------------------
    def odom_callback(self, msg: Odometry):
        # Convert odom → pose stamped
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "map"
        ps.pose = msg.pose.pose

        self.robot_path.poses.append(ps)

        # Publish updated path
        self.robot_path_pub.publish(self.robot_path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
