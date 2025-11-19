#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class TrajectoryGeneratorNode(Node):

    def __init__(self):
        super().__init__("trajectory_generator_node")

        self.sub = self.create_subscription(
            Path, "/smoothed_path", self.callback, 10)

        self.pub = self.create_publisher(PoseArray, "/trajectory", 10)

        self.velocity = 0.2  # m/s
        self.get_logger().info("Trajectory Generator Node Started.")

    def callback(self, msg):
        poses = msg.poses

        traj_msg = PoseArray()
        traj_msg.header.frame_id = "map"

        prev_x = poses[0].pose.position.x
        prev_y = poses[0].pose.position.y
        t = 0.0

        for ps in poses:
            x = ps.pose.position.x
            y = ps.pose.position.y

            dist = np.hypot(x - prev_x, y - prev_y)
            dt = dist / self.velocity
            t += dt

            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = t  # storing time in z temporarily

            traj_msg.poses.append(new_pose)

            prev_x = x
            prev_y = y

        self.pub.publish(traj_msg)
        self.get_logger().info("Published trajectory.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
