#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class TrajectoryGeneratorNode(Node):
    """
    This node receives a smoothed geometric path (nav_msgs/Path)
    and converts it into a time-parameterized trajectory.

    It assigns a timestamp to every point on the path based on a
    constant desired velocity. The output is published as a PoseArray,
    where the timestamp 't' is stored inside the z-coordinate of each.pose.
    """

    def __init__(self):
        super().__init__("trajectory_generator_node")

        # Subscribe to the smoothed path published by path_smoothing_node
        self.sub = self.create_subscription(
            Path, "/smoothed_path", self.callback, 10
        )

        # Publisher for the time-parameterized trajectory
        self.pub = self.create_publisher(PoseArray, "/trajectory", 10)

        # Constant forward velocity assumed for time computation
        self.velocity = 0.2  # meters per second

        self.get_logger().info("Trajectory Generator Node Started.")

    def callback(self, msg):
        """
        This function executes every time a new smoothed path is received.
        It:
        1. Extracts the positions from nav_msgs/Path
        2. Computes distances between consecutive points
        3. Converts those distances to time (t = dist / velocity)
        4. Publishes a PoseArray with positions + timestamps
        """

        poses = msg.poses   # list of PoseStamped in the Path message

        # Output trajectory (each pose contains x, y, and time stored in z)
        traj_msg = PoseArray()
        traj_msg.header.frame_id = "map"

        # Initialize with the first pose for distance computation
        prev_x = poses[0].pose.pose.position.x
        prev_y = poses[0].pose.pose.position.y

        t = 0.0  # Cumulative time along trajectory

        for ps in poses:

            # Extract x and y from current path pose
            x = ps.pose.position.x
            y = ps.pose.position.y

            # Compute euclidean distance from previous point
            dist = np.hypot(x - prev_x, y - prev_y)

            # Convert distance to time increment (assuming constant velocity)
            dt = dist / self.velocity
            t += dt

            # Create trajectory pose (x, y, and timestamp stored in z)
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = t

            # Append to trajectory
            traj_msg.poses.append(new_pose)

            # Update previous point
            prev_x = x
            prev_y = y

        # Publish the time-parameterized trajectory
        self.pub.publish(traj_msg)
        self.get_logger().info("Published trajectory.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
