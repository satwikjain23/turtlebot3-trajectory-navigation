#!/usr/bin/env python3
"""
Path Smoothing Node (Task 1)

This ROS2 node takes a set of discrete 2D waypoints and generates a smooth,
continuous path using B-spline interpolation. The smoothed path is then
published as a nav_msgs/Path message for use by later nodes such as the
trajectory generator and controller.

The algorithm uses SciPy’s `splprep` and `splev` functions to fit a B-spline
curve, handle edge cases like small waypoint counts, and output a uniformly
sampled smooth path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import splprep, splev


# ---------------------------------------------------------------
# Helper Function: Remove duplicate consecutive waypoints
# ---------------------------------------------------------------
def remove_duplicates(points):
    """
    Removes duplicate consecutive points.

    Duplicate points cause SciPy's `splprep` to fail because the
    spline parameterization requires strictly increasing arc length.

    Args:
        points (list): List of (x, y) tuples.

    Returns:
        list: Cleaned list with duplicates removed.
    """
    cleaned = []
    for p in points:
        if len(cleaned) == 0 or p != cleaned[-1]:
            cleaned.append(p)
    return cleaned


# ---------------------------------------------------------------
# Core Function: Smooth using B-Spline interpolation
# ---------------------------------------------------------------
def smooth(points, smoothness=0.0, num_samples=100):
    """
    Fits a continuous B-spline curve through given waypoints and samples it.

    Args:
        points (list): List of (x, y) tuples representing raw waypoints.
        smoothness (float): Smoothing factor for B-spline (0 = strict fit).
        num_samples (int): Number of output points to sample on spline.

    Returns:
        list: List of (x, y) tuples representing a smooth sampled trajectory.
    """

    # Remove duplicate points to avoid spline fitting failures
    points = remove_duplicates(points)

    arr = np.array(points)
    x, y = arr[:,0], arr[:,1]
    m = len(points)

    # ------------------------------------------------------------------
    # Choosing spline degree 'k' depending on number of points:
    #   - k = 1 → linear spline (for only 2 points)
    #   - k = 2 → quadratic spline (for exactly 3 points)
    #   - k = 3 → cubic spline (standard smooth curve)
    #
    # This avoids the "m > k" SciPy error for small waypoint counts.
    # ------------------------------------------------------------------
    if m <= 2:
        k = 1
    elif m == 3:
        k = 2
    else:
        k = 3

    # Fit B-spline
    tck, u = splprep([x, y], s=smoothness, k=k)

    # Sample uniformly along the curve
    u_new = np.linspace(0, 1, num_samples)
    xs, ys = splev(u_new, tck)

    return list(zip(xs, ys))


# ---------------------------------------------------------------
# ROS2 Node: Path Smoothing
# ---------------------------------------------------------------
class PathSmoothingNode(Node):
    """
    ROS2 node that publishes a smoothed path from predefined waypoints.

    This node periodically:
      1. Smooths the hardcoded waypoints using the spline function.
      2. Converts them into a Path message.
      3. Publishes on the /smoothed_path topic.

    Other nodes (trajectory generator, controller) subscribe and use this path.
    """

    def __init__(self):
        super().__init__("path_smoothing_node")

        # Publisher: nav_msgs/Path
        self.pub = self.create_publisher(Path, "/smoothed_path", 10)

        # Hardcoded waypoints (can be loaded from file or service later)
        self.waypoints = [
            (0, 0),
            (2, 0),
            (2, 2)
        ]

        # Timer: publish smoothed path every 1 second
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info("Path Smoothing Node Started.")

    # -----------------------------------------------------------
    # Publish smoothed path as nav_msgs/Path
    # -----------------------------------------------------------
    def publish_path(self):
        # Run the spline smoothing function
        smoothed = smooth(self.waypoints)

        # Create the Path message
        msg = Path()
        msg.header.frame_id = "map"   # Path is expressed in map frame

        for (x, y) in smoothed:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)

        # Publish the message
        self.pub.publish(msg)
        self.get_logger().info("Published smoothed path.")


# ---------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathSmoothingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
