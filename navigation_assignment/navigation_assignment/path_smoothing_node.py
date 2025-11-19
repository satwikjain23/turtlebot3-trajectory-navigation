#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import splprep, splev

def remove_duplicates(points):
    cleaned = []
    for p in points:
        if len(cleaned) == 0 or p != cleaned[-1]:
            cleaned.append(p)
    return cleaned

def smooth(points, smoothness=0.0, num_samples=100):
    points = remove_duplicates(points)
    arr = np.array(points)
    x, y = arr[:,0], arr[:,1]
    m = len(points)

    if m <= 2:
        k = 1
    elif m == 3:
        k = 2
    else:
        k = 3

    tck, u = splprep([x, y], s=smoothness, k=k)
    u_new = np.linspace(0, 1, num_samples)
    xs, ys = splev(u_new, tck)
    return list(zip(xs, ys))

class PathSmoothingNode(Node):

    def __init__(self):
        super().__init__("path_smoothing_node")

        self.pub = self.create_publisher(Path, "/smoothed_path", 10)

        # Hardcoded waypoints for now:
        self.waypoints = [
            (0,0),
            (2,0),
            (2,2)
        ]

        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info("Path Smoothing Node Started.")

    def publish_path(self):
        smoothed = smooth(self.waypoints)

        msg = Path()
        msg.header.frame_id = "map"

        for (x,y) in smoothed:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            msg.poses.append(pose)

        self.pub.publish(msg)
        self.get_logger().info("Published smoothed path.")


def main(args=None):
    rclpy.init(args=args)
    node = PathSmoothingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
