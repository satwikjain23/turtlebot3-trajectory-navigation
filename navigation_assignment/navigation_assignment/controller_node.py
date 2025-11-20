#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import math


class PurePursuitController:
    """
    Implements a Pure Pursuit controller with an important modification:
    ➤ The algorithm keeps track of the last reached trajectory index.
    This prevents the robot from turning backwards toward earlier points.
    """

    def __init__(self, lookahead_distance=0.4):
        # Distance ahead on the trajectory to "chase"
        self.Ld = lookahead_distance

    def find_lookahead_point(self, x, y, traj, current_index, Ld):
        """
        Finds the next point on the trajectory that is at least Ld
        meters ahead of the robot.

        IMPORTANT:
        - Search starts from current_index → prevents robot turning backward.
        """

        for i in range(current_index, len(traj)):
            tx = traj[i].position.x
            ty = traj[i].position.y
            dist = math.hypot(tx - x, ty - y)

            # If point is farther than Ld, return it
            if dist >= Ld:
                return i, tx, ty

        # If no point found (robot is near end), return the last trajectory point
        last = traj[-1]
        return len(traj) - 1, last.position.x, last.position.y

    def compute_cmd(self, x, y, yaw, traj, current_index):
        """
        Computes linear and angular velocity (v, w) using pure pursuit.
        Also returns updated trajectory index.
        """

        # Check if robot is close to the final goal
        final_x = traj[-1].position.x
        final_y = traj[-1].position.y
        dist_to_goal = math.hypot(final_x - x, final_y - y)

        if dist_to_goal < 0.15:
            # Robot reached goal → stop
            return 0.0, 0.0, len(traj) - 1

        # Get next lookahead point
        new_index, goal_x, goal_y = self.find_lookahead_point(
            x, y, traj, current_index, self.Ld)

        # Compute angle to goal
        dx = goal_x - x
        dy = goal_y - y

        angle_to_goal = math.atan2(dy, dx)

        # Angular error = difference between goal direction and robot heading
        heading_error = angle_to_goal - yaw

        # Normalize angle to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error),
                                   math.cos(heading_error))

        # Pure Pursuit control outputs
        v = 0.2  # constant safe forward speed
        w = (heading_error / 3.14) * 1.82  # simple proportional steering

        return v, w, new_index


class ControllerNode(Node):
    """
    ROS2 node:
    - Subscribes to /trajectory (list of poses to follow)
    - Subscribes to /odom (robot pose)
    - Runs Pure Pursuit controller
    - Publishes /cmd_vel to drive robot
    """

    def __init__(self):
        super().__init__("controller_node")

        # Subscribe to generated trajectory
        self.create_subscription(
            PoseArray, "/trajectory", self.trajectory_callback, 10)

        # Subscribe to robot odometry
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)

        # Publisher for robot velocity commands
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Internal storage
        self.traj = []
        self.current_index = 1  # ensures forward progress (prevents backward turning)

        # Robot pose variables updated from /odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Controller instance
        self.controller = PurePursuitController(lookahead_distance=0.4)

    def trajectory_callback(self, msg):
        """ Stores incoming trajectory points """
        self.traj = msg.poses

    def odom_callback(self, msg):
        """ Reads robot pose, then computes and publishes control commands """

        # Extract robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion → yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

        if len(self.traj) == 0:
            # No trajectory yet → do nothing
            return

        # Compute velocity command via Pure Pursuit
        v, w, new_index = self.controller.compute_cmd(
            self.x, self.y, self.yaw,
            self.traj,
            self.current_index  # pass last index to avoid backward motion
        )

        # Update index to move forward along trajectory
        self.current_index = min(new_index, len(self.traj) - 1)

        # Build Twist command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w

        # Publish /cmd_vel
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
