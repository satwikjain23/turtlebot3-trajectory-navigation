#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


class PurePursuitController:
    """
    Implements Pure Pursuit controller that tracks progress forward
    through the trajectory using current_index.
    """

    def __init__(self, lookahead_distance=0.4):
        self.Ld = lookahead_distance

    def find_lookahead_point(self, x, y, traj, current_index, Ld):
        """Find the next point >= Ld ahead of current robot position"""

        for i in range(current_index, len(traj)):
            tx = traj[i].position.x
            ty = traj[i].position.y

            dist = math.hypot(tx - x, ty - y)
            if dist >= Ld:
                return i, tx, ty

        # End of trajectory
        last = traj[-1]
        return len(traj) - 1, last.position.x, last.position.y

    def compute_cmd(self, x, y, yaw, traj, current_index):
        """Compute pure pursuit velocity commands"""

        # Check goal reached
        final_x = traj[-1].position.x
        final_y = traj[-1].position.y
        dist_to_goal = math.hypot(final_x - x, final_y - y)

        if dist_to_goal < 0.15:
            return 0.0, 0.0, len(traj) - 1

        # Find lookahead target
        new_index, goal_x, goal_y = self.find_lookahead_point(
            x, y, traj, current_index, self.Ld
        )

        dx = goal_x - x
        dy = goal_y - y
        angle_to_goal = math.atan2(dy, dx)

        heading_error = angle_to_goal - yaw
        heading_error = math.atan2(math.sin(heading_error),
                                   math.cos(heading_error))

        # Output velocities
        v = 0.20
        w = (heading_error / 3.14) * 1.82

        return v, w, new_index


class ControllerNode(Node):
    """
    Pure Pursuit + Reactive Obstacle Avoidance
    """

    def __init__(self):
        super().__init__("controller_node")

        # Subscriptions
        self.create_subscription(PoseArray, "/trajectory",
                                 self.trajectory_callback, 10)
        self.create_subscription(Odometry, "/odom",
                                 self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan",
                                 self.lidar_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Internal state
        self.traj = []
        self.current_index = 1

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # LIDAR data
        self.min_front_dist = 999.0

        # Obstacle thresholds
        self.safe_distance = 0.8   # start avoiding gently
        self.danger_distance = 0.3 # emergency rotate

        self.controller = PurePursuitController(0.4)

        self.get_logger().info("Controller Node with Obstacle Avoidance Started.")

    # ---------------------- TRAJECTORY CALLBACK ----------------------
    def trajectory_callback(self, msg):
        self.traj = msg.poses

    # ---------------------- LIDAR CALLBACK ---------------------------
    def lidar_callback(self, msg: LaserScan):
        # Extract front 20 degrees of scan
        ranges = msg.ranges

        # Replace inf → large number
        clean = [r if not math.isinf(r) else 10.0 for r in ranges]

        # Extract center 40 degrees (20 left & 20 right)
        
        window = clean[0: 50]
        windowr= clean[-20:-1]

        self.min_front_dist = min(min(window),min(windowr))

    # ---------------------- ODOM CALLBACK ----------------------------
    def odom_callback(self, msg):
        # Update robot position and orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

        # No trajectory yet
        if len(self.traj) == 0:
            return

        # ---------------------
        # 1. OBSTACLE AVOIDANCE LOGIC
        # ---------------------

        cmd = Twist()

        # EMERGENCY STOP + IN-PLACE ROTATION
        if self.min_front_dist < self.danger_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.9   # rotate to avoid collision
            self.cmd_pub.publish(cmd)
            print("danger  0  0.9")
            return

        # # GENTLE AVOIDANCE: slow + curve slightly
        # if self.min_front_dist < self.safe_distance:
        #     cmd.linear.x = 0.10
        #     cmd.angular.z = 0.8  # slight steer
        #     print("safe  0.1  0.8")
        #     self.cmd_pub.publish(cmd)

        #     return

        # ---------------------
        # 2. NO OBSTACLE → PURE PURSUIT
        # ---------------------
        v, w, new_index = self.controller.compute_cmd(
            self.x, self.y, self.yaw,
            self.traj,
            self.current_index
        )

        self.current_index = min(new_index, len(self.traj)-1)

        cmd.linear.x = v
        cmd.angular.z = w
        if self.min_front_dist < self.safe_distance:
            cmd.linear.x = 0.10
            cmd.angular.z = -0.2
            print("safe")
        print("normal v=",v," w=",w)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
