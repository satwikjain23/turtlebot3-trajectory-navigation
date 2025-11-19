#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import math


class PurePursuitController:

    def __init__(self, lookahead_distance=0.4):
        self.Ld = lookahead_distance

    def find_lookahead_point(self, x, y, traj, current_index, Ld):
        """
        Find the next trajectory point *ahead* of the robot,
        starting search from current_index.
        """
        print("len=", len(traj))
        for i in range(current_index, len(traj)):
            tx = traj[i].position.x
            ty = traj[i].position.y
            dist = math.hypot(tx - x, ty - y)

            if dist >= Ld:
                return i, tx, ty

        # No more points ahead → return last point
        last = traj[-1]
        return len(traj) - 1, last.position.x, last.position.y

    def compute_cmd(self, x, y, yaw, traj, current_index):
        final_x = traj[-1].position.x
        final_y = traj[-1].position.y
        dist_to_goal = math.hypot(final_x - x, final_y - y)

        if dist_to_goal < 0.15:
            return 0.0, 0.0, len(traj) - 1 

        new_index, goal_x, goal_y = self.find_lookahead_point(
            x, y, traj, current_index, self.Ld)

        # Compute direction to goal point
        dx = goal_x - x
        dy = goal_y - y
        print("-------------------------")
        print("new_index=",new_index)
        print("x=",x," y=",y," goalx=",goal_x, " goaly=",goal_y)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - yaw
        print("head=",heading_error)

        # Normalize angle
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        print("headn=", heading_error)

        # Output velocities
        v = 0.2  # safe linear speed
        w = (heading_error/3.14)*1.82
        
        return v, w, new_index


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # Subscriptions
        self.create_subscription(PoseArray, "/trajectory",
                                 self.trajectory_callback, 10)
        self.create_subscription(Odometry, "/odom",
                                 self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Internal state
        self.traj = []
        self.current_index = 1  # <--- FIX: remembers progress

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.controller = PurePursuitController(lookahead_distance=0.4)

        #self.get_logger().info("Controller Node with forward-progress Pure Pursuit initialized.")

    def trajectory_callback(self, msg):
        self.traj = msg.poses
        

    def odom_callback(self, msg):
        # Update robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)
        #print("yaw=",math.degrees(self.yaw))

        if len(self.traj) == 0:
            return

        # Compute control command
        v, w, new_index = self.controller.compute_cmd(
            self.x, self.y, self.yaw,
            self.traj,
            self.current_index  # always forward from this index
        )

        # Update index for next iteration
        self.current_index = min(new_index, len(self.traj) - 1)

        # Publish command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
