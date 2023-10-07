#!/usr/bin/env python3

import math
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info("Turtle Controller has started!")
        self.declare_parameter("strategy", "closest")
        self.catch_strategy = self.get_parameter("strategy").value

        self.name = "turtle1"
        self.catch_tolerance = 0.5
        self.pose = None
        self.target = None

        # controller tuning
        self.tx = 2
        self.tz = 6

        self.cmd_vel_publisher = self.create_publisher(
            Twist, f"{self.name}/cmd_vel", 10
        )

        self.pose_subscriber = self.create_subscription(
            Pose, f"{self.name}/pose", self.callback_turtle_pose, 10
        )

        self.alive_turtles_subscriber = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        )

        self.control_loop_timer = self.create_timer(0.01, self.control_loop)

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            # Catch the first prey in the array
            if self.catch_strategy == "first":
                self.target = msg.turtles[0]
            # Catch the closest prey
            else:
                min_dist = math.inf
                for turtle in msg.turtles:
                    tmp_dist = self.get_distance(turtle)
                    if tmp_dist < min_dist:
                        min_dist = tmp_dist
                        self.target = turtle

    def callback_turtle_pose(self, msg):
        self.pose = msg

    def call_catch_turtle(self, name):
        catch_client = self.create_client(CatchTurtle, "/catch_turtle")
        while not catch_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Catch Turtle ...")

        request = CatchTurtle.Request()
        request.name = name
        future = catch_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle, name=name))

    def callback_call_catch_turtle(self, future, name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{name} is caught")
            else:
                self.get_logger().info(f"{name} can not be caught")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def get_distance(self, target):
        dist_x = target.x - self.pose.x
        dist_y = target.y - self.pose.y
        return math.sqrt(dist_x * dist_x + dist_y * dist_y)

    def control_loop(self):
        if self.pose is None or self.target is None:
            return

        dist_x = self.target.x - self.pose.x
        dist_y = self.target.y - self.pose.y
        dist = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        msg = Twist()
        if dist > self.catch_tolerance:
            # position
            msg.linear.x = self.tx * dist

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose.theta
            # Normalise
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = self.tz * diff
        else:
            # target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle(self.target.name)
            self.target = None

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
