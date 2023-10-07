#!/usr/bin/env python3

import math
import random
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.get_logger().info("Turtle Spawner has started!")
        self.declare_parameter("hz", 1.0)
        self.spawn_hz = self.get_parameter("hz").value
        self.declare_parameter("name_prefix", "prey")
        self.name_prefix = self.get_parameter("name_prefix").value

        self.counter = 0
        self.alive_turtles = []

        self.alive_turtles_publisher = self.create_publisher(
            TurtleArray, "alive_turtles", 10
        )

        self.catch_turtle_server = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        self.timer = self.create_timer(1.0 / self.spawn_hz, self.spawn_turtle)

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles
        self.alive_turtles_publisher.publish(msg)

    def callback_catch_turtle(self, request, response):
        self.call_kill(request.name)
        response.success = True
        return response

    def spawn_turtle(self):
        self.counter += 1
        # (0, 11)
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        name = f"{self.name_prefix}{self.counter}"
        self.call_spawn(x, y, theta, name)

    def call_spawn(self, x, y, theta, name):
        spawn_client = self.create_client(Spawn, "/spawn")
        while not spawn_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Spawn ...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        future = spawn_client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, x=x, y=y, theta=theta, name=name)
        )

    def callback_call_spawn(self, future, x, y, theta, name):
        try:
            response = future.result()
            if response.name == name:
                self.get_logger().info(
                    f"{response.name} spawned at ({x}, {y}, {theta})"
                )
                new_turtle = Turtle()
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                new_turtle.name = name
                self.alive_turtles.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def call_kill(self, name):
        kill_client = self.create_client(Kill, "/kill")
        while not kill_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service Kill ...")

        request = Kill.Request()
        request.name = name
        future = kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill, name=name))

    def callback_call_kill(self, future, name):
        try:
            future.result()
            for i, turtle in enumerate(self.alive_turtles):
                if turtle.name == name:
                    del self.alive_turtles[i]
                    self.get_logger().info(f"Kill {name}")
                    self.publish_alive_turtles()
                    break

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
