#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.counter = 0
        self.subscriber = self.create_subscription(
            Int64, "number", self.callback_number, 10
        )
        self.publisher = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Number Counter has been started!")

    def callback_number(self, msg):
        self.get_logger().info(f"{msg.data}")
        self.counter += msg.data
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
