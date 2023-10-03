#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__("number_subscriber")
        self.get_logger().info("Number Subscriber has been started!")
        self.subscriber = self.create_subscription(
            Int64, "number_count", self.callback_number_counter, 10
        )

    def callback_number_counter(self, msg):
        self.get_logger().info(f"{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
