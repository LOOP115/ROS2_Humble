#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.get_logger().info("Number Counter has been started!")
        self.counter = 0

        self.subscriber = self.create_subscription(
            Int64, "number", self.callback_number, 10
        )

        self.publisher = self.create_publisher(Int64, "number_count", 10)

        self.reset_counter_server = self.create_service(
            SetBool, "reset_counter", self.callback_reset_number_count
        )

    def callback_number(self, msg):
        self.get_logger().info(f"{msg.data}")
        self.counter += msg.data
        msg = Int64()
        msg.data = self.counter
        self.publisher.publish(msg)

    def callback_reset_number_count(self, request, response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "Counter has been reset!"
        else:
            response.success = False
            response.message = "Counter has not been reset!"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
