#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        # Parameters
        self.declare_parameter("num", 2)
        self.num = self.get_parameter("num").value
        self.declare_parameter("hz", 1.0)
        self.hz = self.get_parameter("hz").value

        self.publisher = self.create_publisher(Int64, "number", 10)
        self.timer = self.create_timer(1.0 / self.hz, self.publish_num)
        self.get_logger().info("Number Publisher has been started!")

    def publish_num(self):
        msg = Int64()
        msg.data = self.num
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
