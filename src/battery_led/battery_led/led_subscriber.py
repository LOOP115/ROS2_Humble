#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LEDState


class LEDSubscriberNode(Node):
    def __init__(self):
        super().__init__("led_subscriber")
        self.get_logger().info("LED Subscriber has been started.")
        self.subscriber = self.create_subscription(
            LEDState, "led_state", self.callback_led_state, 10
        )

    def callback_led_state(self, msg):
        self.get_logger().info(f"{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = LEDSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
