#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LEDState
from my_robot_interfaces.srv import SetLED


class LEDNode(Node):
    def __init__(self):
        super().__init__("led")
        self.get_logger().info("LED has been started!")
        self.declare_parameter("led_state", [0, 0, 0])
        self.led_state = self.get_parameter("led_state").value

        self.set_led_server = self.create_service(
            SetLED, "set_led", self.callback_set_led
        )

        self.led_state_publisher = self.create_publisher(LEDState, "led_state", 10)
        self.led_state_publish_timer = self.create_timer(1.0, self.publish_led_state)

    def callback_set_led(self, request, response):
        i = request.led_index
        if request.battery_status == 0:
            self.led_state[i] = 1
            curr_led_state = LEDState()
            curr_led_state.data = self.led_state
            response.led_state = curr_led_state
            response.message = f"LED{i} on"
        elif request.battery_status == 1:
            self.led_state[i] = 0
            curr_led_state = LEDState()
            curr_led_state.data = self.led_state
            response.led_state = curr_led_state
            response.message = f"LED{i} off"
        self.publish_led_state()
        return response

    def publish_led_state(self):
        msg = LEDState()
        msg.data = self.led_state
        self.led_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LEDNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
