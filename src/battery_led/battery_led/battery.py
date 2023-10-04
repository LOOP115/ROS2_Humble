#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.srv import SetLED


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_status = 1
        self.last_time_battery_state_changed = self.get_current_time()
        self.get_logger().info("Battery Node has been started!")
        self.battery_timer = self.create_timer(0.1, self.check_battery_state)

    def get_current_time(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs / 1000000000.0

    def check_battery_state(self):
        curr_time = self.get_current_time()
        if self.battery_status == 0:
            if curr_time - self.last_time_battery_state_changed > 6.0:
                self.battery_status = 1
                self.get_logger().info("Battery is charged!")
                self.last_time_battery_state_changed = curr_time
                self.call_set_led(self.battery_status)
        elif self.battery_status == 1:
            if curr_time - self.last_time_battery_state_changed > 4.0:
                self.battery_status = 0
                self.get_logger().info("Battery is empty!")
                self.last_time_battery_state_changed = curr_time
                self.call_set_led(self.battery_status)

    def call_set_led(self, status):
        client = self.create_client(SetLED, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server SetLED ...")

        request = SetLED.Request()
        request.battery_status = status
        request.led_index = 0

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_set_led, status=status)
        )

    def callback_call_set_led(self, future, status):
        try:
            response = future.result()
            self.get_logger().info(
                f"Battery status: {status} -> {response.message} {response.led_state.data}"
            )
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
