#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLedState

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery")
        self.battery_state_ = 1
        self.counter_ = 0
        self.timer_ = self.create_timer(1.0, self.routine_example)
        self.get_logger().info("Battery Service has been started.")
    
    def call_set_led_server(self, led_number, state):
        client = self.create_client(SetLedState, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Set Led...")

        request = SetLedState.Request()
        request.led_number = led_number
        request.state = state

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_set_led, led_number=led_number, state=state))

    def callback_call_set_led(self, future, led_number, state):
        try:
            response = future.result()
            self.get_logger().info(f"REQUEST = led_number: {str(led_number)}, state: {state} / RESPONSE = Success: {response.success}")
            if response.success:
                self.battery_state_ = state
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def routine_example(self):
        if self.battery_state_ == True and self.counter_ == 6:
            self.call_set_led_server(led_number=3, state=False)
            self.counter_ = 0
        elif self.battery_state_ == False and self.counter_ == 4:
            self.call_set_led_server(led_number=3, state=True)
            self.counter_ = 0
        else:
            self.counter_ += 1
        
        self.get_logger().info(f"counter {self.counter_}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()