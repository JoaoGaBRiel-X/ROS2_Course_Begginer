#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLedState
from my_robot_interfaces.msg import ViewLedState

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel")
        self.led1_ = False
        self.led2_ = False
        self.led3_ = False
        self.server_ = self.create_service(SetLedState, "set_led", self.callback_set_led)
        self.led_panel_state_ = self.create_publisher(ViewLedState, "led_panel_state", 10)
        self.get_logger().info("Led Panel has been started.")

    def callback_set_led(self, request, response):
        if request.led_number == 1:
            self.led1_ = request.state
            response.success = True
        elif request.led_number == 2:
            self.led2_ = request.state
            response.success = True
        elif request.led_number == 3:
            self.led3_ = request.state
            response.success = True
        else:
            response.success = False
        
        if response.success:
            msg = ViewLedState()
            msg.led1 = self.led1_
            msg.led2 = self.led2_
            msg.led3 = self.led3_
            self.led_panel_state_.publish(msg)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()