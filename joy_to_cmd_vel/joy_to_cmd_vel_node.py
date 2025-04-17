#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import math

class JoyToCmdVelNode(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel_node')
        # Subscribe to the 'joy' topic
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # Publisher for the 'omnibot/cmd_vel' topic
        self.publisher_ = self.create_publisher(TwistStamped, 'omnibot/cmd_vel', 10)

        # Startup deadzone logic
        self.initial_axes = None
        self.startup = True
        self.deadzone_threshold = 0.05

        # Speed dial state
        self.max_linear_scale = 0.3
        self.last_button0 = 0
        self.axis3_baseline = None
        self.dial_active = False

    def joy_callback(self, msg: Joy):
        # Record initial joystick state
        if self.initial_axes is None:
            self.initial_axes = list(msg.axes)
            self.get_logger().info("Initial joystick state recorded; waiting for movement.")
            return

        # Wait for movement beyond deadzone on startup
        if self.startup:
            moved = any(
                abs(curr - base) > self.deadzone_threshold
                for curr, base in zip(msg.axes, self.initial_axes)
            )
            if not moved:
                return
            self.startup = False

        # —— true “press‑then‑move” speed dial ——
        current_b0 = msg.buttons[0] if len(msg.buttons) > 0 else 0

        # Rising edge: button 0 just pressed
        if current_b0 == 1 and self.last_button0 == 0 and len(msg.axes) > 2:
            self.axis3_baseline = msg.axes[2]
            self.dial_active = False

        # While button 0 is held
        if current_b0 == 1 and len(msg.axes) > 2:
            # start dialing only after exceeding deadzone
            if not self.dial_active and abs(msg.axes[2] - self.axis3_baseline) > 0.02:
                self.dial_active = True
            if self.dial_active:
                # map raw axis3 [–1…1] → scale [0.3…1.0]
                self.max_linear_scale = ((msg.axes[2] + 1.0) / 2.0) * 0.7 + 0.3
        else:
            # button 0 released: reset for next press
            self.axis3_baseline = None
            self.dial_active = False

        # remember button state for edge detection
        self.last_button0 = current_b0
        # —————————————————————————————

        # Prepare TwistStamped message
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"

        # Linear velocities scaled
        if len(msg.axes) >= 2:
            twist_msg.twist.linear.x = msg.axes[1] * self.max_linear_scale
            twist_msg.twist.linear.y = msg.axes[0] * self.max_linear_scale
        else:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0

        # Angular velocities via buttons 3 & 4
        left_pressed  = len(msg.buttons) > 3 and msg.buttons[3] == 1
        right_pressed = len(msg.buttons) > 4 and msg.buttons[4] == 1

        if left_pressed and not right_pressed:
            twist_msg.twist.angular.z = math.pi / 2
        elif right_pressed and not left_pressed:
            twist_msg.twist.angular.z = -math.pi / 2
        else:
            twist_msg.twist.angular.z = 0.0

        # Publish cmd_vel
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
