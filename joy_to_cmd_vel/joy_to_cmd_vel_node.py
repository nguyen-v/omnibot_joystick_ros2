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
        # To record the initial joystick state
        self.initial_axes = None
        # Flag to indicate the startup phase (deadzone applies only during startup)
        self.startup = True
        # Threshold for detecting a change in the joystick's position
        self.deadzone_threshold = 0.05

    def joy_callback(self, msg: Joy):
        # On the very first message, record the baseline and do not publish.
        if self.initial_axes is None:
            self.initial_axes = list(msg.axes)
            self.get_logger().info("Initial joystick state recorded; waiting for movement.")
            return

        # During startup, only publish when movement exceeds the threshold.
        if self.startup:
            movement_detected = any(
                abs(current - base) > self.deadzone_threshold
                for current, base in zip(msg.axes, self.initial_axes)
            )
            if not movement_detected:
                return  # Still within the startup deadzone, so do nothing.
            else:
                self.startup = False  # Movement detected; exit the startup phase.

        # Prepare the TwistStamped message.
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"  # Adjust frame_id if necessary

        # Map linear velocities:
        # twist.linear.x = second element of joy.axes (index 1)
        # twist.linear.y = first element of joy.axes (index 0)
        if len(msg.axes) >= 2:
            twist_msg.twist.linear.x = msg.axes[1]
            twist_msg.twist.linear.y = msg.axes[0]
        else:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0

        # Determine angular velocity based on buttons:
        # 4th button -> index 3; 5th button -> index 4
        left_button_pressed = len(msg.buttons) > 3 and msg.buttons[3] == 1
        right_button_pressed = len(msg.buttons) > 4 and msg.buttons[4] == 1

        if left_button_pressed and not right_button_pressed:
            twist_msg.twist.angular.z = math.pi / 2
        elif right_button_pressed and not left_button_pressed:
            twist_msg.twist.angular.z = -math.pi / 2
        else:
            twist_msg.twist.angular.z = 0.0

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
