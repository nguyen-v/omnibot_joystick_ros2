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
        
        # Initialize the max linear velocity scaling factor to the minimum (0.3)
        self.max_linear_scale = 0.3
        # This variable will record the first value of the third axis when button 0 is pressed.
        self.axis3_baseline = None

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

        # Dynamic scaling:
        # Check if button 0 (first button) is pressed and that we have at least 3 axes.
        if len(msg.buttons) > 0 and msg.buttons[0] == 1 and len(msg.axes) > 2:
            # If this is the first time the button is pressed, record the baseline for axis 2.
            if self.axis3_baseline is None:
                self.axis3_baseline = msg.axes[2]
            # If the axis value has changed relative to the recorded baseline, update the scale.
            elif msg.axes[2] != self.axis3_baseline:
                # Map axis 2 value from [-1, 1] to a scaling factor in [0.3, 1.0].
                self.max_linear_scale = ((msg.axes[2] + 1) / 2) * 0.7 + 0.3
                # Update the baseline to the new value to detect further changes.
                self.axis3_baseline = msg.axes[2]
        elif msg.buttons[7] == 1:
            self.max_linear_scale = 0.3
        else:
            # Reset the baseline when button 0 is not pressed.
            self.axis3_baseline = None

        # Prepare the TwistStamped message.
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"  # Adjust frame_id if necessary

        # Map linear velocities and apply the current scaling factor.
        # twist.linear.x is the second axis (index 1)
        # twist.linear.y is the first axis (index 0)
        if len(msg.axes) >= 2:
            twist_msg.twist.linear.x = msg.axes[1] * self.max_linear_scale
            twist_msg.twist.linear.y = msg.axes[0] * self.max_linear_scale
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
