#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from me416_utilities import MotorSpeedLeft, MotorSpeedRight

class MotorCommand(Node):
    """Node that converts Twist commands to motor speed commands."""

    def __init__(self):
        """Initialize the node, subscribers, publishers, and motor controllers."""
        super().__init__('motor_command_node')
        
        # Initialize motor controllers
        self.motor_left = MotorSpeedLeft()
        self.motor_right = MotorSpeedRight()
        
        # Create subscriber for cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publisher for motor_speeds
        self.publisher = self.create_publisher(
            Twist,
            'motor_speeds',
            10
        )
        
        self.get_logger().info('Motor command node initialized')

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor speeds using differential drive.
        
        Args:
            msg (Twist): Incoming Twist message containing linear and angular velocities
        """
        # Extract linear and angular velocities
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive calculation
        left_speed = linear - angular
        right_speed = linear + angular
        
        # Set motor speeds
        self.motor_left.set_speed(left_speed)
        self.motor_right.set_speed(right_speed)
        
        # Publish motor speeds
        motor_msg = Twist()
        motor_msg.linear.x = left_speed
        motor_msg.linear.y = right_speed
        self.publisher.publish(motor_msg)
        
        self.get_logger().info(f'Set speeds - Left: {left_speed:.2f}, Right: {right_speed:.2f}')


def main(args=None):
    """Main entry point for the ROS node."""
    rclpy.init(args=args)
    node = MotorCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
