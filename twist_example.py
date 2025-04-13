""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist
def twist_fill():
    """Create and populate a Twist message with non-zero values.
    Returns:
        Twist: A Twist message with all fields set to non-zero floating point values
    """
    msg = Twist()
    # Set linear velocity with arbitrary values
    msg.linear.x = 1.5
    msg.linear.y = 0.2
    msg.linear.z = 0.1
    # Set linear velocity with arbitrary values
    msg.angular.x = 0.3
    msg.angular.y = 0.4
    msg.angular.z = 1.0
    return msg


if __name__ == '__main__':
    twist_msg = twist_fill()
    print("Generated Twist message:\n", twist_msg)
