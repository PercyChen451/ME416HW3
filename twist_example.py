""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist


def twist_fill():
    '''
    Fill a twist message with non-zero values
    '''
    # This is a stub. Substitute with your own code
    msg = Twist()
    # Set linear velocity arbitrary 
    msg.linear.x = 1.5          
    msg.linear.y = 0.2 
    msg.linear.z = 0.1  
    # Set angular velocity arbitrary 
    msg.angular.x = 0.3  
    msg.angular.y = 0.4 
    msg.angular.z = 1.0 

    return msg