#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel:
    def __init__(self):
        rospy.init_node('joy_to_cmd_vel')
        
        # Publisher to /joy/cmd_vel
        self.cmd_vel_pub = rospy.Publisher('/joy/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to /joy
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        # Parameters to scale the joystick input to velocity
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.angular_scale = rospy.get_param('~angular_scale', 1.0)

    def joy_callback(self, joy_msg):
        twist = Twist()
        # Mapping joystick axes to velocity commands
        twist.linear.x = joy_msg.axes[4] * self.linear_scale  # forward/backward
        twist.linear.y = joy_msg.axes[3] * self.linear_scale  # left/right
        twist.linear.z = joy_msg.axes[1] * self.linear_scale  # up/down
        twist.angular.z = joy_msg.axes[0] * self.angular_scale  # yaw
        
        # Publish the command
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        JoyToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass