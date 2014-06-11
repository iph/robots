#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *

sub = None

def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    print data.pose.pose

    if(data.pose.pose.position.x <1.0):
        command.linear.x = 0.3
        send_command(command)
    else:
        command.linear.x = 0.0
        send_command(command)
        sub.unregister()



def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    global sub
    sub = rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveForwardOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())
    print "DERRRRRRR?"
    rospy.spin()
    

if __name__ == "__main__":
    initialize()

