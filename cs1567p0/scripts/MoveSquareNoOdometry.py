#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *
from MoveForwardNoOdometry import move_forward
# Speed at .4 rosadians
DEGREES_PER_SECOND = 360.0/20.55
send_command = None 

def init_commands(name):
    rospy.init_node(name, anonymous=True)
    rospy.wait_for_service('constant_command')
    global send_command
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

def rotate_90_degrees_counter_clockwise():
    rotate_command(90)

def rotate_90_degrees_clockwise():
    rotate_command(126.0, clockwise = True)



def rotate_command(degrees, clockwise = False):
    ''' Description here. '''
    command = Twist()
    try:

        command.angular.z = .4
        if clockwise:
            command.angular.z *= -1.0
        response = send_command(command)
        seconds = degrees / DEGREES_PER_SECOND

        rospy.sleep(seconds)
        command.angular.z = 0.0

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def stop_command():
    command = Twist()
    
    try:
        command.linear.x = 0.0
        command.angular.z = 0.0
        send_command(command)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def move_square():
    rospy.init_node('MoveSquareNoOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    command = Twist()
    
    try:
        send_command = rospy.ServiceProxy('constant_command', ConstantCommand)
        command.linear.x = 0.5
        response = send_command(command)
        rospy.sleep(0.5)
        command.linear.x = 0.0
        response = send_command(command)
        print response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    init_commands('MoveSquareNoOdometry')
    for i in range(4):
        move_forward(1.0, 7.0)
        stop_command()
        rotate_command(float(sys.argv[1]))
        print "hi"
        #rotate_90_degrees_counter_clockwise()
        stop_command()
