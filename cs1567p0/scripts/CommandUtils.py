#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *

# Speed at .4 rosadians
DEGREES_PER_SECOND = 360.0/20.55
send_command = None

def degrees_to_seconds(degrees):
    return 0.051167 * degrees + 2.35

def init_commands(name):
    """Always run this command at the start of the program, and only once."""
    rospy.init_node(name, anonymous=True)
    rospy.wait_for_service('constant_command')
    global send_command
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)


# Rosies are the unit veocity that the robot takes as input. The robot's name is rosy.
def meters_to_rosies(meters, seconds):
    return 1.2 * (meters / seconds)


def move_forward(dist_in_meters, seconds):
    command = Twist()
    try:
        command.linear.x = meters_to_rosies(dist_in_meters, seconds)
        response = send_command(command)
        rospy.sleep(seconds)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def move_circle(dist_in_meters):
    seconds = 17.95
    command = Twist()
    try:
        command.linear.x = meters_to_rosies(dist_in_meters, seconds)
        command.angular.z = -.4
        response = send_command(command)
        rospy.sleep(seconds)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        

def rotate_90_degrees_counter_clockwise():
    rotate_command(90.0)

def rotate_90_degrees_clockwise():
    rotate_command(90.0, clockwise = True)



def rotate_command(degrees, clockwise = False):
    ''' Takes in degrees between 0-360 and will rotate the robot that amount counter-clockwise. If you want the robot to rotate clockwise:

rotate_command(90, clockwise=true)
rotate_command(90) # Rotates back counter-clockwise
'''
    command = Twist()
    try:

        command.angular.z = .4
        if clockwise:
            command.angular.z *= -1.0
        response = send_command(command)
        #seconds = degrees / DEGREES_PER_SECOND
        seconds = degrees_to_seconds(degrees)
        print seconds
        rospy.sleep(seconds)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def stop_command():
    """ A command that makes the robot stop all angular and linear movement."""
    command = Twist()

    try:
        command.linear.x = 0.0
        command.angular.z = 0.0 # todo: Do we need to remove this?
        send_command(command)
	rospy.sleep(.5);
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
