#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p3.srv import *
from math import fabs, sqrt

command_code = None
epsilon = 0.1
get_goal_service = None
send_cmd_service = None


def null_command(pose):
    def null_action(command):
        pass

    pursue(True, null_action)

def rotate_counter(pose):
    def rotate_counter_action(command):
        command.angular.z = 0.3

    pursue(pose.orientation.z > sqrt(0.5), rotate_counter_action)

def rotate(pose):
    def rotate_action(command):
        command.angular.z = -0.3

    pursue(pose.orientation.z < -sqrt(0.5), rotate_action)

def forward(pose):
    def forward_action(command):
        command.linear.x = 0.3

    pursue(pose.position.x >= 0.5, forward_action)

def pursue(goal, action):
    command = Twist()

    if goal:
        global command_code
        rospy.loginfo('Completed command: {0}'.format(command_code))
        command_code = None
    else:
        action(command)

    send(command)

def send(command):
    global send_cmd_service
    send_cmd_service(command)

def odometry_callback(odom_data):
    
    action_dict = {
        0           : null_command,
        1 : rotate_counter,
        2         : rotate,
        3        : forward 
    }
    rospy.loginfo(odom_data.pose.pose)
    global command_code 
    if command_code is None:
        clear_odometry()
        command_code = get_goal_service().command
    else:        
        action_dict[command_code](odom_data.pose.pose)

def within(a, b, eps):
    return fabs(a - b) < eps

def initialize():
    rospy.init_node('Rosie', anonymous=True)
    rospy.wait_for_service('constant_command')
    #rospy.wait_for_service('maze_solver')
    rospy.wait_for_service('get_goal')

    clear_odometry()

    odometry_sub = rospy.Subscriber('/odom', Odometry, odometry_callback)

    global get_goal_service
    get_goal_service = rospy.ServiceProxy('get_goal', GetNextGoal)

    global send_cmd_service
    send_cmd_service = rospy.ServiceProxy('constant_command', ConstantCommand)

    rospy.spin()


def clear_odometry():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())

if __name__ == "__main__":
    initialize()
