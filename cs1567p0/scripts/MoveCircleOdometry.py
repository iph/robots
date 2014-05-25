#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p0.srv import *
from time import time
from math import sqrt
sub = None
current_action = None
current_goal = None
chain = []
action_chain = []
action_start_time = 0


def odometry_callback(data):
    command = Twist()
    send_command = rospy.ServiceProxy('constant_command', ConstantCommand)

    global current_goal
    global current_action
    global action_start_time
    print data.pose.pose

    if current_goal(data.pose.pose):
        current_action(command)
        send_command(command)
    elif len(chain) > 0:
        action_start_time = time()
        command.linear.x = 0.0
        command.angular.z = 0.0
        send_command(command)
        current_goal = chain.pop(0)
        current_action = action_chain.pop(0)
    else:
        command.linear.x = 0.0
        command.angular.z = 0.0
        send_command(command)
        sub.unregister()

def initialize():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

    global action_start_time

    rotate_90_and_move_goal = lambda pose: pose.orientation.z < sqrt(0.5) and pose.position.x < 1.0
    rotate_180_and_move_goal = lambda pose: pose.orientation.z > 0 and pose.position.y < 1.0
    rotate_270_and_move_goal = lambda pose: pose.orientation.z < -sqrt(0.5) and pose.position.x > 0.0
    rotate_360_and_move_goal = lambda pose: pose.orientation.z <= 0 and pose.position.y > 0.0
    null_goal = lambda pose: (time() - action_start_time) < 0.5 
    
    global chain
    global current_goal
    global current_action
    global action_chain


    def rotate_and_move_action(command):
        command.linear.x = .2
        command.angular.z = .3
    def null_action(command):
        command.linear.x = 0.0
        command.angular.z = 0.0 

    chain.append(rotate_90_and_move_goal)
    chain.append(null_goal)
    chain.append(rotate_180_and_move_goal)
    chain.append(null_goal)
    chain.append(rotate_270_and_move_goal)
    chain.append(null_goal)
    chain.append(rotate_360_and_move_goal)
    chain.append(null_goal)
    
    action_chain.append(rotate_and_move_action)
    action_chain.append(null_action)
    action_chain.append(rotate_and_move_action)
    action_chain.append(null_action)
    action_chain.append(rotate_and_move_action)
    action_chain.append(null_action)
    action_chain.append(rotate_and_move_action)
    action_chain.append(null_action)

    current_goal = chain.pop(0)
    current_action = action_chain.pop(0)

    global sub
    sub = rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.init_node('MoveSquareOdometry', anonymous=True)
    rospy.wait_for_service('constant_command')
    while pub.get_num_connections() < 1:
        rospy.sleep(0.1)
    pub.publish(Empty())

    pub.publish(Empty())
    rospy.spin()
    

if __name__ == "__main__":
    initialize()

