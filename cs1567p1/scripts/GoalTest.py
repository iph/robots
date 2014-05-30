#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p1.srv import *
from math import fabs, sqrt

commands = [] 

def send_goal(arguments):
	command = 0
	if commands:
		command = commands.pop()
	return command

def initialize():
	rospy.init_node("goal_test", anonymous=True)
	commands.append(3)
	commands.append(2)
	commands.append(1)

	s = rospy.Service('get_goal', GetNextGoal, send_goal)

	rospy.spin()

if __name__ == "__main__":   
    try: 
        initialize()
    except rospy.ROSInterruptException: pass
