#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p3.srv import *
from math import fabs, sqrt

commands = [] 

def send_cmd(arguments):

	cmd = commands.pop(0) if len(commands) > 0 else GetCommandResponse()
	return cmd

def make_request(x, y, theta):
	req = GetCommandResponse()
	req.delta_x = x
	req.delta_y = y
	req.delta_theta = theta
	return req

def initialize():
	rospy.init_node("goal_test", anonymous=True)
	global commands
	#commands.append(make_request(0.5, 0, 0))
	commands.append(make_request(0, 0, 90))
	#commands.append(make_request(0, 0.5, 0))
	#commands.append(make_request(0.5, 0, 0))
	#commands.append(make_request(0.5, 0, 0))
	#commands.append(make_request(0, 0, -90))

	s = rospy.Service('rosie_cmd', GetCommand, send_cmd)

	rospy.spin()

if __name__ == "__main__":   
    try: 
        initialize()
    except rospy.ROSInterruptException: pass
