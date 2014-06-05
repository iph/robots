#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p1.srv import *
from math import fabs, sqrt

from Maze import KnownMaze
from MazeSolver import MazeSolver

known_maze = KnownMaze(5, 5, (0,0), (5,5))
solver = MazeSolver(known_maze)
commands = [] 

ROTATE_UP = 0
ROTATE_DOWN = 1
ROTATE_LEFT = 2
ROTATE_RIGHT = 3
MOVE_RIGHT = 4
MOVE_LEFT = 5
MOVE_UP = 6
MOVE_DOWN = 7
SCAN = 8


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

	commands.append(make_request(0.5, 0, 0))
        #	commands.append(make_request(0, 0, 90))
        #	commands.append(make_request(0, 0.5, 0))
        #	commands.append(make_request(0, 0, 90))
        #	commands.append(make_request(0.5, 0, 0))
        #	commands.append(make_request(0, 0, 90))
        print solver.calculate_next_steps()


	s = rospy.Service('rosie_cmd', GetCommand, send_cmd)

	rospy.spin()

if __name__ == "__main__":   
    try: 
        initialize()
    except rospy.ROSInterruptException: pass
