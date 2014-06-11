#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p1.srv import *
from math import fabs, sqrt

from Maze import KnownMaze, UP, DOWN, LEFT, RIGHT, UNKNOWN, OPEN, CLOSED
from MazeSolver import MazeSolver

known_maze = KnownMaze(5, 5, (0,0), (4,4))
solver = MazeSolver(known_maze)
commands = [] 
steps =[]
ROTATE_UP = 0
ROTATE_DOWN = 1
ROTATE_LEFT = 2
ROTATE_RIGHT = 3
MOVE_RIGHT = 4
MOVE_LEFT = 5
MOVE_UP = 6
MOVE_DOWN = 7
SCAN = 8
GOAL_STEP = 9


make_maze_service = None
print_maze_service = None
get_wall_service = None
constant_command_service = None

def send_cmd(arguments):
    while not commands:
        generate_commands()
    return commands.pop(0) 


def make_request(x, y, theta):
	req = GetCommandResponse()
	req.delta_x = x
	req.delta_y = y
	req.delta_theta = theta
	return req

def generate_commands():
    global steps
    if not steps:
        steps.extend(solver.calculate_next_steps())

    static_commands = {
        ROTATE_UP:{
            UP: make_request(0, 0, 0),
            DOWN: make_request(0, 0, 180),
            LEFT: make_request(0, 0, -90),
            RIGHT: make_request(0, 0, 90)
        },

        ROTATE_LEFT:{
            UP: make_request(0, 0, 90),
            DOWN: make_request(0, 0, -90),
            LEFT: make_request(0, 0, 0),
            RIGHT: make_request(0, 0, 180)
            
        },

        ROTATE_RIGHT:{
            UP: make_request(0, 0, -90),
            DOWN: make_request(0, 0, 90),
            LEFT: make_request(0, 0, 180),
            RIGHT: make_request(0, 0, 0)
            
        },
        
        ROTATE_DOWN:{
             UP: make_request(0, 0, 180),
            DOWN: make_request(0, 0, 0),
            LEFT: make_request(0, 0, 90),
            RIGHT: make_request(0, 0, -90)
        },           

        MOVE_UP: make_request(.5, 0, 0),
        MOVE_DOWN: make_request(.5, 0, 0),
        MOVE_LEFT: make_request(0, .5, 0),
        MOVE_RIGHT: make_request(0, .5, 0),
        GOAL_STEP: make_request(0,0,0),
        SCAN: make_request(0,0,0)
    }

    rotate_types = [ROTATE_UP, ROTATE_LEFT, ROTATE_DOWN, ROTATE_RIGHT]
    step = steps.pop(0)
    if step in rotate_types:
        commands.append(static_commands[step][known_maze.get_rosie_direction()])

    else:
        commands.append(static_commands[step])

    out = {UP:"UP", RIGHT:"RIGHT", LEFT:"LEFT", DOWN:"DOWN"}
    if step is ROTATE_UP:
        # Move rosie.
        print "ROTATE UP",
        print "..from ..", out[known_maze.get_rosie_direction()]
        known_maze.set_rosie_direction(UP)
    elif step is ROTATE_LEFT:
        print "ROTATE LEFT",
        print "..from ..", out[known_maze.get_rosie_direction()]
        known_maze.set_rosie_direction(LEFT)
    elif step is ROTATE_DOWN:
        print "ROTATE DOWN",
        print "..from ..", out[known_maze.get_rosie_direction()]
        known_maze.set_rosie_direction(DOWN)
    elif step is ROTATE_RIGHT:
        print "ROTATE RIGHT",
        print "..from ..", out[known_maze.get_rosie_direction()]
        known_maze.set_rosie_direction(RIGHT)
    elif step is SCAN:
        update_known_maze_wall()
    elif step is MOVE_UP:
        known_maze.move_rosie_up()
    elif step is MOVE_DOWN:
        known_maze.move_rosie_down()
    elif step is MOVE_LEFT:
        known_maze.move_rosie_left()
    elif step is MOVE_RIGHT:
        known_maze.move_rosie_right()
    elif step is GOAL_STEP:
        commands.append(make_request(0, 0, 0))
        print commands
        print known_maze.print_maze()
        exit()
    print known_maze.print_maze()


def initialize():
    rospy.init_node("interpreter", anonymous=True)
    rospy.wait_for_service('make_maze')
    rospy.wait_for_service('print_maze')
    rospy.wait_for_service('get_wall')
    s = rospy.Service('rosie_cmd', GetCommand, send_cmd)

    global make_maze_service, print_maze_service, get_wall_service
    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    make_maze_service(5, 5)
    #commands.append(make_request(0.5, 0, 0))
    #	commands.append(make_request(0, 0, 90))
    #	commands.append(make_request(0, 0.5, 0))
    #	commands.append(make_request(0, 0, 90))
    #	commands.append(make_request(0.5, 0, 0))
    #	commands.append(make_request(0, 0, 90))
    rospy.spin()

        
def update_known_maze_wall():
    col, row = known_maze.get_rosie_position()
    direction = known_maze.get_rosie_direction()
    wall_bool = get_wall_service(col, row, direction)
    wall_status = UNKNOWN
    if wall_bool.wall == 1:
        wall_status = CLOSED
    else:
        wall_status = OPEN
    known_maze.set_cell_wall(col, row, direction, wall_status)

if __name__ == "__main__":   
    try: 
        initialize()
    except rospy.ROSInterruptException: pass
