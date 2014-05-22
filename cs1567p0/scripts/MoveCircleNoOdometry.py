#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *
from CommandUtils import init_commands, move_circle, stop_command

if __name__ == "__main__":
    init_commands("MoveCircleNoOdometry")
    move_circle(4)
    stop_command()
    
