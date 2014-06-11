#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from cs1567p0.srv import *
from CommandUtils import move_forward, init_commands, stop_command

if __name__ == "__main__":
   init_commands("MoveForwardNoOdometry")
   move_forward(1.0, 7.0)
   stop_command()

