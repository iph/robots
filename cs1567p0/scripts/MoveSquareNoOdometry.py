#!/usr/bin/env python
from CommandUtils import init_commands, move_forward, stop_command, rotate_command
import sys
import rospy

if __name__ == "__main__":
    rotate_amount = 90.0
    if len(sys.argv) > 1:
        rotate_amount = float(sys.argv[1])

    init_commands('MoveSquareNoOdometry')
    for i in range(4):
        move_forward(1.0, 3.0)
        stop_command()
        rotate_command(rotate_amount, True)
        stop_command()

    stop_command()

