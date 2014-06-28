#!/usr/bin/python
import rospy
from cs1567p3.srv import *
from std_srvs.srv import * 
import sys
from random import randint
from random import shuffle

MazeGrid = [[]]
Columns = 0
Rows = 0

LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3
OPEN = 0
CLOSED = 1
FILE_NAME = "default.txt" 

class Cell(object):
    # Could abstract into a non-class, but oh well. 
    def __init__(self, left=CLOSED, right=CLOSED, up=CLOSED, down=CLOSED):
        self.walls = [left, right, up, down]

    def set_wall(self, direction, status):
        self.walls[direction] = status
    
    def get_wall(self, direction):
        return self.walls[direction]



def createGrid(rows,cols):
    # initialize to a bunch of closed cells.
    global MazeGrid, Columns, Rows
    MazeGrid = [ [Cell() for x in range(rows)] for y in range(cols)]
    

def getWall(col, row, direction):
    return MazeGrid[col][row].get_wall(direction) == CLOSED

def makeMaze():
    contents = None
    with open(FILE_NAME) as f:
        contents = f.readlines()
    rows = len(contents)
    
    cols = -1 # There is always 1 too many '+' symbols.
    for char in contents[0]:
        if char == '+':
            cols += 1

    createGrid(rows, cols)
    # A fully closed off cell looks like this:
    #    +--+
    #    |  |
    #    +--+
    # Hence, left is | and top is -- for checking closed status.
    cell_closed_left = lambda substr: substr is "|"
    cell_closed_top = lambda substr: substr is "--"


    # Skip top and last row since they will automatically be closed
    for top_and_bottom in range(2, len(contents) - 1, 2): 
        row = top_and_bottom / 2 + 1
        for col in range(Columns):
            str_start_pos = col * 3 + 1 # 3 characters per cell, + 1 for start of '-'
            str_end_pos = str_start_pos + 2
            if cell_closed_top(contents[top_and_bottom][str_start_pos:str_end_pos]):
                MazeGrid[col][row - 1].setWall(DOWN, CLOSED)
                MazeGrid[col][row].setWall(UP, CLOSED)
            else:
                MazeGrid[col][row - 1].setWall(DOWN, OPEN)
                MazeGrid[col][row].setWall(UP, OPEN)
                

    for mid_area in range(1, len(contents), 2): 
        row = mid_area / 2
        for col in range(1, Columns - 1): # Skip first and last, since they are closed.
            str_start_pos = col * 3 
            str_end_pos = str_start_pos + 1
            if cell_closed_left(contents[mid_area][str_start_pos:str_end_pos]):
                MazeGrid[col - 1][row].setWall(RIGHT, CLOSED)
                MazeGrid[col][row].setWall(LEFT, CLOSED)
            else:
                MazeGrid[col - 1][row].setWall(RIGHT, OPEN)
                MazeGrid[col][row].setWall(LEFT, OPEN)


def printMaze(self):
    # If one feels daring, they can change the strings for open and closed status. WOOOO
    WALL_SYMBOLS = {
        UP: {
            OPEN:    "  ",
            CLOSED:  "--"
        },
        DOWN: {
            OPEN:    "  ",
            CLOSED:  "--"
        },
        LEFT: {
            OPEN:    " ",
            CLOSED:  "|"
        },
        RIGHT: {
            OPEN:    " ",
            CLOSED:  "|"
        }
        
    }
    s = "+"
    # Top row.
    for j in range(Columns):
        s += WALL_SYMBOLS[UP][MazeGrid[j][0].get_wall(UP)] + "+"
    s += "\n"
    for i in range(Rows):
        # Far left most wall.
        s += WALL_SYMBOLS[LEFT][MazeGrid[0][i].get_wall(LEFT)]

        for j in xrange(0, Columns):
            s += "  "
            # All right walls are the left wall of the next column!
            s += WALL_SYMBOLS[RIGHT][MazeGrid[j][i].get_wall(RIGHT)]
        s += "\n"
        s += "+"
        
        # All bottoms are tops of the next row!
        for j in range(0, Columns):
            s += WALL_SYMBOLS[DOWN][MazeGrid[j][i].get_wall(DOWN)] + "+"
        s += "\n"
    return s

def make_maze(parameters):
    # We dont care about the parameters, BWAHAHAHA
    makeMaze()
    return 1

def print_maze(parameters):
    print printMaze()
    return EmptyResponse()

def get_maze_wall(parameters):
    if getWall(parameters.col, parameters.row, parameters.direction):
        return 1
    return 0

def initialize_commands():
    rospy.init_node('mazegenerationnode', anonymous=True)
    
    s1 = rospy.Service('make_maze', MakeNewMaze, make_maze)
    s2 = rospy.Service('print_maze', Empty, print_maze)
    s3 = rospy.Service('get_wall', GetMazeWall, get_maze_wall)

    rospy.spin()

if __name__ == "__main__":   
    try: 
        if len(sys.argv) > 1:
            global FILE_NAME
            FILE_NAME = sys.argv[1]

        initialize_commands()
    except rospy.ROSInterruptException: pass

