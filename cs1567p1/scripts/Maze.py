#!/usr/bin/python
import rospy
from cs1567p1.srv import *
from std_srvs.srv import * 

LEFT = 0
RIGHT = 1
UP = 2
DOWN = 3


CLOSED = 0
OPEN = 1
UNKNOWN = 2


class Cell(object):
    def __init__(self, left=UNKNOWN, right=UNKNOWN, up=UNKNOWN, down=UNKNOWN):
        self.walls = [left, right, up, down]

    def get_open_walls(self):
        """Returns a list of walls that are not unknown/closed (aka open)"""
        open_walls = []
        iter = 0
        for wall in self.walls:
            if wall is OPEN:
                open_walls.append(iter)
            iter += 1

    def set_wall(self, direction, status):
        """direction should be defined above{LEFT, RIGHT, UP, DOWN}, same for status {OPEN, CLOSED, UNKNOWN}"""
        self.walls[direction] = status

    
    def get_wall(self, direction):
        return self.walls[direction]


class KnownMaze(object):
    """A mechanism to keep track of where rosie is in the maze, the current status of the maze
and the goal in the maze."""
    def __init__(self, col, row, start_coord, goal_coord):
        self.current_col, self.current_row = start_coord
        self.goal_col, self.goal_row = goal_coord
        self.max_cols = col
        self.max_rows = row
        self.current_direction = UP
        # Initialize an unknown grid of size row, col
        self.grid = []
        for i in range(col):
            self.grid.append([])
            for _ in range(row):
                self.grid[i].append(Cell())

    def get_cell(self, col, row):
        return self.grid[col][row]

    def get_current_cell(self):
        return self.get_cell(self.current_col, self.current_row)

    def move_rosie_forward(self):
        """Moves rosie in the direction she is currently facing. Check walls before movement."""
        direction = self.current_direction
        if direction is UP:
            self.current_row -= 1
        if direction is DOWN:
            self.current_row += 1
        if direction is LEFT:
            self.current_col -= 1
        if direction is RIGHT:
            self.current_col += 1


    def warp_position(self, new_position):
        """Mostly will be used for debugging. """
        self.current_col ,self.current_row = new_position

    def get_rosie_position(self):
        return (self.current_col, self.current_row)

    def get_rosie_direction(self):
        return self.current_direction
        
    def set_rosie_direction(self, direction):
        self.current_direction = direction

    def set_cell_wall(self, col, row, direction, status):
        self.grid[col][row].set_wall(direction, status)

    def is_rosie_done(self):
        return self.current_row is self.goal_row and self.current_col is self.goal_col

    def _build_rosie_wall(self):
        """Not sure if needed yet, but building the initial known wall may be helpful"""
        # Left wall
        for i in range(self.max_rows):
            self.set_cell_wall(0, i, LEFT, CLOSED)
        # Right Wall
        for i in range(self.max_rows):
            self.set_cell_wall(self.max_cols - 1, i, RIGHT, CLOSED)

        # Top wall
        for i in range(self.max_cols):
            self.set_cell_wall(i, 0, LEFT, CLOSED)


    def print_maze(self):
        WALL_SYMBOLS = {
            UP: {
                OPEN:    "    +",
                CLOSED:  "----+",
                UNKNOWN: "....+"
            },
            DOWN: {
                OPEN:    "    +",
                CLOSED:  "----+",
                UNKNOWN: "....+"
            },
            LEFT: {
                OPEN:    " ",
                CLOSED:  "|",
                UNKNOWN: "."
            },

            RIGHT: {
                OPEN:    " ",
                CLOSED:  "|",
                UNKNOWN: "."                
            }
            
        }

        ROSIE_DIRECTION = {
            UP:    "(^^)",
            DOWN:  "(vv)",
            RIGHT: "(>>)",
            LEFT:  "(<<)"
        }

        s = "+"
        for j in xrange(0, self.max_cols):
            s += WALL_SYMBOLS[UP][self.grid[j][0].get_wall(UP)]
        s += "\n"



        for i in xrange(0, self.max_rows):
            s += WALL_SYMBOLS[LEFT][self.grid[0][i].get_wall(LEFT)]
            for j in xrange(0, self.max_cols):
                # Middle rosie
                if self.current_col is j and self.current_row is i:
                    s += ROSIE_DIRECTION[self.current_direction]
                else:
                    s += "    "
                s += WALL_SYMBOLS[RIGHT][self.grid[j][i].get_wall(RIGHT)]
            s += "\n"


            s += WALL_SYMBOLS[LEFT][self.grid[0][i].get_wall(LEFT)]
            for j in xrange(0, self.max_cols):
                # Middle rosie
                if self.current_col is j and self.current_row is i:
                    s += ROSIE_DIRECTION[self.current_direction]
                else:
                    s += "    "
                s += WALL_SYMBOLS[RIGHT][self.grid[j][i].get_wall(RIGHT)]
            s += "\n"

            # Do bottom part
            s += "+"
            for j in xrange(0, self.max_cols):
                s += WALL_SYMBOLS[DOWN][self.grid[j][i].get_wall(DOWN)]
            s += "\n"
        return s
k = KnownMaze(5, 5, (0,0), (5,5))
k.get_cell(0, 0).set_wall(RIGHT, OPEN)
print k.print_maze()
        
