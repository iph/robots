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
        return open_walls

    def set_wall(self, direction, status):
        """direction should be defined above{LEFT, RIGHT, UP, DOWN}, same for status {OPEN, CLOSED, UNKNOWN}"""
        self.walls[direction] = status

    
    def get_wall(self, direction):
        return self.walls[direction]

    def is_fully_unknown(self):
        return self.walls[LEFT] is UNKNOWN and self.walls[RIGHT] is UNKNOWN and self.walls[UP] is UNKNOWN and self.walls[DOWN] is UNKNOWN

    def is_fully_known(self):
        return self.walls[LEFT] is not UNKNOWN and self.walls[RIGHT] is not UNKNOWN and self.walls[UP] is not UNKNOWN and self.walls[DOWN] is not UNKNOWN


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
        """Moves rosie in the direction she is currently facing."""
        direction = self.current_direction
        if direction is UP:
            self.current_row -= 1
        if direction is DOWN:
            self.current_row += 1
        if direction is LEFT:
            self.current_col -= 1
        if direction is RIGHT:
            self.current_col += 1


    def warp_to_position(self, new_position):
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

    def move_rosie_up(self):
        self.current_row -=1

    def move_rosie_down(self):
        self.current_row += 1
    
    def move_rosie_left(self):
        self.current_col -= 1

    def move_rosie_right(self):
        self.current_col += 1

    def is_rosie_done(self):
        return self.current_row is self.goal_row and self.current_col is self.goal_col

    def get_adjacent_connected_positions(self, current_position):
        """Returns positions of any cells that can be traversed from the current position.
        Example: I'm in cell (0,0) and the only open neigbhors are below and to the right of me.
        I would return: (1,0) and (0,1)
        """
        col, row = current_position
        cell = self.get_cell(col, row)
        neighbor_positions = []
        open_neighbors = cell.get_open_walls()
        for neighbor in open_neighbors:
            neihbor_pos = None
            if neighbor is UP:
                neighbor_pos = (col, row - 1)
            elif neighbor is DOWN:
                neighbor_pos = (col, row + 1)
            elif neighbor is LEFT:
                neighbor_pos = (col - 1, row)
            else:
                neighbor_pos = (col + 1, row)

            neighbor_positions.append(neighbor_pos)
        return neighbor_positions
        
    def print_path_and_maze(self, path, turn_path):
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
            UP:    "^^^^",
            DOWN:  "vvvv",
            RIGHT: ">>>>",
            LEFT:  "<<<<"
        }
        
        s = "+"
        for j in xrange(0, self.max_cols):
            s += WALL_SYMBOLS[UP][self.grid[j][0].get_wall(UP)]
        s += "\n"

        for i in xrange(0, self.max_rows):
            for _ in range(2):
                s += WALL_SYMBOLS[LEFT][self.grid[0][i].get_wall(LEFT)]
                for j in xrange(0, self.max_cols):
                    # Middle rosie
                    cur_pos = (j,i)
                    if cur_pos in path and path.index(cur_pos) < len(turn_path):
                            s += ROSIE_DIRECTION[turn_path[path.index(cur_pos)]]
                    elif cur_pos in path:
                        s += "XXXX"
                    elif self.grid[j][i].is_fully_unknown():
                        s += "////"
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
            for _ in range(2):
                s += WALL_SYMBOLS[LEFT][self.grid[0][i].get_wall(LEFT)]
                for j in xrange(0, self.max_cols):
                    # Middle rosie
                    if self.current_col is j and self.current_row is i:
                        s += ROSIE_DIRECTION[self.current_direction]
                    elif self.grid[j][i].is_fully_unknown():
                        s += "////"
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


        


