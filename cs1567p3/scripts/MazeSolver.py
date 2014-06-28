#!/usr/bin/python
import rospy
import pdb
from cs1567p3.srv import *
from std_srvs.srv import * 

from Maze import KnownMaze, CLOSED, OPEN, UNKNOWN, LEFT, RIGHT, UP, DOWN

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
class MazeSolver(object):
    def __init__(self, maze):
        self.known_maze = maze
        self.stack = []
        self.visited = [] # A visited room is a room where it is completely known


    def room_is_known(self, col, row):
        return self.known_maze.get_cell(col, row).is_fully_known()

    def current_room_is_known(self):
        return self.known_maze.get_current_cell().is_fully_known()

    def current_room_is_visited(self):
        return self.known_maze.get_rosie_position() in self.visited

    def is_at_goal(self):
        return self.known_maze.is_rosie_done()
    def add_current_room_to_visited(self):
        self.visited.append(self.known_maze.get_rosie_position())
    def calculate_next_steps(self):
        steps = []
        if self.is_at_goal():
            steps.append(GOAL_STEP)
        if not self.current_room_is_known():
            steps.extend(self.generate_room_steps())
        elif not self.current_room_is_visited():
            self.stack.extend(self.known_maze.get_adjacent_connected_positions(self.known_maze.get_rosie_position()))
            self.add_current_room_to_visited()
            
            # Get next stack position to go to and generate steps to get there..
            next_position = self.stack.pop()
            while(next_position in self.visited):
                next_position = self.stack.pop()
            steps = generate_move_to_position_steps(next_position, self.known_maze)
        else:
            # Where am i and what should i do?
            raise "Illegal state.."
        return steps
            
    def generate_room_steps(self): # todo: Move this into interpreter?
        steps = []
        known_maze = self.known_maze
        next_wall ={ UP: RIGHT, RIGHT: DOWN, DOWN: LEFT, LEFT: UP}
        steps_dict = {UP: ROTATE_UP, DOWN: ROTATE_DOWN, LEFT: ROTATE_LEFT, RIGHT: ROTATE_RIGHT}
        end_direction = known_maze.get_rosie_direction()
        current_direction = next_wall[end_direction]
        steps.append(steps_dict[current_direction])
        steps.append(SCAN)
        while(current_direction is not end_direction):
            current_direction = next_wall[current_direction]
            steps.append(steps_dict[current_direction])
            steps.append(SCAN)
        return steps

        
    def update_known_maze_wall(self): # todo: Move this into interpreter.
        known_maze = self.known_maze
        col, row = known_maze.get_rosie_position()
        direction = known_maze.get_rosie_direction()
        wall_bool = get_wall_service(col, row, direction)
        wall_status = UNKNOWN
        if wall_bool.wall == 1:
            wall_status = CLOSED
        else:
            wall_status = OPEN
        known_maze.set_cell_wall(col, row, direction, wall_status)
        
    
def initialize_commands():
    rospy.init_node('mazesolvernode', anonymous=True)
    rospy.wait_for_service('make_maze')
    rospy.wait_for_service('print_maze')
    rospy.wait_for_service('get_wall')


    global make_maze_service, print_maze_service, get_wall_service
    make_maze_service = rospy.ServiceProxy('make_maze', MakeNewMaze)
    print_maze_service = rospy.ServiceProxy('print_maze', Empty)
    get_wall_service = rospy.ServiceProxy('get_wall', GetMazeWall)
    make_maze_service(5, 5)
    for i in range(5):
        for j in range(5):
            known_maze.warp_to_position((j, i))
            update_room()
    known_maze.warp_to_position((0, 0))
    print known_maze.print_maze()

    generate_move_to_position_steps((4,4))

def generate_move_to_position_steps(end_pos, known_maze):
    path = move_to_position_generation(end_pos, known_maze)
    turn_path = []
    i = 0
    # (4,4)->(3,4) (went left, so turn left)  -1,0
    # (4,4)-> (4,3) (went up, so turn up) 0, -1
    movement_type = {(0, 1): [ROTATE_DOWN, MOVE_DOWN], (0, -1): [ROTATE_UP, MOVE_UP], (1, 0): [ROTATE_RIGHT, MOVE_RIGHT], (-1, 0): [ROTATE_LEFT, MOVE_LEFT]}
    while i < len(path) - 1:
        previous_node = path[i]
        current_node = path[i + 1]
        m_type = movement_type[(current_node[0] - previous_node[0], current_node[1] - previous_node[1])]
        turn_path.extend(m_type)
        i += 1
        # Only have to return turn types now.
    return turn_path
        

def move_to_position_generation(end_pos, known_maze):
    """Generates paths in the known maze."""
    trace = []
    stack = []
    visited = []
    cur_pos = known_maze.get_rosie_position()     
    # 0 is previous, 1 is current
    stack.append((cur_pos, cur_pos))
    while len(stack) > 0:
        cur_pos = stack.pop()
        if cur_pos[1] in visited:
            continue
        else:
            visited.append(cur_pos[1])
        # Clear until we get back
        while len(trace) > 1 and trace[-1][1] is not cur_pos[0]:
            trace.pop()

        trace.append(cur_pos)
        previous_pos, current_pos = cur_pos
        if current_pos[0] is end_pos[0] and current_pos[1] is end_pos[1]:
            print "Success"
            break
        neighbor_positions = known_maze.get_adjacent_connected_positions(current_pos)
        for neighbor_position in neighbor_positions:
            stack.append((current_pos, neighbor_position))
    path = []
    for node in trace:
        _, cur_node = node
        path.append(cur_node)

    return path
