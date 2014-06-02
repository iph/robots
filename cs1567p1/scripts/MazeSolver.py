#!/usr/bin/python
import rospy
import pdb
from cs1567p1.srv import *
from std_srvs.srv import * 

from Maze import KnownMaze, CLOSED, OPEN, UNKNOWN, LEFT, RIGHT, UP, DOWN
known_maze = KnownMaze(5, 5, (0,0), (5,5))

make_maze_service = None
print_maze_service = None
get_wall_service = None
constant_command_service = None

def move_forward():
    return 1

def update_room():
    global known_maze
    next_wall ={ UP: RIGHT, RIGHT: DOWN, DOWN: LEFT, LEFT: UP}
    end_direction = known_maze.get_rosie_direction()
    update_known_maze_wall()
    current_direction = next_wall[end_direction]
    known_maze.set_rosie_direction(current_direction)
    while(current_direction is not end_direction):
        update_known_maze_wall()
        current_direction = next_wall[current_direction]
        known_maze.set_rosie_direction(current_direction)




def update_known_maze_wall():
    global known_maze
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
    #make_maze_service(5, 5)
    for i in range(5):
        for j in range(5):
            known_maze.warp_to_position((j, i))
            update_room()
    print known_maze.print_maze()
    move_to_position_generation((0,0))


def move_to_position_generation(end_pos):
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
    print known_maze.print_path_and_maze(path)

if __name__ == "__main__":   
    try: 
        initialize_commands()
    except rospy.ROSInterruptException: pass

