from collections import namedtuple
import rospy
from copy import deepcopy
import sys

Point = namedtuple('Point', ['x', 'y'])
Color = namedtuple('Color', ['r', 'g', 'b'])
Pixel = namedtuple('Pixel', ['point', 'color'])

obj_threshold = {
    Color(160, 188, 232): [Color(145, 188, 220), Color(170, 205, 245)],
    Color(235, 158, 180): [Color(220, 120, 150), Color(245, 185, 210)],
    Color(240, 221, 191): [Color(220, 200, 170), Color(250, 230, 210)],
    Color(161, 137, 204): [Color(140, 110, 180), Color(170, 170, 220)]
}
def get_index(image, row, col):
    return (row * image.step) + (3 * col)

def label_color(color, color_choices):
    min_key = lambda other: abs(color.r - other.r) + abs(color.g - other.g) + abs(color.b - other.b)
    return min(color_choices, key = min_key)
    

    

def probe_color(image, row, col):
    """Returns the color of the picture at row, col in the image."""

    index = get_index(image, row, col)
    image_matrix = image.data
    return Color(ord(image_matrix[index + 2]), ord(image_matrix[index + 1]),\
                 ord(image_matrix[index]))

# Todo: Might not need this anymore.
def midpoint_2d(points):
    """Computes center of finite set of points in 2d space
       Returns a tuple (center_x, center_y)"""
    
    n = len(points)
    if n is 0:
        return None

    x_sum = 0
    y_sum = 0
    # center of set of finite points is average of those points
    for point in points:
        x_sum += point.x
        y_sum += point.y

    return (x_sum / n, y_sum / n)

def color_match(target, color, threshold=30):
    """Returns true if color is near target, within a certain threshold"""
    global obj_threshold
    return (obj_threshold[target][0].r < color.r and color.r < obj_threshold[target][1].r 
    and obj_threshold[target][0].g < color.g and color.g < obj_threshold[target][1].g 
    and obj_threshold[target][0].b < color.b and color.b < obj_threshold[target][1].b )

def group_colors(image, colors):
    """Buckets all of the points in the image that match one of the given 
       colors. Returns a dictionary of colors to sets of points.
       Points are in (col, row) (x, y)"""

    color_dict = {}
    
    # empty set for each color
    for color in colors:
        color_dict[color] = set()

    # convert image data to list
    image_matrix = image.data
    #image_data = bytes(image.data)
    #image_matrix = image_data
    for row in xrange(image.height):
        for col in xrange(image.width):
            # each row is message.step long, each element is 3 bytes
            index = get_index(image, row, col)
            # grab color of current pixel (conv to r,g,b)
            pixel_color = Color(ord(image_matrix[index + 2]),\
                                ord(image_matrix[index + 1]),\
                                ord(image_matrix[index]))
            for color in colors:
                # if color matches, add the row and col
                if color_match(color, pixel_color):
                    color_dict[color].add(Point(col, row))
                    break
    print "Done grouping.."
    return color_dict

def cluster_points(points):
    """Takes a set of points and clusters the adjacent points into separate 
       lists. Returns a list of sets of points, each set defining a cluster"""
    clusters = []
    points = deepcopy(points)
    while len(points) > 0:
        # grab a random point and grow it
        cluster = grow_cluster(points)
        clusters.append(cluster)
    return clusters

def grow_cluster(points):
    """Grabs a random point from points and grows a cluster
    of adjacent points from it."""

    start = points.pop()

    # unprocessed cluster points
    adjacents = set()
    adjacents.add(start)

    # actual cluster
    cluster = set()
    cluster.add(start)

    while len(adjacents) > 0:
        # grab an unprocessed point
        start = adjacents.pop()
        # check its adjacent points
        for delta_x in [-1, 0, 1]:
            for delta_y in [-1, 0, 1]:
                next = Point(start.x + delta_x, start.y + delta_y)
                if next in points and next not in cluster:
                    adjacents.add(next)
                    points.remove(next)
                    cluster.add(next)

    return cluster

