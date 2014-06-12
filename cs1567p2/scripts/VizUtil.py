class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Color(object):
    def __init__(self, r, g, b, a):
        self.r = r
        self.g = g
        self.b = b
        self.a = a

class Pixel(object):
    def __init__(self, point, color):
        self.x = point.x
        self.y = point.y
        self.r = color.r
        self.g = color.g
        self.b = color.b
        self.a = color.a

def midpoint_2d(points):
    """Computes center of finite set of points in 2d space
       Returns a tuple (center_x, center_y)"""
    x_sum = 0.0
    y_sum = 0.0
    n = len(points)
    # center of set of finite points is average of those points
    for point in points:
        x_sum += point.x
        y_sum += point.y

    return (x_sum / n, y_sum / n)

def color_match(color, target, threshold=20):
    """Returns true if color is near target, within a certain threshold"""
    return abs(color.r - target.r) < threshold\
           and abs(color.b - target.b) < threshold\
           and abs(color.g - target.g) < threshold

# NEEDS TESTED
def group_colors(cloud, colors):
    """Buckets all of the points in the cloud that match one of the given 
       colors. Returns a dictionary of colors to lists of points"""

    color_dict = {}
    
    for color in colors:
        color_dict[color] = []

    threshold = 20
    for point in cloud:
        for color in colors:
            if color_match(color, point):
                color_dict[color].append(point)
                break

    return color_dict

# NEEDS TESTED
def cluster_points(points):
    """Takes a list of points and clusters the adjacent points into separate lists
       Returns a list of lists of points, each list defining a cluster"""
    clusters = []
    while len(points) > 0:
        # grab a random point and grow it
        point = points.pop()
        clusters.append(grow_point(point, [point], points))

# NEEDS TESTED
def grow_point(start, cluster, points):
    """Removes all the points in points that are adjacent to start, adds them
       to cluster, then recursively grows each of them."""

    i = 0
    adjacents = []

    while i < len(points):
        if adjacent(start, points[i]):
            point = points.pop(i)
            adjacents.append(point)
            cluster.append(point)
        else:
            i += 1

    for point in adjacents:
        grow_point(point, cluster, points)

    return cluster





