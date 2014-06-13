from collections import namedtuple
 

Point = namedtuple('Point', ['x', 'y'])
Color = namedtuple('Color', ['r', 'g', 'b', 'a'])
Pixel = namedtuple('Pixel', ['point', 'color'])

def get_index(image, row, col):
    return (row * image.step) + (3 * col)

def probe_color(image, row, col):
    """Returns the color of the picture at row, col in the image."""

    index = get_index(image, row, col)
    image_matrix = list(image.data)
    return Color(image_matrix[index], image_matrix[index + 1],\
                 image_matrix[index + 2])

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

def color_match(color, target, threshold=20):
    """Returns true if color is near target, within a certain threshold"""
    return abs(color.r - target.r) < threshold\
           and abs(color.b - target.b) < threshold\
           and abs(color.g - target.g) < threshold

# NEEDS TESTED
def group_colors(image, colors):
    """Buckets all of the points in the image_matrix that match one of the given 
       colors. Returns a dictionary of colors to sets of points"""

    color_dict = {}
    
    # empty set for each color
    for color in colors:
        color_dict[color] = set()

    # convert image data to list
    image_matrix = list(image.data)

    for row in xrange(image.height):
        for col in xrange(image.width):
            # each row is message.step long, each element is 3 bytes
            index = get_index(image, row, col)
            # grab color of current pixel
            pixel_color = Color(image_matrix[index], image_matrix[index + 1],\
                          image_matrix[index + 2])
            for color in colors:
                # if color matches, add the row and col
                if color_match(color, pixel_color):
                    color_dict[color].add(Point(row, col))
                    break

    return color_dict

# NEEDS TESTED
def cluster_points(points):
    """Takes a set of points and clusters the adjacent points into separate 
       lists. Returns a list of sets of points, each set defining a cluster"""
    clusters = []
    while len(points) > 0:
        # grab a random point and grow it
        point = points.pop()
        cluster = set()
        cluster.add(point)
        grow_point(point, cluster, points)
        clusters.append(cluster)

# NEEDS TESTED
def grow_point(start, cluster, points):
    """Removes all the points in points that are adjacent to start, adds them
       to cluster, then recursively grows each of them."""

    adjacents = set()

    for delta_x in [-1, 0, 1]:
        for delta_y in [-1, 0, 1]:
            next = Point(start.x + delta_x, start.y + delta_y)
            if next in points and next not in cluster:
                adjacents.add(next)
                points.remove(next)
                cluster.add(next)

    for point in adjacents:
        grow_point(point, cluster, points)

