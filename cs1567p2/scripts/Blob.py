from VizUtil import probe_color, Point, Color

# Todo: Should we defer the startup cost until it is required?
class Blob(object):

    def __init__(self, points, image):
        self.points = points

        # Calculate the center.
        x_sum = 0
        y_sum = 0
        for point in points:
            x_sum += point.x
            y_sum += point.y

        length = len(points)
        self.center = Point(x_sum/length,  y_sum/length) if length > 0 else Point(0, 0)


        # Calculate the average color.
        r_sum = 0
        g_sum = 0
        b_sum = 0
        for point in points:
            color = probe_color(image, point.y, point.x)
            r_sum += color.r
            g_sum += color.g
            b_sum += color.b

        self.color = Color(r_sum/length, g_sum/length, b_sum/length)

