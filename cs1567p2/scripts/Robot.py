import math
from VizUtil import Color
class Robot(object):
    identifier = {
        Color(160, 188, 232): 1, # BLUUUEEE
        Color(161, 137, 204): 2, # PURPLE
        Color(243,225, 194): 3, #Brown
        Color(146, 190, 150): 4, # Green
    }
    def __init__(self, dir_blob, identifier_blob):
        self.dir = None
        self.dir_blob = dir_blob
        self.id = Robot.identifier[identifier_blob.color]
        self.identifier_blob = identifier_blob

    def get_loc(self):
        return self.identifier_blob.center

    def print_vector (self):
        other_vec = (self.dir_blob.center[0] - self.identifier_blob.center[0], self.dir_blob.center[1] - self.identifier_blob.center[1])
        return str(other_vec)

    def get_id(self):
        return self.id

    def get_direction(self):
        if self.dir:
            return self.dir

        y_vec = (0.0, 1.0)  # The arrow be pointing down for 0 degrees.
        other_vec = (self.dir_blob.center[0] - self.identifier_blob.center[0], self.dir_blob.center[1] - self.identifier_blob.center[1])

        ans = (y_vec[1] * other_vec[1]) / ((y_vec[1]**2)**(.5) * (other_vec[0]**2 + other_vec[1]**2)**(.5)) # Vector angles modified to not uselessly multiply the 0 in y_vec

        self.dir = math.acos(ans) * 180.0 / math.pi
        if other_vec[0] < 0:
            self.dir = 360 - self.dir
        #if self.dir < 0:
        #    self.dir += 360
        #else:
        #    self.dir = 360 - self.dir
        return self.dir

