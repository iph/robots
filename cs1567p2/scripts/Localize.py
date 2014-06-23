#!/usr/bin/env python
import rospy
import math
import pickle
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
from cs1567p2.msg import *
from VizUtil import *
from Blob import *
from Robot import *
color_mask_list = [[180, 165, 235], [240, 205, 175], [200, 250, 250]]
threshold = 16
locpub = None
kinect1pub = None
kinect2pub = None
top_mask = Image()
mid_mask = Image()
robot_centers = []

DIRECTION_BLOB_COLOR = Color(235, 158, 180)
CLUSTER_POINT_THRESHOLD = 60
CLUSTER_POINT_CEILING = 600
# TODO: use localizer with ros callbacks
# idea: each localizer associated with different transform
# and image publisher. define a new class that takes centers
# from all localizers and determines final position/orientation
# of each object. 
class Localizer(object):
    # TODO: put color filters here
    obj_colors = [
#        Color(214, 185, 146), # Shit stain brown
#        Color(244, 243, 240), # White for cross
 #       Color(209, 243, 210), # Neon construction green.
        Color(160, 188, 232), # Blue
        Color(235, 158, 180), # Red
        Color(240, 221, 191), # Crappy oj/ yellow?
        Color(161, 137, 204) # Purple
#        Color(250, 232, 195)
]

    dir_color = Color(235, 158, 180)

    def __init__(self, local):
        super(Localizer, self).__init__()
        self.local = local
        self.obj_blobs = {}
        self.blobs = []
        self.unprocessed_blobs = []
        self.obj_centers = {}
        #self.dir_blobs = []

    def process_image(self, image):
        print "Processing image"
        obj_colors = Localizer.obj_colors

        groups = group_colors(image, obj_colors)

        self.obj_blobs = groups

        #for color_key in groups:
        #    print "Color: " , color_key, " Center: ", midpoint_2d(groups[color_key])

        all_points = set()
        clusters = []
        for color_key in groups:
            all_points = all_points | groups[color_key]
            clusters.extend(cluster_points(groups[color_key]))
        print "Done clustering.."

        for cluster in clusters:
            if len(cluster) > CLUSTER_POINT_THRESHOLD :
                blob = Blob(cluster, image)
                blob.color = label_color(blob.color, obj_colors)
                self.blobs.append(blob)
                self.unprocessed_blobs.append(blob)
                print "Blob: ", len(blob.points), " color: ", blob.color

        filtered_points = set()
        for blob in self.blobs:
            filtered_points = filtered_points | blob.points
            
        send_mask(image, filtered_points)
        print "Finished processing image"

        #self.obj_centers = centers

    def process_cloud(self, cloud):
        if not self.unprocessed_blobs:
            return
        print "Processing cloud"
        obj_centers = {}

        obj_blobs = self.obj_blobs
        for blob in self.unprocessed_blobs:
            
            points = []
            point_iter = pc2.read_points(cloud, field_names=None,\
                                         skip_nans=True, uvs=blob.points)
            try:
                point = next(point_iter)
                while point != None:
                    points.append(Point(point[0], point[1]))
                    point = next(point_iter)
                blob.set_center(midpoint_2d(points))
            except StopIteration:
                blob.set_center(midpoint_2d(points))
                pass


        self.obj_centers = obj_centers
        # Some weird nones in the point cloud. REMOVE THEM!
        # todo: This could be a bigger problem. Check dis out when you can
        filtered_blobs = []
        for blob in self.blobs:
            if blob.center is not None:
                filtered_blobs.append(blob)
                # Check if it is red.
                print "Blob: ", blob.center, " , color: ", blob.color, " points:", len(blob.points)

        self.blobs = filtered_blobs
        dir_blobs, potential_identifier_blobs = classify_blobs(self.blobs)
        robots = find_robots(dir_blobs, potential_identifier_blobs)
        print "Finished processing all robots"    
        transformation = lambda point, local: (point[0] + local[0], point[1] + local[1])
        for robot in robots:
            print "Robot %d: (%.2f, %.2f) with dir: %d" % (robot.get_id(), 
                                                transformation(robot.get_loc(), self.local)[0],
                                                transformation(robot.get_loc(), self.local)[1], 
                                                robot.get_direction())

def classify_blobs(blobs):
    dir_blobs = []
    potential_identifier_blobs = []
    for blob in blobs:
        if blob.color == DIRECTION_BLOB_COLOR:
            dir_blobs.append(blob)
        else:
            potential_identifier_blobs.append(blob)
    return dir_blobs, potential_identifier_blobs

def find_robots(red_blobs, others):
    robots = []
    for red_blob in red_blobs:
        euclid_dist = lambda blob: math.sqrt((red_blob.center[0] - blob.center[0])**2 + (red_blob.center[1] - blob.center[1])**2)
        closest_blob = min(others, key = euclid_dist)

        if abs(closest_blob.center[0] - red_blob.center[0]) > .2 or abs(closest_blob.center[1] - red_blob.center[1]) > .2:
            continue
        else:
            robots.append(Robot(red_blob, closest_blob))
    return robots
    

def send_mask(image, points):
    mask = Image()  
    mask.height = image.height
    mask.width = image.width
    mask.encoding = image.encoding
    mask.is_bigendian = image.is_bigendian
    mask.step = image.step
    mask.data = list(image.data)

    for row in xrange(mask.height):
        for col in xrange(mask.width):
            p = Point(col, row)
            if p not in points:
                index = get_index(image, row, col)
                mask.data[index] = chr(0)
                mask.data[index + 1] = chr(0)
                mask.data[index + 2] = chr(0)

    mask.data = "".join(mask.data)
    global kinect1pub
    kinect1pub.publish(mask)



def initialize():
    global kinect1pub
    global kinect2pub
    global locpub
    rospy.init_node("localize")
    #locpub = rospy.Publisher("/rosie/location",LocationList) #publish your locations
    kinect1pub = rospy.Publisher("/rosie/mask",Image) #test your mask
    kinect2pub = rospy.Publisher("/rosie/mask",Image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, process_image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, top_image_callback)
    #rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, top_cloud_callback)
    #rospy.Subscriber("/kinect2/rgb/image_color", Image, mid_image_callback)
    #rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, mid_cloud_callback)

    localizer1 = Localizer((0,0))
    #    rospy.Subscriber("/kinect2/rgb/image_color", Image, localizer1.process_image)
    #    rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, localizer1.process_cloud)
    input = open("data2.img", "rb")
    input2 = open("pc2.img", "rb")
    data = pickle.load(input)
    data2 = pickle.load(input2)
    localizer1.process_image(data)
    localizer1.process_cloud(data2)
    rospy.spin()

if __name__ == "__main__":
    initialize()

