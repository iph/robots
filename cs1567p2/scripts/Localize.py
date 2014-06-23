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
global imgur
DIRECTION_BLOB_COLOR = Color(235, 158, 180)
CLUSTER_POINT_THRESHOLD = 100
CLUSTER_POINT_CEILING = 1000
# TODO: use localizer with ros callbacks
# idea: each localizer associated with different transform
# and image publisher. define a new class that takes centers
# from all localizers and determines final position/orientation
# of each object. 

class Coordinator(object):
    transforms = [(0.0, 0.0), (0.0, 0.0), (.15485772525065952, 0.8954903004095727)]
    
    def __init__(self):
        self.all_robots = []

    def register_localizer(self):
        ident = len(self.all_robots)
        self.all_robots.append([])
        return ident

        
    def update(self, robots, ident):
        self.all_robots[ident] = robots
        self.publish()

    def publish(self):
        robots_identified = set()
        locations = []
        transformation = lambda point, local: (point[0] - local[0], point[1] - local[1])
        i = 0
        global locpub

        for localized_robots in self.all_robots:
            local_coords = Coordinator.transforms[i]
            
            for robot in localized_robots:
                if robot.id not in robots_identified:
                    center = transformation(robot.get_loc(), local_coords)
                    locations.append(Location(robot.id, center[0], center[1], robot.get_direction()))

                    robots_identified.add(robot.id)

            i += 1
        print locations
        locpub.publish(locations)
                    
            

class Localizer(object):

    color_names = {
        Color(243,225, 194): "Brown",
        Color(146, 190, 150): "Green", # Deep green
        Color(160, 188, 232): "Blue", # Blue
        Color(235, 158, 180): "Red", # Red
        Color(161, 137, 204): "Purple" # Purple     
    }
    obj_colors = [
        Color(243,225, 194), # BROWN
        Color(146, 190, 150), # Deep green
        Color(160, 188, 232), # Blue
        Color(235, 158, 180), # Red
        Color(161, 137, 204), # Purple
]

    dir_color = Color(235, 158, 180)

    def __init__(self, master):
        super(Localizer, self).__init__()
        self.master = master
        self.ident = master.register_localizer()
        self.obj_blobs = {}
        self.blobs = []
        self.unprocessed_blobs = []
        self.obj_centers = {}

    def process_image(self, image):
        print "Processing image"
        obj_colors = Localizer.obj_colors

        # todo: Make this cleaner..
        self.obj_blobs = {}
        self.blobs = []
        self.unprocessed_blobs = []
        self.obj_centers = {}
        self.robots = []
        groups = group_colors(image, obj_colors)

        global imgur
        imgur = image

        self.obj_blobs = groups

        all_points = set()
        clusters = []
        for color_key in groups:
            all_points = all_points | groups[color_key]
            clusters.extend(cluster_points(groups[color_key]))
        print "Done clustering.."

        for cluster in clusters:
            if len(cluster) > CLUSTER_POINT_THRESHOLD  and len(cluster) < CLUSTER_POINT_CEILING:
                blob = Blob(cluster, image)
                blob.color = label_color(blob.color, obj_colors)
                self.blobs.append(blob)
                self.unprocessed_blobs.append(blob)
                print "Blob: ", len(blob.points), " color: ", Localizer.color_names[blob.color]

        filtered_points = set()
        for blob in self.blobs:
            filtered_points = filtered_points | blob.points
            
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
            except StopIteration:
                print len(points), "  ", len(blob.points)
                blob.set_center(midpoint_2d(points))
                pass

        self.obj_centers = obj_centers
        # Some weird nones in the point cloud. REMOVE THEM!
        # todo: This could be a bigger problem. Check dis out when you can
        filtered_blobs = []
        for blob in self.blobs:
            if blob.center is not None:
                filtered_blobs.append(blob)
                print "Blob: ", blob.center, " , color: ", Localizer.color_names[blob.color], " points:", len(blob.points)

        self.blobs = filtered_blobs
        dir_blobs, potential_identifier_blobs = classify_blobs(self.blobs)
        robots = find_robots(dir_blobs, potential_identifier_blobs)
        print "Finished processing all robots"    

        filtered_points = set()
        for robot in robots:
            filtered_points = filtered_points | robot.dir_blob.points
            filtered_points = filtered_points | robot.identifier_blob.points
        self.master.update(robots, self.ident)
        send_mask(imgur, filtered_points)

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
    print "red blob count: ", len(red_blobs)
    for red_blob in red_blobs:
        euclid_dist = lambda blob: math.sqrt((red_blob.center[0] - blob.center[0])**2 + (red_blob.center[1] - blob.center[1])**2)
        closest_blob = min(others, key = euclid_dist)

        if abs(closest_blob.center[0] - red_blob.center[0]) > .2 or abs(closest_blob.center[1] - red_blob.center[1]) > .2:
            print "Closest blob color: ", Localizer.color_names[closest_blob.color]
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
    deep = mask.data
    mask.data = "".join(mask.data)
    global kinect2pub
    kinect2pub.publish(mask)

    mask.data = deep
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
    coordinator = Coordinator()
    rospy.init_node("localize")
    locpub = rospy.Publisher("/rosie/location",LocationList) #publish your locations
    kinect1pub = rospy.Publisher("/rosie/mask",Image) #test your mask
    kinect2pub = rospy.Publisher("/rosie/maskz",Image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, process_image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, top_image_callback)
    #rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, top_cloud_callback)
    #rospy.Subscriber("/kinect2/rgb/image_color", Image, mid_image_callback)
    #rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, mid_cloud_callback)
    local2 = (.15485772525065952, 0.8954903004095727)
    local1 = (0.0, 0.0)
    localizer1 = Localizer(coordinator)
    
    
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

