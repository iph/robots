#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
from cs1567p2.msg import *
from VizUtil import *

color_mask_list = [[110,0,0]]
threshold = 20
locpub = None
kinect1pub = None
kinect2pub = None
top_mask = Image()
mid_mask = Image()
robot_centers = []

# TODO: use localizer with ros callbacks
# idea: each localizer associated with different transform
# and image publisher. define a new class that takes centers
# from all localizers and determines final position/orientation
# of each object. 
class Localizer(object):
    # TODO: put color filters here
    obj_colors = [Color(0,0,0)]
    dir_color = None

    def __init__(self):
        super(ImageProcessor, self).__init__()
        self.obj_centers = []
        self.dir_centers = []

    def process_image(image):
        obj_colors = Localizer.obj_colors

        groups = group_colors(message, obj_colors)

        centers = []
        for obj_color in obj_colors:
            centers.append(midpoint_2d(groups[obj_color]))

        self.obj_centers = centers

        # transform centers here with per-localizer variable?



def process_image(message):

    # TODO: put colors of post-its here
    colors = [
        Color(0,0,0)
    ]

    # group the colors into points
    groups = group_colors(message, colors)
    # get center of each color group
    centers = []
    for color in colors:
        centers.append(midpoint_2d(groups[color]))

    # store centers
    global robot_centers
    robot_centers = centers

    print probe_color(message, 0, 0)

    # TODO: grab directional blobs & cluster


def top_image_callback(message):
    global color_mask_list
    global top_mask
    global threshold
    global kinect1pub
    #make a new image if you want to view your mask
    top_mask = Image()
    top_mask.height = message.height
    top_mask.width = message.width
    top_mask.encoding = message.encoding
    top_mask.is_bigendian = message.is_bigendian
    top_mask.step = message.step
    if message.encoding == "bgr8": #this is image_color encoding
        byte_array = list(message.data) #convert unit8[] from string to chars
        for index in xrange(message.height*message.width): #iterate through
            byte_array[3*index] = chr(0)
            byte_array[3*index+1] = chr(0)
            byte_array[3*index+2] = chr(0)
            for k in xrange(len(color_mask_list)): 
                #iterate through color list, if the bytes match, save the color
                #in the mask
                if abs(color_mask_list[k][0] - ord(byte_array[3*index])) < threshold\
                        and abs(color_mask_list[k][1] - ord(byte_array[3*index+1])) < threshold\
                        and abs(color_mask_list[k][2] - ord(byte_array[3*index+2])) < threshold:
                    byte_array[3*index+0] = chr(color_mask_list[k][0])
                    byte_array[3*index+1] = chr(color_mask_list[k][1])
                    byte_array[3*index+2] = chr(color_mask_list[k][2])
    top_mask.data = "".join(byte_array) #make char[] back into uint8[] string
    kinect1pub.publish(top_mask) #publish the mask for viewing
    print "done1"
        
def mid_image_callback(message):
    global color_mask_list
    global mid_mask
    global threshold
    global kinect2pub
    mid_mask = Image()
    mid_mask.height = message.height
    mid_mask.width = message.width
    mid_mask.encoding = message.encoding
    mid_mask.is_bigendian = message.is_bigendian
    mid_mask.step = message.step
    if message.encoding == "bgr8":
        byte_array = list(message.data)
        for index in xrange(message.height*message.width):
            byte_array[3*index] = chr(0)
            byte_array[3*index+1] = chr(0)
            byte_array[3*index+2] = chr(0)
            for k in xrange(len(color_mask_list)):
                if abs(color_mask_list[k][0] - ord(byte_array[3*index])) < threshold\
                        and abs(color_mask_list[k][1] - ord(byte_array[3*index+1])) < threshold\
                        and abs(color_mask_list[k][2] - ord(byte_array[3*index+2])) < threshold:
                    byte_array[3*index+0] = chr(color_mask_list[k][0])
                    byte_array[3*index+1] = chr(color_mask_list[k][1])
                    byte_array[3*index+2] = chr(color_mask_list[k][2])
    mid_mask.data = "".join(byte_array)
    kinect2pub.publish(mid_mask)
    print "done2"


def top_cloud_callback(message):
    try:
        global robot_centers
        centers = robot_centers

        if len(centers) == 0:
            return
        #make a generator, skipping points that have no depth, on points in 
        # list of uvs (index into image [col,row]) or if empty list, get all pt
        data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=centers) 
        i=0
        iteration1 = next(data_out) #format x,y,z,rgba
        while iteration1 != None:
            print "X: ", iteration1[0] , " Y: ", iteration1[1]
            iteration1 = next(data_out)
            rospy.sleep(0.5)
            i=i+1
    except StopIteration: 
        print "1 complete"

def mid_cloud_callback(message):
    try:
        data_out = pc2.read_points(message, field_names=None, skip_nans=True, uvs=[])
        i=0
        iteration1 = next(data_out) #format x,y,z,rgba
        while iteration1 != None:
            iteration1 = next(data_out)
            i=i+1
    except StopIteration: 
        print "2 complete"

def initialize():
    global kinect1pub
    global kinect2pub
    global locpub
    rospy.init_node("localize")
    locpub = rospy.Publisher("/rosie/location",LocationList) #publish your locations
    kinect1pub = rospy.Publisher("/rosie/mask",Image) #test your mask
    kinect2pub = rospy.Publisher("/rosie/mask",Image)
    rospy.Subscriber("/kinect1/rgb/image_color", Image, process_image)
    rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, top_cloud_callback)
    #rospy.Subscriber("/kinect2/rgb/image_color", Image, mid_image_callback)
    #rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, mid_cloud_callback)
    rospy.spin()

if __name__ == "__main__":
    initialize()

