#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import *
from cs1567p2.msg import *
from VizUtil import *

color_mask_list = [[180, 165, 235], [240, 205, 175], [200, 250, 250]]
threshold = 16
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
    obj_colors = [Color(235, 165, 180)]
    dir_color = None

    def __init__(self):
        super(Localizer, self).__init__()
        self.obj_blobs = {}
        self.obj_centers = {}
        #self.dir_blobs = []

    def process_image(self, image):
        print "Processing image"
        obj_colors = Localizer.obj_colors

        groups = group_colors(image, obj_colors)

        self.obj_blobs = groups

        for color_key in groups:
            print "Color: " , color_key, " Center: ", midpoint_2d(groups[color_key])

        all_points = set()
        for color_key in groups:
            all_points = all_points | groups[color_key]

        send_mask(image, all_points)
        print "Finished processing image"

        #self.obj_centers = centers

    def process_cloud(self, cloud):
        if not self.obj_blobs:
            return
        print "Processing cloud"
        obj_centers = {}

        obj_blobs = self.obj_blobs
        for color_key in obj_blobs:
            
            points = []
            point_iter = pc2.read_points(cloud, field_names=None,\
                                skip_nans=True, uvs=obj_blobs[color_key])
            try:
                point = next(point_iter)
                while point != None:
                    points.append(Point(point[0], point[1]))
                    point = next(point_iter)
            except StopIteration:
                pass

            obj_centers[color_key] = midpoint_2d(points)

        for color_key in obj_centers:
            print "Color: ", color_key, " Center: " , obj_centers[color_key]

        self.obj_centers = obj_centers  
        print "Finished processing cloud"    

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


def process_image(message):

    # TODO: put colors of post-its here
    colors = [
        Color(0,0,0)
    ]

    print probe_color(message, message.height / 2, message.width / 2)

    # group the colors into points
    groups = group_colors(message, colors)
    # get center of each color group
    centers = []
    for color in colors:
        centers.append(midpoint_2d(groups[color]))

    # store centers
    global robot_centers
    robot_centers = centers

    #print probe_color(message, 0, 0)

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
        mid = (message.height/2 * message.step) + ((message.width/2) * 3)
        print "Mid color: ", ord(byte_array[mid]), " ", ord(byte_array[mid + 1]), " ", ord(byte_array[mid + 2])
        for index in xrange(message.height*message.width): #iterate through
            match = False
            for k in xrange(len(color_mask_list)): 
                #iterate through color list, if the bytes match, save the color
                #in the mask
                if abs(color_mask_list[k][0] - ord(byte_array[3*index])) < threshold\
                        and abs(color_mask_list[k][1] - ord(byte_array[3*index+1])) < threshold\
                        and abs(color_mask_list[k][2] - ord(byte_array[3*index+2])) < threshold:
                    match = True
                    break

            if not match:
                byte_array[3 * index] = chr(0)
                byte_array[3 * index + 1] = chr(0)
                byte_array[3 * index + 2] = chr(0)

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
            iteration1 = next(data_out)            
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
    #kinect2pub = rospy.Publisher("/rosie/mask",Image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, process_image)
    #rospy.Subscriber("/kinect1/rgb/image_color", Image, top_image_callback)
    #rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, top_cloud_callback)
    #rospy.Subscriber("/kinect2/rgb/image_color", Image, mid_image_callback)
    #rospy.Subscriber("/kinect2/depth_registered/points", PointCloud2, mid_cloud_callback)
    localizer1 = Localizer()
    rospy.Subscriber("/kinect1/rgb/image_color", Image, localizer1.process_image)
    #rospy.Subscriber("/kinect1/depth_registered/points", PointCloud2, localizer1.process_cloud)
    
    rospy.spin()

if __name__ == "__main__":
    initialize()

