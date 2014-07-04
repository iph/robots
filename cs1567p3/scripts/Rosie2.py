#!/usr/bin/env python


import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
from cs1567p3.srv import *
from math import fabs, sqrt, asin, degrees
from functools import partial
from time import clock
from threading import *

def orientation_to_theta(orientation):
    """Takes an orientation object and outputs the z component as degrees"""
    theta = degrees(2 * asin(orientation.z))
    if (theta < 0):
        theta += 360
    return theta

def linear_action(forward, cmd):
    """Action to apply to move linearly"""
    cmd.linear.x = 0.1
    if not forward:
        cmd.linear.x *= -1

def rotate_action(counter_clock, cmd):
    """Action to apply to rotate"""
    cmd.angular.z = 0.6
    if not counter_clock:    
        cmd.angular.z *= -1 

def null_action(cmd):
    """Action to apply to do nothing"""
    cmd.angular.z = 0.0
    cmd.linear.x = 0.0

class Pursuit(object):
    def __init__(self):
        self.target = 0
        self.action = None

    def update(self, pose):
        """Updates goal, returns true if goal has been met"""
        print "Implement me!"

    def start(self, pose):
        pass

class DistancePursuit(Pursuit):
    def __init__(self):
        super(DistancePursuit, self).__init__()
        self.step = None

    def update(self, pose):
        self.target -= self.step(pose);
        return self.target < 0 or fabs(self.target) < 0.01

class RotatePursuit(Pursuit):
    def update(self, pose):
        diff = fabs(self.target - orientation_to_theta(pose.orientation))
        # if target is 0 or 360, return check against 360 - eps/2, 0 + eps/2
        if fabs(self.target - 0.0) < 0.01 or fabs(self.target - 360.0) < 0.01:
            return diff < 0.25 or fabs(diff - 360) < 0.25
        # otherwise just check against (target - eps, target + eps)
        return diff < .4
    def start(self, pose):
        pass
        

class NullPursuit(Pursuit):
    def __init__(self):
        super(NullPursuit, self).__init__()
        self.target = 0.05
        self.time = 0.0
        self.action = null_action
        
    def update(self, pose):
        if (self.time == 0.0):
            self.time = clock()
            return False
        delta_time = clock() - self.time
        return delta_time > self.target
        
class Rosie(object):
    ROTATE_DELTA = 25
    def __init__(self):
        self.lock = Lock()        
        self.pursuits = []
        self.pose = None
        self.start_time = None
        self.start_pose= None

        rospy.init_node('Rosie', anonymous=True)
        rospy.wait_for_service('constant_command')

        self.clear_odometry()
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.on_bump)
        self.const_cmd = rospy.ServiceProxy('constant_command', ConstantCommand)


    def send_cmd(self, cmd):
        self.const_cmd(cmd)
    
            
    def start(self):
        self.start_time = clock()
        self.move_then_probe();

    def move_then_probe(self):
        self.move(0.5)
        self.rotate(-1 * Rosie.ROTATE_DELTA)
        #self.move(1000)
        

    def clear_odometry(self):
        pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        while pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        pub.publish(Empty())


    def rotate_step(self, counter_clock, pose):
        prev_theta = orientation_to_theta(self.pose.orientation)
        curr_theta = orientation_to_theta(pose.orientation)

        if counter_clock:
            diff = curr_theta - prev_theta
        else:
            diff = prev_theta - curr_theta
        if diff < 0: # flipped around circle
            diff += 360
        return diff

    def move(self, dist):
        goal = DistancePursuit()
        goal.target = fabs(dist)
        goal.step = lambda pose: sqrt( fabs(self.pose.position.x - pose.position.x)**2 + fabs(self.pose.position.y - pose.position.y)**2)
        goal.action = partial(linear_action, dist >= 0)
        self.pursuits.append(goal)
        self.pursuits.append(NullPursuit())

    def rotate(self, theta):
        theta_goal = RotatePursuit()
        delta_theta = fabs(theta) % 360
        if theta < 0:
            delta_theta *= -1
        
        theta_goal.target = orientation_to_theta(self.pose.orientation) + delta_theta

        if theta >= 0: 
            # positive theta, so run around 360
            theta_goal.target = theta_goal.target % 360
            
        elif theta_goal.target < 0:
            # run around 360 opposite way
            theta_goal.target = 360 + theta_goal.target;

        print "theta_goal.target: ", theta_goal.target

        theta_goal.action = partial(rotate_action, theta >= 0)
        self.pursuits.append(theta_goal)
        self.pursuits.append(NullPursuit())

    def stop(self):
        self.pursuits = [NullPursuit()]

    def on_bump(self, bump):
        if bump.state is 1:
            self.lock.acquire()
            # stop rosie
            self.stop()
            # move backward
            self.move(-0.1)
            # rotate away from wall
            self.rotate(Rosie.ROTATE_DELTA )
            self.lock.release()

    def is_done(self, pose):
        if self.start_time is None or self.start_pose is None:
            return False
        
        pose_epsilon = 0.2
        startup_time = 2
        x_diff = fabs(self.start_pose.position.x - pose.position.x)
        y_diff = fabs(self.start_pose.position.y - pose.position.y)
        t_diff = clock() - self.start_time
        print "x_diff: ", x_diff, " y_diff: ", y_diff, " t_diff: ", t_diff

        return (x_diff < pose_epsilon and y_diff < pose_epsilon and t_diff > startup_time)
            
    def odometry_callback(self, odom):
        pose = odom.pose.pose
        if self.pose is None:
            self.start_pose = pose
            self.pose = pose
            self.start()
            return

        if self.is_done(pose):
            self.lock.acquire()
            self.stop()
            self.lock.release()

        #print "Pose: ", self.pose
        command = Twist()
        # see if we have any more goals to execute
        self.lock.acquire()
        if self.pursuits:
            
            pursuit = self.pursuits[0]
            # update distance based on current odometry
            #print "Target: {0}".format(pursuit.target)
            
            if not pursuit.update(odom.pose.pose):
                pursuit.action(command)
            else:
                self.pursuits.pop(0)
        else:
            self.move_then_probe()
     
        self.lock.release()

        #print "Theta: ", orientation_to_theta(odom.pose.pose.orientation)

        # update pose and send command
        self.pose = odom.pose.pose
        self.send_cmd(command)


if __name__ == "__main__":
    try:
        rosie = Rosie()
        rospy.spin()
    except rospy.ROSInterruptException: pass
