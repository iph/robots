#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from cs1567p1.srv import *
from math import fabs, sqrt, asin, degrees
from functools import partial
from time import clock

def orientation_to_theta(orientation):
    theta = degrees(2 * asin(orientation.z))
    if (theta < 0):
        theta += 360
    return theta

def linear_action(forward, cmd):
    cmd.linear.x = 0.3
    if not forward:
        cmd.linear.x *= -1

def rotate_action(counter_clock, cmd):
    cmd.angular.z = 0.6
    if not counter_clock:    
        cmd.angular.z *= -1 

def null_action(cmd):
    cmd.angular.z = 0.0
    cmd.linear.x = 0.0

def round_to(x, to = 1):
    return to * round(float(x) / to)

class Pursuit(object):
    def __init__(self):
        self.forward = True
        self.target = 0
        self.action = None

    def update(self, pose):
        """Updates goal, returns true if goal has been met"""
        print "Implement me!"

class DistancePursuit(Pursuit):
    def __init__(self):
        super(DistancePursuit, self).__init__()
        self.step = None

    def update(self, pose):
        self.target -= self.step(pose);
        return self.target < 0 or fabs(self.target) < 0.01

class RotatePursuit(Pursuit):
    def update(self, pose):
        #print "Target: ", self.target
        #print "Orientation: ", orientation_to_theta(pose.orientation)
        return fabs(self.target - orientation_to_theta(pose.orientation)) < 0.4

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
    def __init__(self):        
        self.pursuits = []
        self.pose = None

        rospy.init_node('Rosie', anonymous=True)
        rospy.wait_for_service('constant_command')
        rospy.wait_for_service('rosie_cmd')

        self.clear_odometry()
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.get_next_cmd = rospy.ServiceProxy('rosie_cmd', GetCommand)
        self.send_cmd = rospy.ServiceProxy('constant_command', ConstantCommand)

    def start(self):
        rospy.spin()

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
        print diff,"DURRRRR"
        if diff < 0: # flipped around circle
            diff += 360
        print diff, "HUUURRRR"
        return diff


    def plan_goal(self, goal):
        epsilon = 0.01
        if fabs(goal.delta_x - 0.0) < epsilon and fabs(goal.delta_y - 0.0) < epsilon and fabs(goal.delta_theta - 0.0) < epsilon:
            return False

        if fabs(goal.delta_x - 0.0) > epsilon:
            x_goal = DistancePursuit()
            #x_goal.target = fabs(round_to(self.pose.position.x + goal.delta_x, 0.5) - self.pose.position.x)
            #print "X: ", x_goal.target
            x_goal.target = fabs(goal.delta_x)
            x_goal.step = lambda pose: fabs(pose.position.x - self.pose.position.x)
            x_goal.action = partial(linear_action, goal.delta_x >= 0)
            self.pursuits.append(x_goal)
            self.pursuits.append(NullPursuit())
            return True
        
        if fabs(goal.delta_y - 0.0) > epsilon:
            y_goal = DistancePursuit()
            #y_goal.target = fabs(round_to(self.pose.position.y + goal.delta_y, 0.5) - self.pose.position.y)
            #print "Y: ", y_goal.target
            y_goal.target = fabs(goal.delta_y)
            y_goal.step = lambda pose: fabs(pose.position.y - self.pose.position.y)
            y_goal.action = partial(linear_action, goal.delta_y >= 0)
            self.pursuits.append(y_goal)
            self.pursuits.append(NullPursuit())
            return True

        if fabs(goal.delta_theta - 0.0) < epsilon:
            return False

        theta_goal = RotatePursuit()
        delta_theta = fabs(goal.delta_theta) % 360
        if goal.delta_theta < 0:
            delta_theta *= -1
        
        theta_goal.target = orientation_to_theta(self.pose.orientation) + delta_theta

        if goal.delta_theta >= 0: 
            # positive theta, so run around 360
            theta_goal.target = theta_goal.target % 360
            
        elif theta_goal.target < 0:
            # run around 360 opposite way
            theta_goal.target = 360 + theta_goal.target

        theta_goal.target = round_to(theta_goal.target, 90)
        print "Target: ", theta_goal.target
        theta_goal.action = partial(rotate_action, goal.delta_theta >= 0)
        self.pursuits.append(theta_goal)
        self.pursuits.append(NullPursuit())

        return True


    def odometry_callback(self, odom):
        if self.pose is None:
            self.pose = odom.pose.pose

        #print self.pose
        command = Twist()
        # see if we have any more goals to execute
        if self.pursuits:
            
            pursuit = self.pursuits[0]
            # update distance based on current odometry
            #print "Target: {0}".format(pursuit.target)
            
            if not pursuit.update(odom.pose.pose):
                pursuit.action(command)
            else:
                print odom.pose.pose
                self.pursuits.pop(0)
            
            #print "Target: {0}".format(pursuit.target)
        # no pursuits, grab and plan the next goal
        else:
            next_goal = self.get_next_cmd()
            self.plan_goal(next_goal)

        # update pose and send command
        self.pose = odom.pose.pose
        self.send_cmd(command)


if __name__ == "__main__":
    try:
        rosie = Rosie()
        rosie.start()
    except rospy.ROSInterruptException: pass
