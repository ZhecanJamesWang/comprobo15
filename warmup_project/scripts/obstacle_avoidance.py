#!/usr/bin/env python
"""Node for person following"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import math

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

class ObstacleAvoidance(object):
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_signal)
        rospy.Subscriber("/odom", Odometry, self.odom_signal)
        self.range = 90
        self.range_rad = math.pi / 180 * self.range
        self.num_quads = int(360/self.range)
        self.opposite_turns = [(i*self.range+180+self.range/2) % 360 for i in range(self.num_quads)]
        # self.opposite_turns_rad = [deg * math.pi / 180 for deg in self.opposite_turns]
        self.turn_k = .2
        self.twist = Twist()
        self.noturn = True
        self.turn_target = None
        self.unit_dist = 0.4
        self.yaw = None
        self.ex_x, self.ex_y = None, None
        self.first_jank = True
        self.adjust_angle_flag = False

    @staticmethod
    def convert_pose_to_xy_and_theta(pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]
    
    def odom_signal(self, msg):
        self.odom = msg
        self.x, self.y, self.yaw = self.convert_pose_to_xy_and_theta(self.odom.pose.pose)
        if self.first_jank:
            self.ex_x, self.ex_y = self.x, self.y
            self.final_target = self.yaw
            self.first_jank = False

        if self.noturn:
            d = math.sqrt((self.ex_x-self.x)**2 + (self.ex_y-self.y)**2)
            
            # once we've moved the appropriate distance
            if d >= self.unit_dist:
                # we are preparing to turn again
                self.noturn = False
                self.twist.linear.x = 0



    def scan_signal(self, msg):
        print "(self.noturn, yaw)", (self.noturn, self.yaw)
        if not self.noturn:
            self.scan = msg
            quad = [np.array(self.scan.ranges[i*self.range:(i+1)*self.range]) for i in range(self.num_quads)]
            quad = [quad[0],quad[3]]
            
            # remove the zeros
            quad = [q[q.nonzero()] for q in quad]

            quad_average = np.array([np.mean(q) for q in quad])
            
            # if nan, make big!
            for i, e in enumerate(quad_average):
                if np.isnan(e):
                    quad_average[i] = 10
            # print "quad_average", quad_average

            q = np.min(quad_average[quad_average.nonzero()])
            i = list(quad_average).index(q)
            print "quad", (i, q)
            
            # if i'm still close to an object
            if not self.adjust_angle_flag:
                if q < 1.5:
                    if self.yaw:
                        # print "YAW"
                        # print "opposite_turns", self.opposite_turns_rad[i]

                        if self.turn_target is None:
                            self.turn_target = [self.yaw-math.pi/2, self.yaw+math.pi/2][i]
                        
                        print "self.turn_target", self.turn_target

                        self.twist.angular.z = self.turn_k*(angle_diff(self.turn_target, self.yaw))
                        # if it has reached the desired angle
                        print "angle_diff: ", np.abs(angle_diff(self.turn_target, self.yaw))
                        if np.abs(angle_diff(self.turn_target, self.yaw)) < 0.1:
                            self.params_to_go_forward()
                            self.adjust_angle_flag = True
                # once I'm in safe distance
            else:
                self.adjust_original_angle()
        else:
            self.twist.linear.x = .1

    def adjust_original_angle(self):
        self.twist.angular.z = self.turn_k*(angle_diff(self.final_target, self.yaw))
        self.adjust_angle_flag = True
        if np.abs(angle_diff(self.final_target, self.yaw)) < 0.1:
            self.params_to_go_forward()
            self.adjust_angle_flag = False


    def params_to_go_forward(self):
        self.twist.angular.z = 0
        self.noturn = True
        self.turn_target = None
        # set initial x,y -> to prepare for move forward (in self.odom_signal)
        self.ex_x, self.ex_y = self.x, self.y


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.twist:
                # pass
                # print self.twist
                self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    ObstacleAvoidance().run()