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
        self.opposite_turns_rad = [deg * math.pi / 180 for deg in self.opposite_turns]
        self.turn_k = .7
        self.twist = Twist()
        self.turn_flag = False
        self.unit_dist = .2
        self.yaw = None
    
    @staticmethod
    def convert_pose_to_xy_and_theta(pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]
    


    def odom_signal(self, msg):
        self.odom = msg
        self.x, self.y, self.yaw = self.convert_pose_to_xy_and_theta(self.odom.pose.pose)
        if self.turn_flag:
            d = math.sqrt((self.ex_x-self.x)**2 + (self.ex_y-self.y)**2)
            if d == self.unit_dist:
                self.turn_flag = False


    def scan_signal(self, msg):
        if not self.turn_flag:   
            self.scan = msg
            quad = [self.scan.ranges[i*self.range:(i+1)*self.range] for i in range(self.num_quads)]
            quad_average = [np.mean(q) for q in quad]
            q = np.min(quad_average)
            i = quad_average.index(q)    
            if q < .3:
                if self.yaw:
                    self.twist.angular.z = self.turn_k*(angle_diff(self.opposite_turns_rad[i], self.yaw))
                    if angle_diff(self.opposite_turns_rad[i], self.yaw) < 0.1:
                        self.ex_x, self.ex_y = self.x, self.y
                        self.turn_flag = True
            elif q > .3:
            # elif q in [.3:2]:
                if angle_diff(0, self.yaw) > 0.1:
                    self.twist.angular.z = self.turn_k*(angle_diff(0, self.yaw))
                else:
                    self.twist.linear.x = 1

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.twist:
                self.pub.publish(self.twist)
            r.sleep()

if __name__ == '__main__':
    ObstacleAvoidance().run()