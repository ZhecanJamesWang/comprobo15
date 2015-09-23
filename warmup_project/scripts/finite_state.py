#!/usr/bin/env python
"""Node for person following"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from nav_msgs.msg import Odometry

from obstacle_avoidance import ObstacleAvoidance, angle_diff

class ComboPFOA(object):
    """ Combo PersonFollowing (PF) and Obstacle Avoidance (OA) """

    # these are constants that let us give a name to each of our states
    # the names don't necessarily have to match up with the names of
    # the methods that are used to implement each behavior.
    OBSTACLE_AVOIDANCE_STATE = "obstacle_avoidance"
    PERSON_FOLLOW_STATE = "person_follow" 

    def __init__(self):
        rospy.init_node('combo_pfoa')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        rospy.Subscriber("/bump", Bump, self.process_bump)
        rospy.Subscriber("/odom", Odometry, self.process_odom)
        self.twist = Twist()
        
        # parameters pf
        self.target_dist = .6
        self.width = 20
        self.turn_k_pf = .1
        self.dist_k = 1
        self.degrees = 30
        self.checksum_queue = []
        self.epsilon_pf = 25
        self.queue_size = 5

        # parameters oa
        self.range = 90
        self.range_rad = math.pi / 180 * self.range
        self.num_quads = int(360/self.range)
        self.opposite_turns = [(i*self.range+180+self.range/2) % 360 for i in range(self.num_quads)]
        self.turn_k_oa = 1
        self.noturn = True
        self.turn_target = None
        self.unit_dist = 0.1
        self.object_distance = 0.9
        self.epsilon_oa = 0.1
        self.yaw = None
        self.ex_x, self.ex_y = None, None
        self.first_jank = True
        self.adjust_angle_flag = False
        self.turning = False
 
        # flags
        self.moving_object_detected = False
        self.bumped = False

    def checksum_with_angles(self, ranges):
        total = 0
        for i in range(-self.degrees, self.degrees + 1):
            # weight by angle
            total += ranges[i] * (i+30)
        return total

    def checksum_dumb(self, ranges):
        return sum(ranges[-self.degrees: self.degrees])

    def check_movement(self):
        # if the queue hasn't filled
        if len(self.checksum_queue) < self.queue_size:
            self.checksum_queue.append(self.checksum_with_angles(self.scan.ranges))
            print len(self.checksum_queue)
        else:
            std = np.std(self.checksum_queue)
            print "q, std: ", (self.checksum_queue, std)

            # clear the queue for the next check movement
            self.checksum_queue = []
            
            if std > self.epsilon_pf:
                # movement happened
                self.moving_object_detected = True
            else:
                self.moving_object_detected = False

    def stop_movement(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.moving_object_detected = False
    
    def process_bump(self, msg):
        if (msg.leftFront or
            msg.rightFront or
            msg.rightSide or
            msg.leftSide):
            self.bumped = True

    def process_scan(self, msg):
        self.scan = msg
        self.check_movement()
        print "self.moving_object_detected: ", self.moving_object_detected

    def process_odom(self, msg):
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
                self.params_to_turn()
            
    def person_following(self):
        while not rospy.is_shutdown():
            # scans
            left = self.scan.ranges[0:self.width/2]
            right = self.scan.ranges[360-self.width/2:360]
            all_valid_scans = [x for x in left + right if x > 0.0]
            left_count = sum([dist > 0.0 for dist in left if dist < 3])
            right_count = sum([dist > 0.0 for dist in right if dist < 3])
            if (len(all_valid_scans) != 0):
                mean_dist = sum(all_valid_scans) / len(all_valid_scans)
            else:
                mean_dist = self.target_dist
            turn_error = left_count - right_count
            dist_error = mean_dist - self.target_dist

            # velocity actions
            self.twist.linear.x = self.dist_k * dist_error
            self.twist.angular.z = self.turn_k_pf * turn_error

            if not self.moving_object_detected:
                return ComboPFOA.OBSTACLE_AVOIDANCE_STATE
            
            if self.bumped:
                # reset the bumped flag
                self.bumped = False

                self.stop_movement()
                return ComboPFOA.OBSTACLE_AVOIDANCE_STATE

        if self.bumped:
            self.stop_movement()

    def obstacle_avoidance(self):
        while not rospy.is_shutdown():
            
    def run(self):
        while not rospy.is_shutdown():
            if self.state == ComboPFOA.OBSTACLE_AVOIDANCE_STATE:
                self.state = self.obstacle_avoidance()
            elif self.state == ComboPFOA.PERSON_FOLLOW_STATE:
                self.state = self.person_follow()
            else:
                print "invalid state!!!" # note this shouldn't happen

if __name__ == '__main__':
    PersonFollowing().run()