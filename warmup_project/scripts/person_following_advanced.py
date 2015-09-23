#!/usr/bin/env python
"""Node for person following"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump


class PersonFollowing(object):
    def __init__(self):
        rospy.init_node('person_following')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_signal)
        rospy.Subscriber("/bump", Bump, self.bump_signal)
        self.twist = Twist()
        self.target_dist = .6
        self.width = 20
        self.turn_k = .1
        self.dist_k = 1
        self.degrees = 30
        self.checksum_queue = []
        self.epsilon = 25
        self.movement = False
        self.queue_size = 5

    def bump_signal(self, msg):
        self.bump = msg

    def sum_scan_with_angles(self, ranges):
        total = 0
        for i in range(-self.degrees, self.degrees + 1):
            # weight by angle
            total += ranges[i] * (i+30)
        return total

    def sum_scan_dumb(self, ranges):
        return sum(ranges[-self.degrees: self.degrees])

    def check_movement(self):
        # if the queue hasn't filled
        if len(self.checksum_queue) < self.queue_size:
            self.checksum_queue.append(self.sum_scan_with_angles(self.scan.ranges))
            print len(self.checksum_queue)
        else:
            std = np.std(self.checksum_queue)
            print "q, std: ", (self.checksum_queue, std)

            # clear the queue for the next check movement
            self.checksum_queue = []
            
            if std > self.epsilon:
                # movement happened
                self.movement = True
            else:
                self.movement = False

    def scan_signal(self, msg):
        self.scan = msg
        self.check_movement()

        print "self.movement: ", self.movement

        if self.movement:
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

            self.twist.linear.x = self.dist_k * dist_error
            self.twist.angular.z = self.turn_k * turn_error
            if (self.bump.leftFront or self.bump.rightFront or self.bump.rightSide or self.bump.leftSide):
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.movement = False
            
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.twist:
                self.pub.publish(self.twist)
            r.sleep()
if __name__ == '__main__':
    PersonFollowing().run()