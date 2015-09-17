#!/usr/bin/env python
"""Node for person following"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class PersonFollowing(object):
    def __init__(self):
        rospy.init_node('person_following')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_signal)
        self.twist = Twist()
        self.target_dist = .6
        self.width = 20
        self.turn_k = .08
        self.dist_k = .3
  
    def scan_signal(self, msg):
        self.scan = msg
        left = self.scan.ranges[0:self.width/2]
        right = self.scan.ranges[360-self.width/2:360]
        print left + right
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

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.twist:
                self.pub.publish(self.twist)
            r.sleep()
if __name__ == '__main__':
    PersonFollowing().run()