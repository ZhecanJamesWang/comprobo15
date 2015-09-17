#!/usr/bin/env python
"""Node for wall following"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

class WallFollowing(object):
    def __init__(self):
        rospy.init_node('wall_following')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/bump", Bump, self.bump_signal)
        self.twist = None
  
    def bump_signal(self, msg):
        self.bump = msg
        if self.bump.leftFront:
            self.twist = Twist(linear=Vector3(x= -0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=-1.5))
        elif self.bump.rightFront:
            self.twist = Twist(linear=Vector3(x= -0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=1.5))
        else:
            self.twist = Twist(linear=Vector3(x= 1,y=0,z=0))
    def run(self):
        R = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.twist:
                self.pub.publish(self.twist)
            R.sleep()
if __name__ == '__main__':
    WallFollowing().run()