#!/usr/bin/env python

"""Node to drive Neato in a square (approximately) using odom measurements"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan

class three_state(object):
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.init_node('states')
        rospy.Subscriber("/bump", Bump, self.bump_signal)
        rospy.Subscriber("/scan", LaserScan, self.Laser_scan_signal)
        # rospy.spin()
        self.state = 0
        self.threshold = 1


    def bump_signal(self, msg):
        self.bump = msg
        # print "bump_signal", self.bump

    def Laser_scan_signal(self, msg):
        self.scan = msg.ranges[0]
    
    def move_forward(self):
        """Moves the robot forward until it bumps"""
        twister = Twist(linear=Vector3(x=0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
        self.pub.publish(twister)

        print "move_forward"
        try:
            if (self.bump.leftFront or self.bump.rightFront):
                self.state += 1
                print 'bump'
        except:
            pass

    def rotate_left(self):
        twister = Twist(linear=Vector3(x=0,y=0,z=0),angular=Vector3(x=0,y=0,z=1))
        self.pub.publish(twister)
        # rospy.sleep(1)
        self.state = 0  

    def move_backward(self):
        """Stops the robot and marks the has_bumped flag as True"""
        twister = Twist(linear=Vector3(x=-0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
        self.pub.publish(twister)
        try:
            print self.scan
            if self.scan >= self.threshold:
                self.state += 1
        except:
            pass

    def run(self):
        while not rospy.is_shutdown():
            # print self.state
            if self.state == 0 :
                self.move_forward()
            if self.state == 1:
                self.move_backward() 
            if self.state == 2:
                self.rotate_left()


if __name__ == '__main__':
    robot = three_state()
    robot.run()
    
