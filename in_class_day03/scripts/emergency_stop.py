#!/usr/bin/env python

"""Node to drive Neato in a square (approximately) using odom measurements"""
import rospy
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

def move_forward():
    """Moves the robot forward until it bumps"""
    twister = Twist(linear=Vector3(x=0.5,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
    pub.publish(twister)

def stop():
    """Stops the robot and marks the has_bumped flag as True"""
    twister = Twist(linear=Vector3(x=0,y=0,z=0),angular=Vector3(x=0,y=0,z=0))
    pub.publish(twister)
    global has_bumped
    has_bumped = True

def callback(msg):
    bump = msg

    leftFront = bump.leftFront
    rightFront = bump.rightFront

    if not (leftFront or rightFront) and not has_bumped:
        move_forward()
    else:
        stop()

def listener():
    """Main function handler"""
    rospy.Subscriber('bump',Bump,callback,queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('drive_fwd')
    has_bumped = False

    while not rospy.is_shutdown():
        listener()
