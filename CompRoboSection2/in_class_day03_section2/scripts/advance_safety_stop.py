#!/usr/bin/env python

""" This ROS node implements a simple controller that moves forward
    until it bumps into an obstacle.  When an obstacle is encountered
    the robot stops """

import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

WALL_LEFT = 0
WALL_RIGHT = 1
WALL_UNKNOWN = 2

wall_direction = WALL_UNKNOWN

STATE_E_STOP = 0
STATE_BACKUP = 1
STATE_TURN_LEFT = 2
STATE_TURN_RIGHT = 3
STATE_FORWARD = 4

state = STATE_FORWARD

def process_scan(msg):
    global wall_direction
    global turn_stop_time
    global state

    if msg.ranges[20] != 0 and msg.ranges[-20] != 0:
        if msg.ranges[20] > msg.ranges[-20]:
            wall_direction = WALL_RIGHT
        else:
            wall_direction = WALL_LEFT
    if rospy.Time.now() > backup_stop_time and state == STATE_BACKUP:
        if wall_direction == WALL_LEFT:
            state = STATE_TURN_RIGHT
            turn_stop_time = rospy.Time.now() + rospy.Duration(1.0)
        elif wall_direction == WALL_RIGHT:
            state = STATE_TURN_LEFT
            turn_stop_time = rospy.Time.now() + rospy.Duration(1.0)
        else:
            state = STATE_E_STOP
    if (rospy.Time.now() > turn_stop_time and
        (state == STATE_TURN_RIGHT or state == STATE_TURN_LEFT)):
        state = STATE_FORWARD

def process_bump(msg):
    global state
    global backup_stop_time
    if (msg.rightFront or
        msg.leftFront or
        msg.rightSide or
        msg.leftSide):
        if state == STATE_FORWARD:
            state = STATE_E_STOP
        elif state == STATE_E_STOP:
            print wall_direction
            if wall_direction == WALL_LEFT or wall_direction == WALL_RIGHT:
                state = STATE_BACKUP
                backup_stop_time = rospy.Time.now() + rospy.Duration(5.0)

safety_threshold = 0.5
rospy.init_node('safety_stop')

backup_stop_time = rospy.Time.now()
turn_stop_time = rospy.Time.now()

rospy.Subscriber("/bump", Bump, process_bump)
rospy.Subscriber("/scan", LaserScan, process_scan)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    if state == STATE_E_STOP:
        velocity_msg = Twist()
    elif state == STATE_TURN_LEFT:
        velocity_msg = Twist(angular=Vector3(z=0.5))
    elif state == STATE_TURN_RIGHT:
        velocity_msg = Twist(angular=Vector3(z=-0.5))
    elif state == STATE_FORWARD:
        velocity_msg = Twist(linear=Vector3(x=.1))
    elif state == STATE_BACKUP:
        velocity_msg = Twist(linear=Vector3(x=-.1))
    pub.publish(velocity_msg)
    r.sleep()