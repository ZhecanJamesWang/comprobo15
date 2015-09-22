#!/usr/bin/env python

""" The script will be used to explore basics of ROS messages in Python """

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header
import rospy

rospy.init_node('test_message')

point_msg = Point(x=1.0, y=2.0, z=0.0)
header_msg = Header(stamp=rospy.Time.now(),
				    frame_id="odom")
msg = PointStamped(header=header_msg, point=point_msg)
pub = rospy.Publisher("/my_point", PointStamped, queue_size=10)

r = rospy.Rate(10)
while not rospy.is_shutdown():
	pub.publish(msg)
	r.sleep()