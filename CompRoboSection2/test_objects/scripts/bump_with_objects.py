#!/usr/bin/env python

import rospy
from neato_node.msg import Bump
from geometry_msgs.msg import Twist, Vector3

class EmergencyStop(object):
	def __init__(self):
		rospy.init_node('emergency_stop_node')
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		rospy.Subscriber("/bump", Bump, self.process_bump)
		self.has_bumped = False

	def process_bump(self, msg):
		if msg.frontLeft or msg.frontRight:
			self.has_bumped = True
		else:
			self.has_bumped = False

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			if self.has_bumped:
				self.pub.publish(Twist())
			else:
				self.pub.publish(Twist(linear=Vector3(x=0.1)))
			r.sleep()

if __name__ == '__main__':
	node = EmergencyStop()
	node.run()