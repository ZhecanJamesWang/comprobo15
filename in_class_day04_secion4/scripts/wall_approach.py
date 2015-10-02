#!/usr/bin/env python

"""this is a ros node that approaches a wall using proportional control"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3


class Wallapproach(object):
	"""docstring for ClassName"""


	def __init__(self, target=1):
		self.K = .1		
		rospy.init_node("wall_approach")
		rospy.Subscriber("/scan", LaserScan, self.process_scan)
		self.target = target
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.movement = Twist(linear=Vector3(x=1))

	def process_scan(self, msg):
		dist = msg.ranges[0]
		if dist != 0.0:
			error = dist - self.target
			self.movement.linear.x = self.K * error
			print self.movement.linear.x 
			print error

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.pub.publish(self.movement)
			r.sleep()

if __name__ =="__main__":
	node = Wallapproach()
	node.run()

