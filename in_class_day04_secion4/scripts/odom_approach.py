#!/usr/bin/env python

"""this is a ros node that approaches a wall using proportional control"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class Wallapproach(object):
	"""docstring for ClassName"""


	def __init__(self, target=1):
		self.k = -.1		
		rospy.init_node("wall_approach")
		rospy.Subscriber("/odom", Odometry, self.process_odom)
		self.target = target
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.movement = Twist(linear=Vector3(x=1))
		self.x0 = None
		self.current_x = 0
		self.target_x = 1

	def process_odom(self, msg):
		if self.x0 == None:
			self.x0 = msg.pose.pose.position.x
			self.target_x = self.x0+1
		self.current_x = msg.pose.pose.position.x

	def run(self):
		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			error = self.current_x - self.target_x
			self.movement.linear.x = self.k*error
			self.pub.publish(self.movement)
			r.sleep()

if __name__ =="__main__":
	node = Wallapproach()
	node.run()

