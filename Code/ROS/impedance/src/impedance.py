#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from dvrk import mtm
import PyKDL
import numpy as np
from std_msgs.msg import Int32

OPEN = 1
CLOSED = 2
MAX = 9

class impedanceController:
	def __init__(self):
		
		self.impedance = 1
		self.scale = 50

		self.m = mtm("MTMR")

		impedance = rospy.Subscriber('impedance', Int32, self.changeImpedance)
		self.desired_force_pub = rospy.Publisher('/dvrk/MTMR/desired_force', Vector3, queue_size=1)
		
		rate = rospy.Rate(1000)
		while not rospy.is_shutdown():
			self.control()

	def gripState(self):
		if self.m.get_current_gripper_position() > 3:
			return CLOSED
		else:
			return OPEN

	def toNumpy(self, vector):
		if type(vector) == PyKDL.Vector:
			return np.array([vector[0], vector[1], vector[2]])
		elif type(vector) == Vector3:
			return np.array([vector.x, vector.y, vector.z])
		elif type(vector) == np.ndarray:
			return vector[:3]

	def changeImpedance(self, val):
		self.impedance = val.data

	def control(self):
		if self.gripState() == CLOSED:
			#Update velocity
			v = self.toNumpy(self.m.get_current_twist_body())

			#force = self.toNumpy(force)
			imp = self.impedance * self.scale
			force = v * imp

			force[0] = 0
			force[2] = 0

			print(force)

			if abs(force[0]) > 9:
				force[0] = 9 * np.sign(force[0])
			if force[1] > 9:
				force(1) = 9 * np.sign(force[1])
			if force[2] > 9:
				force[2] = 9 * np.sign(force[2])

			self.desired_force_pub.publish(Vector3(force[0], force[1], force[2]))

if __name__ == "__main__":
	c = impedanceController()