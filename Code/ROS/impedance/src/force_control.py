#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from dvrk import mtm
import PyKDL
import numpy as np
import ati_sensor

OPEN = 1
CLOSED = 2

class mtm_force:
	def __init__(self):
		self.m = mtm(rospy.get_param("master_name"))
		self.m.shutdown()
		self.m.home()

		self.sensor = ati_sensor.Ati(path = rospy.get_param('path'), serialNum=rospy.get_param('serial'), armName="MTMR", comedi_arm_num=rospy.get_param('comedi_num'))
		self.bias = self.sensor.read()[:3]
		self.startPos = self.m.get_current_position()
		
		self.goal_sub = rospy.Subscriber('/dvrk/MTMR/desired_force', Vector3, self.update_goal)
		impedance = rospy.Subscriber('impedance', Int32, self.changeImpedance)

		self.impedance = 1

		self.desired_force = np.array([0.0,0.0,0.0])
		self.lastError = np.array([0.0,0.0,0.0])
		self.integral = np.array([0.0,0.0,0.0])

		self.P = np.array([0.8,0.8,0.8])
		self.I = np.array([0,0,0.0])
		self.D = np.array([-0,-0,-0])

		self.unlocked = False

		rate = rospy.Rate(3000)
		while not rospy.is_shutdown():
			self.control()
			rate.sleep()

	def changeImpedance(self, val):
		self.impedance = val.data

	def gripState(self):
		if self.m.get_current_gripper_position() < 0:
			return CLOSED
		else:
			return OPEN

	def control(self):
		if self.gripState() == CLOSED:
			if self.unlocked == False:
				self.m.unlock_position()
				self.unlocked = True

			#Find actual force
			force = self.sensor.read()[:3]
			force[0] *= -1

			#Caluclate error
			error = self.desired_force - force

			#Update Integral
			self.integral += error

			#Calculate Derivative
			derivative = error - self.lastError
			self.lastError = error

			#Calculate updated force to apply
			wrench = force + error * self.P + self.integral * self.I + derivative * self.D
			force[0] = 0
			force[2] = 0
			self.m.set_wrench_body_force(wrench)
		else:
			self.m.lock_position()
			self.unlocked = False
			self.bias = self.sensor.read()[:3]


	def update_goal(self, msg):
		self.desired_force = np.array([msg.x, msg.y, msg.z])

if __name__ == "__main__":
	rospy.init_node('force_control')
	control = mtm_force()
