#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3, Wrench
from dvrk import mtm
import time
import PyKDL
import numpy as np
from std_msgs.msg import Int32

OPEN = 1
CLOSED = 2
MAX = 9

class impedanceController:
	def __init__(self):
		
		self.impedance = 50
		self.scale = 10

		self.m = mtm("MTMR")
		
		if not self.m.get_arm_current_state() == 'READY':
			self.m.shutdown() #Reset master position
			self.m.home()
			self.m.dmove_joint_one(np.pi/2,4)

		self.bias = np.zeros(3)

		self.unlocked = False

		self.disp = np.zeros(3)

		self.forces = []
		self.velocities = []
		self.velocity_set = []
		self.count=0

		impedance = rospy.Subscriber('impedance', Int32, self.changeImpedance)
		sub = rospy.Subscriber('/dvrk/MTMR/ati_ft', Wrench, self.control)
		rospy.spin()
	
	def gripState(self):
		if self.m.get_current_gripper_position() > 3:
			return CLOSED
		else:
			return OPEN

	def toNumpy(self, vector):
		if type(vector) == PyKDL.Vector:
			return np.array([vector[0], vector[1], vector[2]])
		elif type(vector) == np.ndarray:
			return vector

	def changeImpedance(self, val):
		self.impedance = val.data
		print('Impedance set to ' + str(self.impedance))

	def control(self, msg):
		force = np.array([msg.force.x, -msg.force.y, msg.force.z])

		if force[0] < 0:
			force[0] *= 2

		if self.gripState() == OPEN:
			if self.unlocked:
				self.m.lock_position()
				self.unlocked = False
			self.bias = force
		else:
			if self.unlocked == False:
				# self.m.unlock_position()
				self.unlocked = True

			imp = self.impedance * self.scale
			# print(imp)
			vel = (force - self.bias) / imp

			vel[0] = 0
			vel[2] = 0

			self.forces.append(force[1])
			self.velocities.append(self.m.get_current_twist_body()[1])
			self.velocity_set.append(vel[1])
			
			self.count += 1
			print(self.count)
			if self.count == 10000:
				np.savetxt('~/catkin_ws/src/impedance/forces-'+str(self.impedance)+'y.txt', np.array(self.forces))
				np.savetxt('~/catkin_ws/src/impedance/vels-'+str(self.impedance)+'y.txt', np.array(self.velocities))
				np.savetxt('~/catkin_ws/src/impedance/vels_desired-'+str(self.impedance)+'y.txt', np.array(self.velocity_set))
				print('Saved')
			self.m.dmove(PyKDL.Vector(vel[0], vel[1], vel[2]), blocking=False)


if __name__ == "__main__":
	c = impedanceController()