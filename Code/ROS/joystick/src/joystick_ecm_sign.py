#! /usr/bin/env python

import rospy
from dvrk import mtm, ecm
from geometry_msgs.msg import Wrench
import numpy as np
import PyKDL
import time

PI = np.pi
DEADBAND = 0.1
DMOVE = 0.001

class joystick:
	def __init__(self):
		self.fx_scale = 1
		self.fy_scale = 0.8
		self.fz_scale = 1
		self.ty_scale = 4

		#Initialize and home arms
		mtm_name = rospy.get_param("master_name")
		ecm_name = rospy.get_param("ecm_name")

		self.master = mtm(mtm_name)
		self.ecm = ecm(ecm_name)
		
		self.master.dmove_joint_one(np.pi/2,4)

		self.bias = rospy.wait_for_message('/dvrk/MTMR/ati_ft', Wrench)
		self.force = []
		self.pos = []
		self.vel = []
		self.posm = []
		self.roll = []

		self.count = 0
		self.mode = -1
		self.changed = 0

		#Subscribe to force data
		sub = rospy.Subscriber('/dvrk/MTMR/ati_ft', Wrench, self.force_control)
		print('Starting subscription')
		rospy.spin()


	def ecm_ws_limit(self, scale=1):
		#joint 0 limit between -pi/2 and pi/2
		#joint 1 limit between -pi/4 and 2pi/5
		#joint 2 limit between -0.12 and 0.12
		#joint 3 limit between -pi/2 and pi/2
		pos = self.ecm.get_current_joint_position()
		if abs(pos[0]) > PI/2/scale or abs(pos[3]) > PI/2/scale:
			return False
		if abs(pos[2]-0.12) > 0.1/scale:
			return False
		if pos[1] > 2*PI/5/scale or pos[1] < -PI/4/scale:
			return False

		return True

	def deadband(self, force):
		deadBand = DEADBAND
		if abs(force) < deadBand:
			return 0
		else:
			return force

	def sign(self, force):
		if force > 0:
			return 1
		elif force < 0:
			return -1
		else: 
			return 0

	def force_control(self, msg):
		# print(msg)
		grip = self.master.get_current_gripper_position()
		# self.mode = 0
		if grip < 6:
			self.ecm.dmove(PyKDL.Vector(0,0,0),blocking=False)
			self.bias = msg
			if self.changed == 0:
				self.mode += 1
				# self.changed = 0
			self.changed = 1
		else:
			if self.changed == 1:
				time.sleep(0.5)

			ecm_within_limit = self.ecm_ws_limit(0.8)

			posmv = self.master.get_current_position().p
			posv = self.ecm.get_current_position().p
			if ecm_within_limit:
				self.posm.append(posmv.x())
				self.posm.append(posmv.y())
				self.posm.append(posmv.z())
				self.pos.append(posv.x())
				self.pos.append(posv.y())
				self.pos.append(posv.z())
				self.roll.append(self.ecm.get_current_joint_position()[3])
			
			fx = self.sign(self.deadband((msg.force.x-self.bias.force.x)))
			fy = self.sign(self.deadband((msg.force.y-self.bias.force.y)))
			fz = self.sign(self.deadband((msg.force.z-self.bias.force.z)))
			ty = self.sign(self.deadband(msg.torque.y))

			if ecm_within_limit:
				self.force.append(fx)
				self.force.append(fy)
				self.force.append(fz)
				self.force.append(ty)
				v = self.ecm.get_current_twist_body()
				self.vel.append(v[0])
				self.vel.append(v[1])
				self.vel.append(v[2])

			if self.mode == 0:
				fy = 0
				fz = 0
				ty = 0
			elif self.mode == 1:
				fz = 0
				fx = 0
				ty = 0
			elif self.mode == 2:
				fx = 0
				fy = 0
				ty = 0
			elif self.mode == 3:
				fx = 0
				fy = 0
				fz = 0
			else:
				self.mode = 0

			f = PyKDL.Vector(fx*DMOVE*self.fx_scale, fy*DMOVE*self.fy_scale, fz*DMOVE*self.fz_scale)
			
			if ty != 0:
				self.ecm.dmove_joint_one(ty*DMOVE*self.ty_scale,3, blocking=False)
			else:
				self.ecm.dmove(f, blocking=False)

			if ecm_within_limit:
				self.count += 1

			self.changed = 0
			print(self.count)
		
		# if self.count == 50000:
		# 	np.savetxt('~/catkin_ws/src/joystick/data/ecm/force-'+str(self.mode)+'.txt', np.array(self.force))
		# 	np.savetxt('~/catkin_ws/src/joystick/data/ecm/pos-'+str(self.mode)+'.txt', np.array(self.pos))
		# 	np.savetxt('~/catkin_ws/src/joystick/data/ecm/vel-'+str(self.mode)+'.txt', np.array(self.vel))
		# 	np.savetxt('~/catkin_ws/src/joystick/data/ecm/mtmPos-' + str(self.mode) + '.txt', np.array(self.posm))
		# 	if self.mode == 3:
		# 		np.savetxt('/home/david/catkin_ws/src/joystick/data/ecm/roll-'+str(self.mode)+'.txt', np.array(self.roll))
		# 	self.count+=1
		# 	print("Saved")


if __name__ == "__main__":
	j = joystick()