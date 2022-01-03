#! /usr/bin/env python
# Note, you may have to change the file locations at the bottom of this file

import rospy
from dvrk import mtm, psm
from geometry_msgs.msg import Wrench
import numpy as np
import PyKDL
import time

class joystick:
	def __init__(self):
		self.fx_scale = 0.005
		self.fy_scale = 0.005
		self.fz_scale = 0.005#0.02

		self.x_tolerance = 0.2*self.fx_scale
		self.y_tolerance = 0.2*self.fy_scale
		self.z_tolerance = 0.2*self.fz_scale

		#Initialize and home arms
		mtm_name = rospy.get_param("master_name")
		psm_name = rospy.get_param("slave_name")

		self.master = mtm(mtm_name)
		self.slave = psm(psm_name)

		if not self.master.get_arm_current_state() == 'READY':
			self.master.shutdown() #Reset master position
			self.master.home()
			self.master.dmove_joint_one(np.pi/2,4)
		else:
			self.check_home(self.master)
		if not self.slave.get_arm_current_state() == 'READY':
			self.slave.shutdown() #Reset slave position
			self.slave.home()
		else:
			self.check_home(self.slave)


		self.bias = rospy.wait_for_message('/dvrk/MTMR/ati_ft', Wrench)
		self.force = []
		self.pos = []
		self.vel = []
		self.posm = []

		self.count = 0
		self.mode = -1
		self.changed = 0

		#Subscribe to force data
		sub = rospy.Subscriber('/dvrk/MTMR/ati_ft', Wrench, self.force_control)
		print('Starting subscription')
		rospy.spin()

	def check_home(self, arm):
		joints = arm.get_current_joint_position()
		for angle in joints:
			if angle > 0.05:
				arm.shutdown()
				arm.home()
				self.master.dmove_joint_one(np.pi/2,4)
				return

	def slave_ws_limit(self):
		#joint 0 limit between -0.7 and 0.7
		#joint 1 limit between -0.7 and 1
		#joint 2 limit between -0.1 and 0.1
		pos = self.slave.get_current_joint_position()
		if abs(pos[0]) > 0.7:
			return False
		if abs(pos[2]-0.12) > 0.1:
			return False
		if pos[1] > 1 or pos[1] < -0.7:
			return False

		return True

	def force_control(self, msg):
		# print(msg)
		grip = self.master.get_current_gripper_position()
		# self.mode = 2
		if grip < 3:
			self.bias = msg
			if self.changed == 0:
				self.mode += 1
				# self.changed = 0
			self.changed = 1
		else:
			if self.changed == 1:
				time.sleep(0.5)

			slave_within_limit = self.slave_ws_limit()

			posmv = self.master.get_current_position().p
			posv = self.slave.get_current_position().p
			if slave_within_limit:
				self.posm.append(posmv.x())
				self.posm.append(posmv.y())
				self.posm.append(posmv.z())
				self.pos.append(posv.x())
				self.pos.append(posv.y())
				self.pos.append(posv.z())
			
			fx = (msg.force.x-self.bias.force.x)*self.fx_scale
			fz = -(msg.force.y-self.bias.force.y)*self.fy_scale
			fy = (msg.force.z-self.bias.force.z)*self.fz_scale

			if abs(fx) < self.x_tolerance:
				fx = 0
			if abs(fy) < self.y_tolerance:
				fy = 0
			if abs(fz) < self.z_tolerance:
				fz = 0

			if self.mode == 0:
				fy=0
				fz=0
			elif self.mode == 1:
				fz = 0
				fx = 0
			elif self.mode == 2:
				fx = 0
				fy = 0
			else:
				self.mode = 0

			if slave_within_limit:
				self.force.append(fx)
				self.force.append(fy)
				self.force.append(fz)
				v = self.slave.get_current_twist_body()
				self.vel.append(-v[1])
				self.vel.append(v[0])
				self.vel.append(v[2])

			f = PyKDL.Vector(fx, fy, fz)

			self.slave.dmove(f, blocking=False)
			if slave_within_limit:
				self.count += 1
			self.changed = 0

			print(self.count)
		if self.master.get_arm_current_state() == 'DISABLED':
			self.master.shutdown()
			self.master.home()
			self.master.dmove_joint_one(np.pi/2,4)
		if self.count == 60000:
			np.savetxt('~/catkin_ws/src/joystick/data/v1force-'+str(self.mode)+'.txt', np.array(self.force))
			np.savetxt('~/catkin_ws/src/joystick/data/v1pos-'+str(self.mode)+'.txt', np.array(self.pos))
			np.savetxt('~/catkin_ws/src/joystick/data/v1vel-'+str(self.mode)+'.txt', np.array(self.vel))
			np.savetxt('~/catkin_ws/src/joystick/data/v1mtmPos-' + str(self.mode) + '.txt', np.array(self.posm))
			self.count+=1
			print("Saved")


if __name__ == "__main__":
	j = joystick()