#! /usr/bin/env python

import rospy
from dvrk import mtm, ecm
from sensor_msgs.msg import JointState
from twisted.internet import task, reactor
import numpy as np
import PyKDL
import time

PI = np.pi
DMOVE = 0.001
DEADBAND = [0.3, 0.05, 0.4, 0.013]
SCALE = [1, 0.5, -1, 4]

class joystick:
	def __init__(self):

		#Initialize and home arms
		mtm_name = rospy.get_param("master_name")
		ecm_name = rospy.get_param("ecm_name")

		self.master = mtm(mtm_name)
		self.ecm = ecm(ecm_name)
		
		self.master.dmove_joint_one(np.pi/2,4)

		self.bias = self.master.get_current_joint_effort()
		self.force = []
		self.pos = []
		self.vel = []
		self.posm = []
		self.roll = []

		self.count = 0
		self.mode = -1
		self.changed = 0

		#Subscribe to force data
		sub = rospy.Subscriber('/dvrk/MTMR/state_joint_current', JointState, self.force_control)
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

	def force_control(self, msg):
		grip = self.master.get_current_gripper_position()
		msg = msg.effort
		self.mode = 0
		if grip < 6:
			self.ecm.dmove(PyKDL.Vector(0,0,0),blocking=False)
			self.bias = msg
			if self.changed == 0:
				# self.mode += 1
				self.changed = 0
			self.changed = 1
		else:
			if self.changed == 1:
				if self.mode == 0:
					print('Controlling X-axis')
				elif self.mode == 1:
					print('Controlling Y-Axis')
				elif self.mode == 2:
					print('Controlling Z-Axis')
				elif self.mode == 3:
					print('Controlling Roll')
				time.sleep(0.5)
				self.changed = 0

			ecm_within_limit = self.ecm_ws_limit(0.8)

			#Save some data
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
			
			#Bias the force readings
			force = [0,0,0,0]
			force[0] = msg[0]-self.bias[0] #+ msg[5]*10
			force[1] = msg[4]#-self.bias[5]
			force[2] = msg[2]-self.bias[2]
			force[3] = msg[6]-self.bias[6]
			
			#Implement a deadband and sign function
			for i in range(4):
				if abs(force[i]) < DEADBAND[i]:
					force[i] = 0
				if force[i] < 0:
					force[i] = -1
				else:
					force[i] = 1

			#Save some more data
			if ecm_within_limit:
				self.force.append(force[0])
				self.force.append(force[1])
				self.force.append(force[2])
				self.force.append(force[3])
				v = self.ecm.get_current_twist_body()
				self.vel.append(v[0])
				self.vel.append(v[1])
				self.vel.append(v[2])

			# Set all forces to 0 except for the one whose mode it is
			if self.mode > 3:
				self.mode = -1
			else:
				for i in range(4):
					if self.mode != i:
						force[i] = 0

			f = PyKDL.Vector(force[0]*DMOVE*SCALE[0], force[1]*DMOVE*SCALE[1], force[2]*DMOVE*SCALE[2])

			if force[3] != 0:
				self.ecm.dmove_joint_one(force[3]*DMOVE*SCALE[3],3, blocking=False)
			else:
				self.ecm.dmove(f, blocking=False)

			if ecm_within_limit:
				self.count += 1

			if self.count % 500 == 0:
				print(self.count)
		
		#Save data
		if self.count == 5000:
			np.savetxt('~/catkin_ws/src/joystick/data/ecm/eff_force-'+str(self.mode)+'.txt', np.array(self.force))
			np.savetxt('~/catkin_ws/src/joystick/data/ecm/eff_pos-'+str(self.mode)+'.txt', np.array(self.pos))
			np.savetxt('~/catkin_ws/src/joystick/data/ecm/eff_vel-'+str(self.mode)+'.txt', np.array(self.vel))
			np.savetxt('~/catkin_ws/src/joystick/data/ecm/eff_mtmPos-' + str(self.mode) + '.txt', np.array(self.posm))
			if self.mode == 3:
				np.savetxt('/home/david/catkin_ws/src/joystick/data/ecm/eff_roll-'+str(self.mode)+'.txt', np.array(self.roll))
			self.count+=1
			print("Saved")


if __name__ == "__main__":
	j = joystick()