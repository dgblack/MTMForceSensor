#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from dvrk import mtm
import time

v_threshold_max = 0.05 #At v<0.1, arm considered unmoving, if it has been moving for a while
v_threshold_min = 0.015 #If the arm has been at rest, it is considered to be moving if v>0.015
f_threshold_moving = 0.3 #If arm is unlocked and moving, don't want to lock up too easily
f_threshold_unmoving = 0.5 #If arm is unlocked but not moving, should lock up easily
f_threshold_z = 2.8 #Force required just to hold up the MTM
f_threshold_locked_min = 0.75 #If arm is locked and has been for a while, should be easy to unlock
f_threshold_locked_max = 1.1 #If arm was just locked, should be hardish to unlock
min_unmoving_time = 0.3 #Number of seconds the arm should be unmoving before locking is allowed
arm = mtm('MTMR')
arm.home()
is_unlocked = False
count = 0
locked_count = 0
start_time = -1
print('Starting gravity compensation')

def norm(wrench):
	if type(wrench) == Wrench:
		f = wrench.force
		fnorm = (f.x**2 + f.y**2 + f.z**2)**(0.5)
	else:
		fnorm = (wrench[0]**2 + wrench[1]**2 + wrench[2]**2)**0.5
	return fnorm

def lock_if_no_force(msg):
	global is_unlocked
	global count
	global locked_count
	global start_time

	fx = abs(msg.force.x)
	fy = abs(msg.force.y)
	fz = abs(msg.force.z)
	
	v = norm(arm.get_current_twist_body()[:3])

	vthresh = v_threshold_min * (1+count/1000)
	if vthresh > v_threshold_max:
		vthresh = v_threshold_max

	lthresh = f_threshold_locked_max * (1-locked_count/1000)
	if lthresh < f_threshold_locked_min:
		lthresh = f_threshold_locked_min
		lthreshz = lthresh
	else:
		lthreshz = f_threshold_z

	if is_unlocked and v > vthresh: #Arm is unlocked and moving
		start_time = -1
		print(1)
		count += 1
		#Require all three forces to be very small before locking
		if (fx <= f_threshold_moving and fy <= f_threshold_moving and fz <= 1.2*f_threshold_moving):
			arm.lock_position()
			is_unlocked = False
			count = 0
		
	elif is_unlocked: #Arm is unlocked but not moving
		print(2)
		count += 1
		if is_unlocked == False:
			start_time = time.time()
		#To lock, require all three forces to be smallish except z, which is 2.5 just by holding up the arm
		if (fx <= f_threshold_unmoving and fy <= f_threshold_unmoving and fz <= f_threshold_z):
			if (time.time()-start_time > min_unmoving_time):
				arm.lock_position()
				is_unlocked = False
				count = 0

	else: #Arm is locked
		start_time = -1
		print(3)
		locked_count += 1
		#Unlock at slight push in any direction
		if (fx >= lthresh or fy >= lthresh or fz >= lthreshz):
			arm.unlock_position()
			is_unlocked = True
			locked_count = 0

sub = rospy.Subscriber('/dvrk/MTMR/ati_ft', Wrench, lock_if_no_force)
rospy.spin()