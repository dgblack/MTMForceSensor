#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Wrench
from dvrk import mtm

#TOTAL GUESS
f_tolerance = 0.5
t_tolerance = 5

armr = mtm("MTMR")
arml = mtm("MTML")



def norm(wrench):
	f = wrench.force
	fnorm = (f.x**2 + f.y**2 + f.z**2)**(0.5)
	t = wrench.torque
	tnorm = (t.x**2 + t.y**2 + t.z**2)**(0.5)
	return fnorm, tnorm

def lock_if_no_force_r(msg):
	f, t = norm(msg)
	if (f < f_tolerance) and (t < t_tolerance):
		armr.lock_orientation()
	else:
		armr.unlock_orientation()

def lock_if_no_force_l(msg):
	f, t = norm(msg)
	if (f < f_tolerance) and (t < t_tolerance):
		arml.lock_orientation()
	else:
		arml.unlock_orientation()

rospy.init_node('mtm_force_unlock')
subr = rospy.Subscriber("ati_mtmr_data", Wrench, lock_if_no_force_r)
subl = rospy.Subscriber("ati_mtml_data", Wrench, lock_if_no_force_l)
rospy.spin()