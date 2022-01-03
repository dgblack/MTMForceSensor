#! /usr/bin/env python

import comedi
import numpy as np
from dvrk import mtm
import time
import xml.etree.cElementTree as xml
from geometry_msgs.msg import Wrench, Vector3
import PyKDL

class Calibration:
	def __init__(self, ft_vector, frame):
		#Adjust finger grip rotation matrix to force sensor frame
		#Force sensor is -90deg rotation about x from finger grips
		#frame.M.DoRotX(np.pi/2)

		#Split into force vector and torque vector
		f = PyKDL.Vector(ft_vector[0],ft_vector[1],ft_vector[2])
		t = PyKDL.Vector(ft_vector[3],ft_vector[4],ft_vector[5])

		#Take inverse of rotation matrix and transform force and torque vectors into base reference frame
		inv = frame.M.Inverse()
		self.bias_force = inv * f
		self.bias_torque = inv * t
		# self.bias_force = f
		# self.bias_torque = t

	def bias(self, frame, ft_vector):
		#Adjust finger grip rotation matrix to force sensor frame
		#Force sensor is -90deg rotation about x from finger grips
		#frame.M.DoRotX(np.pi/2)

		#Split into force vector and torque vector
		f = PyKDL.Vector(ft_vector[0],ft_vector[1],ft_vector[2])
		t = PyKDL.Vector(ft_vector[3],ft_vector[4],ft_vector[5])

		#Take inverse of rotation matrix and transform force and torque vectors into base reference frame 
		inv = frame.M.Inverse()
		f_rot = inv * f
		t_rot = inv * t

		#Subtract bias in inertial coord system from transformed force and toque
		f_rot -= self.bias_force
		t_rot -= self.bias_torque
		# f -= self.bias_force
		# t -= self.bias_torque

		#Rotate the force and torque back to the current coordinate system
		f = frame.M * f_rot
		t = frame.M * t_rot

		#Recombine force and torque into one vector
		return np.array([f.x(), f.y(), f.z(), t.x(), t.y(), t.z()])

class Ati:

	def __init__(self, path, serialNum="FT24092", armName="MTMR", calibration=None, comedi_arm_num=0):

		self.num_channels = 6
		self.sub_device = 0
		self.aref = 0 # = AREF_GROUND
		self.range = 0 #0 = [-10V, 10V], 1 = [-5V, 5V]
		self.sample_rate = 1500.0
		self.armName = armName

		self.dev = []
		self.cal_matrix = np.empty((self.num_channels, self.num_channels))
		self.arm = mtm(armName)

		#Create a comedi connection for connected device
		self.dev = comedi.comedi_open("/dev/comedi" + str(comedi_arm_num))
		#Read in calibration matrix
		self.cal_matrix = self.read_cal_file(path + "/calibration/" + serialNum + ".cal")

		self.calibration = calibration
		self.calibrate()

		print("Node started with sensor " + serialNum + " on " + armName + "\n")

	def __del__(self):
		comedi.comedi_close(self.dev)

	def read_cal_file(self, filename):
		"""
		Read the .cal file into a calibration matrix
		"""
		root = xml.parse(filename).getroot()
		axes = root.findall('Calibration/UserAxis')
		return np.array([[float(axes[i].attrib['values'].split()[j]) for j in range(self.num_channels)] for i in range(self.num_channels)])

	def calibrate(self, tolerance = 2):
		"""
		Find the value with which to bias each channel. Note that there are multiple channels
		and multiple devices so an n-element vector is returned where n is the number of sensors
		and each element is a Calibration object
		:param tolerance max magnitude of force when it should actually be 0
		"""
		print("Calibrating ATI F/T Sensor...\n")
		#Calibrate unless told not to
		if self.calibration == None:

			#Average num_samples measurements to remove outliers
			num_samples = 10
			avg = np.zeros(self.num_channels)
			for i in range(num_samples):
				dat = np.array(self.read_raw())
				avg += dat
			avg /= num_samples

			pos = self.arm.get_current_position()
			self.calibration = Calibration(np.dot(self.cal_matrix,avg), pos)

		#Ensure the calibration was successful and a reading returns a force around 0. Otherwise redo
		check = self.read()
		norm = np.sqrt(check[0]**2 + check[1]**2 + check[2]**2)
		print(norm)
		# if norm > tolerance:
		# 	self.calibrate()

		print("Calibration Successful\n")

	def read(self):
		"""
		Read one sample from all the connected sensors. Convert the reading to force and torque in N
		:return 6xn matrix where each column is a measurement (6 components of force/torque) and there are n connected devices,
		so each column corresponds to one connected sensor
		"""

		data_array = self.read_raw()
		# data_array = np.dot(self.cal_matrix, data_array)
		pos = self.arm.get_current_position()
		data_array = self.calibration.bias(pos, np.dot(self.cal_matrix, data_array))
		return data_array

	def read_raw(self):
		"""
		Read directly from 6 channels of the sensor and convert from uint-16 to voltage with no additional processing
		"""
		data_array = np.empty(self.num_channels)
		for chan in range(self.num_channels):
			rc, data_array[chan] = comedi.comedi_data_read(self.dev, self.sub_device, chan, self.range, self.aref)
			data_array[chan] = self.comedi_to_phys(data_array[chan])
		return data_array

	def comedi_to_phys(self, data):
		"""
		Convert measured uint-16 to voltage
		"""
		return (data - 2**15) / 2**15 *10