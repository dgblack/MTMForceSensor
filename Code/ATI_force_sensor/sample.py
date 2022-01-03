# from ati import Ati
# import numpy as np

# NUM_SAMPLES = 10000
# path = '/home/david/Desktop/sensor_software/ATI'

# s_arm = Ati(path) #The default serial number and port number are the sensor mounted on the MTM
# s_2 = Ati(path, serialNum="FT24093", comediNum=1) #The other sensor has these values

# #Prepare empty numpy arrays
# data_arm = []
# data_2 = []

# #Take readings in a for loop
# for i in range(NUM_SAMPLES):
# 	data_arm.append(s_arm.read()) #Poll 1 reading
# 	data_2.append(s_2.read())

# #Save the data
# np.savetxt('/home/david/Desktop/ATI/data_arm.txt', np.array(data_arm)
# np.savetext('/home/david/Desktop/ATI/data_2.txt', np.array(data_arm))


###### For exact loop timing and measurement at a specific rate:  #######

from twisted.internet import task, reactor
from ati import Ati
import numpy as np
import time
import sys

RATE = 1500.0
NUM_SAMPLES = 1500
path = '/home/david/Desktop/sensor_software/ATI'

s_arm = Ati(path) #The default serial number and port number are the sensor mounted on the MTM
s_2 = Ati(path, serialNum="FT24093", comediNum=1) #The other sensor has these values


class looper:
	def __init__(self, loop_rate, num_samples):
		self.rate = loop_rate
		self.num_samples = num_samples
		#Prepare empty arrays
		self.data_arm = []
		self.data_2 = []

		self.count = 0
		self.start_time = time.time()
		self.loop = task.LoopingCall(self.measure)
		self.loop.start(1/RATE)
		reactor.run()

	def measure(self):

		#Poll and store 1 reading from each sensor
		self.data_arm.append(s_arm.read())
		self.data_2.append(s_2.read())

		self.count += 1
		# print(self.count)

		if self.count == self.num_samples:
			#Save the data
			elapsed = time.time()-self.start_time

			# np.savetxt('/home/david/Desktop/sensor_software/ATI/data_arm.txt', np.array(self.data_arm))
			# np.savetxt('/home/david/Desktop/sensor_software/ATI/data_2.txt', np.array(self.data_2))
			
			print('Took ' + str(self.num_samples) + ' measurements in ' + str(elapsed) + ' seconds at ' + str(self.rate) + 'Hz')
			print('This constitutes ' + str(100*abs((self.num_samples/elapsed)-self.rate)/self.rate) + '% error in rate')
			#Stop the loop
			self.loop.stop()


l = looper(RATE, NUM_SAMPLES)