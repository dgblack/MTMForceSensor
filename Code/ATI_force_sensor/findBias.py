from ati import Ati
import numpy as np
from dvrk import mtm

path = '/home/david/Desktop/sensor_software/ATI'

s = Ati(path) 

m = mtm("MTMR")
m.dmove_joint_one(np.pi/2,3)
# m.dmove_joint_one(np.pi/2,4)

d = []
angle = []

#Take readings in a for loop
for i in range(200):
	d.append(s.read()) #Poll 1 reading
	m.dmove_joint_one(-np.pi/200,3) #Gradually scan across
	# m.dmove_joint_one(-np.pi/200,4)
	angle.append(np.pi/2-i*np.pi/200)

#Save the data
np.savetxt(path + '/data1.txt', np.array(d))
np.savetxt(path + '/angle1.txt', np.array(angle))
