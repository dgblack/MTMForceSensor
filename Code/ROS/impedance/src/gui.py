#! /usr/bin/env python

import rospy
from PyQt5.QtWidgets import (QApplication, QSlider, QHBoxLayout, QWidget, QMainWindow)
from PyQt5.QtCore import Qt
from std_msgs.msg import Int32
import time

class  gui(QMainWindow):
	def __init__(self):
		super(gui, self).__init__()
		self.pub = rospy.Publisher('impedance', Int32, queue_size=1)
		slider = QSlider(Qt.Horizontal, self)
		slider.setGeometry(30,40,200,30)
		slider.setMinimum(0.5)
		slider.setMaximum(100)
		slider.setTickPosition(QSlider.NoTicks)
		slider.setTickInterval(100)
		value=50
		slider.setValue(value)
		slider.valueChanged[int].connect(self.changeImpedance)
		self.setGeometry(50,50,320,100)
		self.setWindowTitle("Impedance Control")
		self.show()

	def changeImpedance(self, val):
		self.pub.publish(Int32(val))


if __name__ == "__main__":
	rospy.init_node('impedance_gui')
	app = QApplication([])
	GUI = gui()
	app.exec_()