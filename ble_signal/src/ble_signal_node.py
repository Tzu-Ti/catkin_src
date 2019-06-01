#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int32
from time import gmtime, strftime, sleep
import sys
import numpy as np
import subprocess
from sklearn import linear_model


class Ble_signal(object):
	def __init__(self):
		self.signal_list = []
		self.way_list = []
		self.reg = linear_model.LinearRegression()

		self.pub_way = rospy.Publisher("~signal", Int32, queue_size=1)
		self.scan()
	def scan(self):	
		while not rospy.is_shutdown():
			p = subprocess.Popen(['sudo python ~/ble1.py'], stdout=subprocess.PIPE, shell=True)		
			out = p.communicate()
			signal = out[0][0:-2]
			signal = float(signal)
			
			self.signal_list.append(signal)
			if len(self.signal_list) == 6:
				self.signal_list.pop(0)
				way = self.find_way()
				self.way_list.append(way)

			if len(self.way_list) == 4:
				if self.way_list.count(True) >= 3:
					self.send_way(True)
				else:
					self.send_way(False)
			#print("[ble_signal_node] Publish Successfully!")

	def send_way(self, way):
		way_msg = Int32()
		way_msg.data = way
		self.pub_way.publish(way_msg)
		#print("[ble_signal_node] Publish Successfully!")
		self.way_list = []

	def find_way(self):
		signal_list = self.signal_list
		print(signal_list)
		self.reg.fit(np.array(range(len(signal_list))).reshape(-1,1),np.array(signal_list).reshape(-1,1))
                slope = self.reg.coef_[0][0]
		print(slope)
		# Reach
		for i in signal_list:
			if i >= -40:
				self.send_way(100)
			
		# Wrong Way
		if slope < 0:
			return False
		# Right Way
		else:
			return True

if __name__ == "__main__":
	rospy.init_node("ble_signal", anonymous=False)
	ble_signal = Ble_signal()
	rospy.spin()
