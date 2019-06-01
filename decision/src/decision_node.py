#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import numpy as np
import time
from sklearn import linear_model

class Decision(object):
	def __init__(self):
		# initialize parameters
		self.signal_list = []
		self.reg = linear_model.LinearRegression()

		# Publishers
		self.pub_move = rospy.Publisher("~move", Int32, queue_size=1)

		# Subscribers
		self.sub_signal = rospy.Subscriber("~signal", Int32, self.find_way, queue_size=1)
		self.sub_stop = rospy.Subscriber("~stop", Int32, self.ultrastop, queue_size=1)
		#self.sub_reach = rospy.Subscriber("~id", Int32, self.reach, queue_size=1)
	
	def reach(self, reach_msg):
		reach_msg = reach_msg
		reach = reach_msg.data
		print("Reach:", reach)

	def ultrastop(self, stop_msg):
		stop_msg = stop_msg
		stop = stop_msg.data
		if stop:
			print("Stop!")
			self.send_data(3)
			time.sleep(1)

	def find_way(self, way_msg):
		way_msg = way_msg
		way = way_msg.data
		#print("[decision_node] Successfully get the signal!")
		print("[decision_node]", way)
		if way == 100:
			self.send_data(100)
		elif way:
			self.send_data(1)
		else:
			self.send_data(3)
			time.sleep(1)

		
	
	# 0: stop
	# 1: forward
	# 2: turn
	# 3: left

	def send_data(self, data):
		move_msg = Int32()
		move_msg.data = data
		self.pub_move.publish(move_msg)
		#print("[decision_node] Publish Successfully!")
		#print(data)

if __name__ == "__main__":
	rospy.init_node("decision", anonymous=False)
	decision = Decision()
	rospy.spin()
