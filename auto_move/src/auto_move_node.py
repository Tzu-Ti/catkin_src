#!/usr/bin/env python
import rospy
import time

from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class Auto_move(object):
        def __init__(self):
		self.joy = None
			
		# Setup Parameters
		self.v_gain = 0.41
		self.omega_gain = 8.3
		self.auto_switch = False
		self.stop_switch = True	
		self.turn_direction = True
		self.turn_list = []	

		# Subscribers
		self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
		self.sub_move = rospy.Subscriber("~move", Int32, self.go, queue_size=1)
                
		# Publishers
		self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

	############## Joy Control ##############
	def cbJoy(self, joy_msg):
		self.joy = joy_msg
		#print("joy:", self.joy.buttons)
		self.forced_stop()	
		if self.stop_switch:
			self.move()
			self.auto()

	def forced_stop(self):
		if (self.joy.buttons[7] == 1):
			self.stop_switch = False
			print("Turn to auto mode!")
		elif (self.joy.buttons[6] == 1):
			self.stop_switch = True
			print("Turn to control mode!")

	def auto(self):
		if (self.joy.buttons[0] == 1):
			self.forward()
		elif (self.joy.buttons[1] == 1):
			self.turn()
	def move(self):
		if not self.auto_switch:
			car_cmd_msg = Twist2DStamped()
               		car_cmd_msg.v = self.joy.axes[1] * self.v_gain
              		car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
			self.pub_car_cmd.publish(car_cmd_msg)
	###########################################

	def forward(self):
		self.auto_switch = True
                now = time.time()
                end = time.time()
                while (end-now) < 0.7 and self.auto_switch:
                        car_cmd_msg = Twist2DStamped()
                        car_cmd_msg.v = 0.25
                        car_cmd_msg.omega = 0
                        self.pub_car_cmd.publish(car_cmd_msg)
                        end = time.time()
                self.auto_switch = False

	def right(self):
		self.turn_list.append("right")
		self.auto_switch = True
                now = time.time()
                end = time.time()
                while (end - now) < 0.5:
                        car_cmd_msg = Twist2DStamped()
                        car_cmd_msg.v = 0.01
                        car_cmd_msg.omega = -8
                        self.pub_car_cmd.publish(car_cmd_msg)
                        end = time.time()
                self.auto_switch = False

	def left(self):
		self.turn_list.append("left")
                self.auto_switch = True
                now = time.time()
                end = time.time()
                while (end - now) < 0.5:
                        car_cmd_msg = Twist2DStamped()
                        car_cmd_msg.v = 0.01
                        car_cmd_msg.omega = 8
                        self.pub_car_cmd.publish(car_cmd_msg)
                        end = time.time()
                self.auto_switch = False

	# turn_direction True -> turn left
	# 		 False -> turn right
	def change_turn(self):
		if self.turn_list.count("right") >= 3:
			self.turn_direction = True
			self.turn_list = []
		elif self.turn_list.count("left") >= 3:
			self.turn_direction = False
			self.turn_list = []
		elif len(self.turn_list) == 5:
			self.turn_list = []

	def turn(self):
		self.change_turn()
		if self.turn_direction:
			self.left()
		else:
			self.right()

	def go(self, move_msg):
		if not self.stop_switch:
			move_msg = move_msg
			move = move_msg.data
		
			if move == 1:
				self.forward()
			elif move == 3:
				self.turn()
				self.forward()

			elif move == 100:
				self.stop_switch = True
			else:
				# stop!!
				self.auto_switch = False
if __name__ == "__main__":
        rospy.init_node("auto_move", anonymous=False)
        auto_move = Auto_move()
        rospy.spin()
