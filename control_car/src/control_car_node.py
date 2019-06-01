#!/usr/bin/python
import rospy
from std_msgs.msg import Int32
from duckietown.msgs.msg import Twist2DStamped
from sensor_msgs.msg import Joy

class Control_car(object):
	def __init__(self):
		self.joy = None
		
		# Setup parameter
		self.v_gain = 0.41
		self.omega_gain = 0.83
		
		# Subscribers
		self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
		
		# Pubishers
		self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

	##### Joy Control #####
	def cbJoy(self, joy_msg):
		self.joy = joy_msg
		
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.v = self.joy.axes[1] * self.v_gain
		car_cmd_msg.omega = self.joy.axes[3] * self.omega_gain
		self.pub_car_cmd.publish(car_cmd_msg)

if __name__ == "__main__":
	rospy.init_node("control_car", anonymous=False)
	control_car = Control_car()
	rospy.spin()
