#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

class Led_subscribe(object):
	def __init__(self):
		self.sub_data = rospy.Subscriber("~data", Int32, self.change_light, queue_size=1)
		self.ledPin = 11
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(11, GPIO.OUT)
		self.pwm = GPIO.PWM(11, 80)
		self.pwm.start(0)
		print("start pwm!!!")
	def change_light(self, data_msg):
		while not rospy.is_shutdown():
			data_msg = data_msg
			self.pwm.ChangeDutyCycle(data_msg.data)
		self.pwm.stop()
		GPIO.cleanup()		
#print("[led_subscribe_node] Successfully get the data!")		

if __name__ == "__main__":
	rospy.init_node("led_subscribe", anonymous=False)
	led_subscribe = Led_subscribe()
	rospy.spin()
