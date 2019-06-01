#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

class Ultrasound(object):
	def __init__(self):
		# initailize parameters
		self.trig_pin = 23
		self.echo_pin = 24
		
		# Publishers
		self.pub_stop = rospy.Publisher("~stop", Int32, queue_size=1)
		
		# Start detect distance
		GPIO.setmode(GPIO.BOARD)
                GPIO.setup(self.trig_pin, GPIO.OUT)
                GPIO.setup(self.echo_pin, GPIO.IN)
                print("Start detect distance")
		self.ultra()

	def ultra(self):
		while not rospy.is_shutdown():
			data = self.get_distance()

			# Stop
			if data < 25:
				stop_msg = Int32()
				stop_msg.data = True
				self.pub_stop.publish(stop_msg)
				print("[ultrasound_node] Publish Successfully!")

			time.sleep(2)	
	
	def send_trigger_pulse(self):
		GPIO.output(self.trig_pin, True)
		time.sleep(0.001)
		GPIO.output(self.trig_pin, False)

	def wait_for_echo(self, value, timeout):
		count = timeout
		while GPIO.input(self.echo_pin) != value and count > 0:
			count = count -1
	
	def get_distance(self):
		self.send_trigger_pulse()
		self.wait_for_echo(True, 5000)
		start = time.time()
		self.wait_for_echo(False, 5000)
		finish = time.time()
		pulse_len = finish - start
		distance_cm = pulse_len * 340 * 100 / 2
		return distance_cm

if __name__ == "__main__":
	rospy.init_node("ultrasound", anonymous=False)
	ultrasound = Ultrasound()
	rospy.spin()
