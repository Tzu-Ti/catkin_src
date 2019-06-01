#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

class Speech_recognition(object):
	def __init__(self):
		self.sub_activity = rospy.Subscriber("/speech_recognition/activity", Int32, self.speech_control, queue_size=1)

	def speech_control(self, activity_msg):
		activity_msg = activity_msg
		activity = activity_msg.data
		print("[Speech_recognition] %s" %activity)

if __name__ == "__main__":
	rospy.init_node("speech_recognition", anonymous=False)
	speech_recognition = Speech_recognition()
	rospy.spin() 
