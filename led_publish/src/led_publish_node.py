#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

class Led_publish(object):
        def __init__(self):
                self.pub_data = rospy.Publisher("~data", Int32, queue_size=1)
                self.send_data()
        def send_data(self):
		while not rospy.is_shutdown():
                	while True:
				num = input("Enter data: ")
				override_msg = Int32()
                        	override_msg.data = num
                        	self.pub_data.publish(override_msg)
                        	print("[led_publish_node] Publish successfully!")

if __name__ == "__main__":
        rospy.init_node("led_publish", anonymous=False)
        led_publish = Led_publish()
        rospy.spin()
