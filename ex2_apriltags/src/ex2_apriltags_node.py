#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import AprilTagsWithInfos
from std_msgs.msg import Int32
import time

class ex2_apriltags_node(object):
    def __init__(self):

        self.sub_tag_id = rospy.Subscriber("apriltags_postprocessing_node/apriltags_out", AprilTagsWithInfos, self.get_Apriltag, queue_size=1)
	self.pub_reach = rospy.Subscriber("~id", Int32, queue_size=1)

    def get_Apriltag(self, Tag):
        try:
            tag_id = Tag.infos[0].id
            rospy.loginfo("-------TAG = %d---------" %(tag_id))
	    send_data(True)
	    time.sleep(2)
        except:
            print("NO TAG")

    def send_data(self, reach):
	reach_msg = Int32()
        reach_msg.data = reach
        self.pub_reach.publish(reach_msg)

if __name__ == "__main__":
    rospy.init_node("ex2_apriltags_node", anonymous=False)
    ex2_apriltags = ex2_apriltags_node()
    rospy.spin()
