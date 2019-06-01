#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32



class Ex1_subscribe(object):

    def __init__(self): 

        self.sub_password = rospy.Subscriber("~password", Int32, self.cbPassword, queue_size=1)

        

    def cbPassword(self, password_msg):

        password_msg = password_msg

        print("[ex1_subscribe_node] Successfully open the security box!")

        print("[ex1_subscribe_node] Your password is: %d" % password_msg.data)



if __name__ == "__main__":

    rospy.init_node("ex1_subscribe",anonymous=False)

    ex1_subscribe = Ex1_subscribe()

    rospy.spin()


