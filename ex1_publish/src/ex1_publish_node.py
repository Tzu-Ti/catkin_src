#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32



class Ex1_publish(object):

    def __init__(self):

        self.pub_password = rospy.Publisher("~password", Int32, queue_size=1)

        self.securityBox()



    def securityBox(self):

        while(True):

            key = input("Enter password: ")

            if key == 666:  # if the password is correct, then publish message to another node

                override_msg = Int32()

                override_msg.data = key

                self.pub_password.publish(override_msg)

                print("[ex1_publish_node] Publish successfully!")

                break

            else:  # if the password is wrong, then ask user for re-enter

                print("[ex1_publish_node] Wrong password! Try to enter your password again.")



if __name__ == "__main__":

    rospy.init_node("ex1_publish",anonymous=False)

    ex1_publish = Ex1_publish()

    rospy.spin()


