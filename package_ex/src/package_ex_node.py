#!/usr/bin/env python

import rospy

class Package_ex(object):

    def __init__(self):

        self.printInitializeMessage()



    def printInitializeMessage(self):

        print("[Package_ex] Initialize successfully!")

    

if __name__ == "__main__":

    rospy.init_node("package_ex",anonymous=False)

    package_ex = Package_ex()

    rospy.spin()


