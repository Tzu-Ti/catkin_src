#!/usr/bin/env python
import cv2
import picamera
import numpy as np
import time
import rospy
from std_msgs.msg import Int32

class Obstacle_detection(object):
        def __init__(self):
                self.pub_stop = rospy.Publisher("~stop", Int32, queue_size=1)
        	self.main()
	def canny(self, image):
                # Gray
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                # Gaussian Blur
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                # Edge Detection
                canny = cv2.Canny(blur, 50, 150)
                return canny

        def region_of_interest_top(self, image):
                height = image.shape[0]
                weight = image.shape[1]
                polygons = np.array([
                [(0, height), (weight, height), (weight, height/2), (0, height/2)]
                ])
                mask = np.zeros_like(image)
                cv2.fillPoly(mask, polygons, 255)
                masked_image = cv2.bitwise_and(image, mask)
                return masked_image

        def region_of_interest_down(self, image):
                height = image.shape[0]
                weight = image.shape[1]
                polygons = np.array([
                [(0, 0), (weight, 0), (weight, 3*height/4), (0, 3*height/4)]
                ])
                mask = np.zeros_like(image)
                cv2.fillPoly(mask, polygons, 255)
                masked_image = cv2.bitwise_and(image, mask)
                return masked_image

        def average_slope_intercept(self, lines):
                left_fit = []
                right_fit = []
                left_line = None
                right_line = None
                for line in lines:
                        x1, y1, x2, y2 = line.reshape(4)
                        parameters = np.polyfit((x1, x2), (y1, y2), 1)
                        slope = parameters[0]
                        intercept = parameters[1]
                        if slope > 0.1 and slope < 3:
                                left_fit.append((slope, intercept))
                        elif slope < -0.1 and slope > -3:
                                right_fit.append((slope, intercept))
                #print(left_fit, " --- ", right_fit)
                if len(left_fit) != 0:
                        left_fit_average = np.average(left_fit, axis=0)
                        left_line = self.make_coordinates(left_fit_average)
                if len(right_fit) != 0:
                        right_fit_average = np.average(right_fit, axis=0)
                        right_line = self.make_coordinates(right_fit_average)
                if left_line is not None and right_line is not None:
                        self.send_data()
			time.sleep(2)
                return np.array([left_line, right_line])

	def send_data(self):
		stop_msg = Int32()
                stop_msg.data = True
                self.pub_stop.publish(stop_msg)


        def make_coordinates(self, line_parameters):
                slope, intercept = line_parameters
                y1 = 132
                y2 = 145
                x1 = int((y1 - intercept) / slope)
                x2 = int((y2 - intercept) / slope)
                return np.array([x1, y1, x2, y2])

        def display_lines(self, image, lines):
                line_image = np.zeros_like(image)
                if lines is not None:
                        for line in lines:
                                if line is not None:
                                        x1, y1, x2, y2 = line.reshape(4)
                                        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 10)
                return line_image

	        

	def main(self):
                with picamera.PiCamera() as camera:
                        camera.resolution = (320, 240)
                        camera.framerate = 24
                        time.sleep(2)

                        while not rospy.is_shutdown():
                                image = np.empty((240 * 320 *3,), dtype=np.uint8)
                                camera.capture(image, format='bgr')
                                image = image.reshape((240, 320, 3))
                                image_cp = image.copy()

                                edges = self.canny(image)
                                cropped_top_image = self.region_of_interest_top(edges)
                                cropped_down_image = self.region_of_interest_down(cropped_top_image)

                                lines = cv2.HoughLinesP(cropped_down_image, 1, np.pi/180, 100, np.array([]), minLineLength=20, maxLineGap=5)
                                if lines is not None:
                                        averaged_lines = self.average_slope_intercept(lines)
                                        line_image = self.display_lines(image_cp, averaged_lines)
if __name__ == "__main__":
        rospy.init_node("obstacle_detection", anonymous=False)
        obstacle_detection = Obstacle_detection()
        rospy.spin()

