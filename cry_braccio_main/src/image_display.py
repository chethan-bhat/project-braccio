#!/usr/bin/env python2
from collections import deque
import numpy as np
import argparse
import sys
import cv2
import imutils
import time
from cv_bridge import CvBridge

import rospy
from std_msgs.msg import Int32 
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import rospkg
from std_msgs.msg import Int16MultiArray

class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.frame = None
		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)
		#self.pts = deque(maxlen=args["buffer"])
		self.pub = rospy.Publisher('/bounding_box', Int16MultiArray, queue_size=1)
		rospy.Subscriber("/camera/color/image_raw",Image,self.image_callback)

	def image_callback(self,msg):
		self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		cv2.imshow("video_feed", self.frame)
		cv2.waitKey(3)

	def track_ball(self):
		if self.frame is None:
			print("no IMage")
		# resize the frame, blur it, and convert it to the HSV
		# color space
		#frame = imutils.resize(frame, width=600)
		self.frame = imutils.resize(self.frame, width=600)
		self.blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
		self.hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)
		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		self.mask = cv2.inRange(self.hsv, self.greenLower, self.greenUpper)
		self.mask = cv2.erode(self.mask, None, iterations=2)
		self.mask = cv2.dilate(self.mask, None, iterations=2)
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		self.cnts = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		self.center = None
		# only proceed if at least one contour was found
		if len(self.cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			self.c = max(cnts, key=cv2.contourArea)
			((self.x, self.y), self.radius) = cv2.minEnclosingCircle(self.c)
			self.M = cv2.moments(self.c)
			self.center = (int(self.M["m10"] / self.M["m00"]), int(self.M["m01"] / self.M["m00"]))
			# only proceed if the radius meets a minimum size
			if self.radius > 10:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(self.frame, (int(self.x), int(self.y)), int(self.radius),(0, 255, 0), 2)
				cv2.circle(self.frame, self.center, 5, (0, 0, 255), -1)
				cv2.imshow("video_feed", self.frame)
				cv2.waitKey(3)
				#x,y,w,h = cv2.boundingRect(c)	 
				#frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
				print(self.x,self.y)
				self.box_msg=Int16MultiArray()
	           		self.box_msg.data=[1,self.x,self.y,self.radius]
				self.pub.publish(self.box_msg)
		# update the points queue
		#self.pts.appendleft(self.center)



def main(args):
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		ic.track_ball()
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
