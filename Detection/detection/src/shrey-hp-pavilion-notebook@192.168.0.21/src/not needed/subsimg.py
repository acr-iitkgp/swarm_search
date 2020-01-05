#!/usr/bin/env python

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import cv2 
import numpy as np 

import numpy as np
import argparse

import cv2

import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from detection.msg import point_list
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def point_publisher():
	
	################## CHANGE VIDEO FOR CAMERA Stream
	cap = cv2.VideoCapture("/home/archit/Documents/INter IIT/c.MOV")

	# Check if camera opened successfully
	if (cap.isOpened() == False): 
		print("Unable to read feed")

	# cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
	image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

	bridge = CvBridge()
 
	rospy.init_node('image', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		ret,frameorg = cap.read()
		frameorg = cv2.resize(frameorg, (frameorg.shape[1]//2, frameorg.shape[0]//2))
		cv2.imshow("frame",frameorg)

		image_pub.publish(bridge.cv2_to_imgmsg(frameorg, "bgr8"))
		rate.sleep()

point_publisher()
