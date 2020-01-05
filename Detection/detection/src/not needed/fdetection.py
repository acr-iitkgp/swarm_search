#!/usr/bin/env python

import cv2
import numpy as np 

from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from detection.msg import point_list
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def avg_col(img):
	rows = img.shape[0]
	cols = img.shape[1]
	colour = np.int32([np.sum(img[:, :, 0]), np.sum(img[:, :, 1]), np.sum(img[:, :, 2])])/(rows*cols)
	return colour 

def avg_col_bw(img):
	rows = img.shape[0]
	cols = img.shape[1]
	colour = np.int32(np.sum(img[:, :]))/(rows*cols)
	return colour 

def avg_col_removed(img, img0):
	rows = img.shape[0]
	cols = img.shape[1]
	rows0 = img0.shape[0]
	cols0 = img0.shape[1]
	colour = (np.int32([np.sum(img[:, :, 0]), np.sum(img[:, :, 1]), np.sum(img[:, :, 2])])-np.int32([np.sum(img0[:, :, 0]), np.sum(img0[:, :, 1]), np.sum(img0[:, :, 2])]))/((rows*cols)-(rows0*cols0))
	return colour 

def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
 
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)

def all_preprocess(frame):
	image = cv_image[:1000, :, :]
	image = adjust_gamma(image, 1)
	# DID resize
	image = cv2.resize(image, (image.shape[1]//2, image.shape[0]//2))
	
	threshold= cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (25, 0, 0), (100, 255, 255));
	cv2.imshow("inRange", threshold)

	cv2.imshow("frame",image)
	blur = (cv2.GaussianBlur(image, (5,5), 0))
	
	#grayg = image[:, :, 1]
	# blurs = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
	blurs = blur[:, :, 1]
	# blur = grayg
	
	# blurred = blur[:, :, 1]
	#blurred = blur
	# blurred = cv2.equalizeHist(blurred)
	# cv2.imshow("blurgray", blurs)
	ret,thresh = cv2.threshold(blurs,150,255,0)
	thresh = cv2.erode(thresh, None, iterations=6)
	thresh = cv2.dilate(thresh, None, iterations=6)
	cv2.imshow("thresherodedilate", thresh)
	cv2.imshow("thresh", thresh)

	imgx = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	imgy = cv2.cvtColor(imgx, cv2.COLOR_BGR2GRAY)

	ret, threshhsv = cv2.threshold(imgy,100,255,0)
	cv2.imshow("invthresh", threshhsv)

	invcontours, invhierarchy = cv2.findContours((threshhsv),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# cnts = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	# contours = imutils.grab_contours(contours)
	cnts = []
	for cnt in contours:
		k = cv2.isContourConvex(cnt)
		# if not k:
		# 	continue
		area = cv2.contourArea(cnt)
		if area<80:
			continue

		epsilon = 0.05*cv2.arcLength(cnt,True)
		approx = cv2.approxPolyDP(cnt,epsilon,True)
		# if len(approx) > 4:
		# 	continue
		
		hull = cv2.convexHull(cnt)
		hullarea = cv2.contourArea(hull)
		if hullarea != 0:
			# print(area/hullarea)
			if area/hullarea<0.7:
				continue

		rect = cv2.minAreaRect(cnt)
		rectarea = cv2.contourArea(hull)
		# print(rectarea/area)
		# if rectarea/area>1.10:
		# 	continue

		cnts.append(cnt)

	# for cnt in invcontours:
	# 	k = cv2.isContourConvex(cnt)
	# 	# if not k:
	# 	# 	continue
	# 	area = cv2.contourArea(cnt)
	# 	if area<50:
	# 		continue


	# 	epsilon = 0.05*cv2.arcLength(cnt,True)
	# 	approx = cv2.approxPolyDP(cnt,epsilon,True)
	# 	# if len(approx) > 4:
	# 	# 	continue
		
	# 	hull = cv2.convexHull(cnt)
	# 	hullarea = cv2.contourArea(hull)
	# 	# print(area/hullarea)
	# 	if area/hullarea<0.7:
	# 		continue
	# 	rect = cv2.minAreaRect(cnt)
	# 	rectarea = cv2.contourArea(hull)
	# 	# print(rectarea/area)

	# 	cnts.append(cnt)

	cnts1 = []
	for cnt in cnts:
		(x, y, w, h) = cv2.boundingRect(cnt)

		cntimg = blur[y:y+h, x:x+w, :]
		cnt_extnd_wnd = blur[max(0, y-10):min(y+h+10, image.shape[0]), max(0, x-10):min(x+w+10, image.shape[1]), :]
		c1 = avg_col(cntimg)
		c0 = avg_col_removed(cnt_extnd_wnd, cntimg)
		diffs = (np.mean(np.abs(c1 - c0)))
		if diffs<20:
			# print("GRASS")
			continue
		threshimg = threshold[y:y+h, x:x+w]
		threscol = avg_col_bw(threshimg)
		print(threscol)
		if threscol<100:
			continue
		cnts1.append(cnt)

	return cnts1


class image_converter:

  def __init__(self):
	# self.image_pub = rospy.Publisher("image_topic_2",Image)
	
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

	pub = rospy.Publisher('roi_coordinates', point_list)
	rate = rospy.Rate(10) # 10hz
	msg = point_list()

  def callback(self,data):
	try:
	  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	  print(e)

	print("dbaskdj")
	cntlist = all_preprocess(cv_image)

	for cnts in cntlist:
		
		[x, y, w, h] = cnts
		point1 = Point()
		point1.x = x
		point1.y = y
		point1.z = 0
		point2 = Point()
		point2.x = x+w
		point2.y = y
		point2.z = 0
		point3 = Point()
		point3.x = x
		point3.y = y+h
		point3.z = 0
		point4 = Point()
		point4.x = x+w
		point4.y = y+h
		point4.z = 0
		msg.points = [point1, point2, point3, point4]

		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

	cv2.drawContours(image,cnts1,-1,(0,0,255),3)
	cv2.imshow("contours", image)
	cv2.waitKey(1)

	# try:
	#   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	# except CvBridgeError as e:
	#   print(e)



def main():

  rospy.init_node('detect_roi', anonymous=True)
  ic = image_converter()
 #  try:
	# rospy.spin()
 #  except KeyboardInterrupt:
	# print("Shutting down")
  cv2.destroyAllWindows()

main()