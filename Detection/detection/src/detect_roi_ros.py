#!/usr/bin/env python
import sys
# print(sys.path)

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Point
from swarm_search.msg import point_list, local_flags
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

M_PI = 3.14156

sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2
import numpy as np

import numpy as np
import argparse

import cv2
font = cv2.FONT_HERSHEY_COMPLEX
import roi_detect
import close_detect

current_flag = 0
state = []

current_pose = NavSatFix()

heading = Float64()


def callback(data):
    global state, current_flag
    if data.scan_flag.data == True:
        new_flag = 1
    elif data.search_flag.data == True:
        new_flag = 2
    else:
        new_flag = 0

    if new_flag != current_flag:
        state = []
    current_flag = new_flag


def global_position(data):
    global current_pose
    current_pose = data


def heading_feed(data):
    global heading
    heading = data.data * M_PI / 180.0


def point_publisher():
    global state, current_flag

    # CHANGE VIDEO FOR CAMERA Stream
    cap = cv2.VideoCapture(0)

    if (cap.isOpened() == False):
        print("Unable to read feed")
    rospy.init_node('detect_ros', anonymous=True)

    pub = rospy.Publisher('/drone1/probable_target_locations', point_list, queue_size=10)
    pubc = rospy.Publisher('/drone1/close_coordinates', point_list, queue_size=10)
    rospy.Subscriber("/drone1/flags", local_flags, callback)
    rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, global_position)
    rospy.Subscriber("/drone1/mavros/global_position/compass_hdg", Float64, heading_feed)

    rate = rospy.Rate(60)
    msg = point_list()
    ind = 0
    point_global = Point()
    while not rospy.is_shutdown():
#        print(frameorg.shape)
        if current_flag == 0:
            rate.sleep()
            continue
        ret, frameorg = cap.read()
        if not ret:
            print("chudd gyi aur laude lg gye")
        contourlist = []
        ind+=1
        #cv2.imshow("init", frameorg)
        if current_flag == 1:
            print("now detecting for ROI")
            contourlist, state = roi_detect.cntsearch(frameorg, state)
        else:
            print("doing the close detect")
            contourlist, state = close_detect.cntsearch(frameorg, state)
        point_global.x = current_pose.latitude
        point_global.y = current_pose.longitude
        point_global.z = heading

        msg.points.append(point_global)
        for cnts in contourlist:
            [x, y, w, h] = cnts
            cv2.rectangle(frameorg, (x, y), (x + w, y + h), (0, 255, 0), 10)

            point1 = Point()
            point1.x = x + w / 2.0
            point1.y = y + h / 2.0
            point1.z = 0
            msg.points.append(point1)
        # cv2.imshow("f", frameorg)
        cv2.waitKey(1)
        if current_flag == 1:
            pub.publish(msg)
        else:
            pubc.publish(msg)
        #if ind%20==0:
#            cv2.imshow("f", frameorg)
        cv2.imwrite("ardupilot_ws/src/UAV_Fleet_Challenge/Detection/detection/src/output"+str(ind)+".jpg", frameorg)

        #cv2.imshow("f", frameorg)
        msg = point_list()
        rate.sleep()

# first coordinate is the gps coordinate {x, y, z}   {lat, long, heading_angle}
point_publisher()
