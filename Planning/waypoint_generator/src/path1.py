#!/usr/bin/env python
import AHP
import GOW
import math
import rospy
from geometry_msgs.msg import Point
from waypoint_generator.msg import point_list
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from geopy.distance import geodesic
from swarm_search.msg import local_flags
from time import sleep
from std_msgs.msg import Float64MultiArray

pos_list = []

global pub1
global current_position
current_position = Point()
flag = local_flags()
flag.transition_s2s.data = False
GPS_flag = 0


def frameCallback(data):
    global pos_list
    pos_list = data.points
    if flag.transition_s2s.data == True and GPS_flag == 1:
        path()


def current_callback(data):
    global GPS_flag
    global current_position
    current_position.x = data.latitude
    current_position.y = data.longitude
    GPS_flag = 1


def flag_callback(data):
    global flag
    flag = data


def listener():
	global pub1
	rospy.init_node('path_drone1', anonymous=True)
	frame_sub = rospy.Subscriber('/drone1/ROI_flow_initial', point_list, frameCallback)
	current_pos = rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, current_callback)
	flags = rospy.Subscriber("/drone1/flags", local_flags, flag_callback)
	#pub1 = rospy.Publisher('/listener', Float64MultiArray, queue_size=10)
	pub1 = rospy.Publisher('/drone1/ROI_flow', point_list, queue_size=5)
	rospy.spin()


def path():
    print("Entered Path()")
    global pub1
    global pos_list
    current_pos = current_position
    edge = []

    print(pos_list)
    for i in range(len(pos_list)):
        for j in range(i + 1, len(pos_list)):
            edge.append(AHP.Edge(i, j, geodesic((pos_list[i].x, pos_list[i].y), (pos_list[j].x, pos_list[j].y)).m))

    for e in edge:
        pass
        # print(e.u, "<->", e.v, ", W:", e.w)

    path, w = AHP.AHP(edge, len(pos_list))

    order_waypt = GOW.generateOrderedWaypoints(pos_list, current_pos, path)
    while not rospy.is_shutdown():
		pub1.publish(order_waypt)
		#pub1.publish(Float64MultiArray(data=order_waypt))
		sleep(1)

if __name__ == '__main__':
    listener()
