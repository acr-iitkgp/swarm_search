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


pos_list = []

global pub1
global current_position
current_position = Point()
global flag
flag = Twist()
flag.linear.y = 0
#flags.limnear.y


def posCallback(data):
    global pos_list
    pos_list = data.points
    if flag.linear.y == 1:
        path()
#	print (pos_list)

def current_callback(data):
    global current_position
    current_position.x = data.latitude
    current_position.y = data.longitude

def flag_callback(data):
    global flag 
    flag = data

def listener():
    global pub1
    rospy.init_node('path2', anonymous=True)
    pos_sub = rospy.Subscriber('/drone2/chatter', point_list, posCallback)
    current_pos = rospy.Subscriber("/drone2/mavros/global_position/global",NavSatFix, current_callback)
    flags = rospy.Subscriber("/drone2/flags",Twist,flag_callback)
    pub1 = rospy.Publisher('/drone2/ROI_flow', point_list, queue_size=5)
    rospy.spin()


def path():
    global pub1
    global pos_list
    current_pos = current_position
    # pos_list = pos_list
    edge = []

    for i in range(len(pos_list)):
        for j in range(i + 1, len(pos_list)):
            edge.append(AHP.Edge(i, j, geodesic((pos_list[i].x, pos_list[i].y), (pos_list[j].x,pos_list[j].y)).m ))

    for e in edge:
        pass
        # print(e.u, "<->", e.v, ", W:", e.w)

    path, w = AHP.AHP(edge, len(pos_list))
    # print("Final result", path, "TW:", w)

    order_waypt = GOW.generateOrderedWaypoints(pos_list, current_pos, path)
    pub1.publish(order_waypt)
    # print(order_waypt)


if __name__ == '__main__':
    listener()
