#!/usr/bin/env python
import rospy
from waypoint_generator.msg import point_list
from geometry_msgs.msg import Point

ROI_list_0 = []
ROI_list_1 = []
ROI_list_2 = []
ROI_list_3 = []

ROI_list_0_new = point_list()
ROI_list_1_new = point_list()
ROI_list_2_new = point_list()
ROI_list_3_new = point_list()

flag = [0,0,0,0]

ROI_list_pub_0 = rospy.Publisher('/drone0/ROI_flow', point_list, queue_size=5)
ROI_list_pub_1 = rospy.Publisher('/drone1/ROI_flow', point_list, queue_size=5)
ROI_list_pub_2 = rospy.Publisher('/drone2/ROI_flow', point_list, queue_size=5)
ROI_list_pub_3 = rospy.Publisher('/drone3/ROI_flow', point_list, queue_size=5)

def ROICallback0(data):
	global ROI_list_0
	ROI_list_0 = data.points
	flag[0] = 1
	synchroniser()

def ROICallback1(data):
	global ROI_list_1
	ROI_list_1 = data.points
	flag[1] = 1
	synchroniser()

def ROICallback2(data):
	global ROI_list_2
	ROI_list_2 = data.points
	flag[2] = 1
	synchroniser()

def ROICallback3(data):
	global ROI_list_3
	ROI_list_3 = data.points
	flag[3] = 1
	synchroniser()

def synchroniser():
	global ROI_list_pub_0
	global ROI_list_pub_1
	global ROI_list_pub_2
	global ROI_list_pub_3
	check = 1
	scale = 0.00001
	for i in range(4):
		check = check*flag[i]
	if check:
		#drone0
		for i in range(len(ROI_list_0)):
			
			for j in range (len(ROI_list_1)):	
				if abs(ROI_list_0[i].x-ROI_list_1[j].x)<0.00001 and abs(ROI_list_0[i].y-ROI_list_1[j].y)<0.00001:
					if len(ROI_list_0)<len(ROI_list_1):
						ROI_list_0_new.points.append(ROI_list_0[i])
					else:
						ROI_list_1_new.points.append(ROI_list_1[j])

				else:
					ROI_list_0_new.points.append(ROI_list_0[i])
					ROI_list_1_new.points.append(ROI_list_1[j])

			for j in range (len(ROI_list_2)):	
				if abs(ROI_list_0[i].x-ROI_list_2[j].x)<0.00001 and abs(ROI_list_0[i].y-ROI_list_2[j].y)<0.00001:
					if len(ROI_list_0)<len(ROI_list_2):
						ROI_list_0_new.points.append(ROI_list_0[i])
					else:
						ROI_list_2_new.points.append(ROI_list_2[j])

				else:
					ROI_list_0_new.points.append(ROI_list_0[i])
					ROI_list_2_new.points.append(ROI_list_2[j])

			for j in range (len(ROI_list_3)):	
				if abs(ROI_list_0[i].x-ROI_list_3[j].x)<0.00001 and abs(ROI_list_0[i].y-ROI_list_3[j].y)<0.00001:
					if len(ROI_list_0)<len(ROI_list_3):
						ROI_list_0_new.points.append(ROI_list_0[i])
					else:
						ROI_list_3_new.points.append(ROI_list_3[j])

				else:
					ROI_list_0_new.points.append(ROI_list_0[i])
					ROI_list_3_new.points.append(ROI_list_3[j])

		#drone1
		for i in range(len(ROI_list_1)):

			for j in range (len(ROI_list_2)):	
				if abs(ROI_list_1[i].x-ROI_list_2[j].x)<0.00001 and abs(ROI_list_1[i].y-ROI_list_2[j].y)<0.00001:
					if len(ROI_list_1)<len(ROI_list_2):
						ROI_list_1_new.points.append(ROI_list_1[i])
					else:
						ROI_list_2_new.points.append(ROI_list_2[j])

				else:
					ROI_list_1_new.points.append(ROI_list_1[i])
					ROI_list_2_new.points.append(ROI_list_2[j])

			for j in range (len(ROI_list_3)):	
				if abs(ROI_list_1[i].x-ROI_list_3[j].x)<0.00001 and abs(ROI_list_1[i].y-ROI_list_3[j].y)<0.00001:
					if len(ROI_list_1)<len(ROI_list_3):
						ROI_list_1_new.points.append(ROI_list_1[i])
					else:
						ROI_list_3_new.points.append(ROI_list_3[j])

				else:
					ROI_list_1_new.points.append(ROI_list_1[i])
					ROI_list_3_new.points.append(ROI_list_3[j])

		#drone2
		for i in range(len(ROI_list_2)):

			for j in range (len(ROI_list_3)):	
				if abs(ROI_list_2[i].x-ROI_list_3[j].x)<0.00001 and abs(ROI_list_2[i].y-ROI_list_3[j].y)<0.00001:
					if len(ROI_list_2)<len(ROI_list_3):
						ROI_list_2_new.points.append(ROI_list_2[i])
					else:
						ROI_list_3_new.points.append(ROI_list_3[j])

				else:
					ROI_list_2_new.points.append(ROI_list_2[i])
					ROI_list_3_new.points.append(ROI_list_3[j])

	ROI_list_pub_0.publish(ROI_list_0_new)
	ROI_list_pub_1.publish(ROI_list_1_new)
	ROI_list_pub_2.publish(ROI_list_2_new)
	ROI_list_pub_3.publish(ROI_list_3_new)


def main():
	rospy.init_node('synchroniser', anonymous=True)
	ROI_list_sub_0 = rospy.Subscriber('/drone0/chatter', point_list, ROICallback0)
	ROI_list_sub_1 = rospy.Subscriber('/drone1/chatter', point_list, ROICallback1)
	ROI_list_sub_2 = rospy.Subscriber('/drone2/chatter', point_list, ROICallback2)
	ROI_list_sub_3 = rospy.Subscriber('/drone3/chatter', point_list, ROICallback3)

	rospy.spin()

if __name__=="__main__":
	main()