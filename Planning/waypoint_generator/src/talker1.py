#!/usr/bin/env python
import rospy
from waypoint_generator.msg import point_list
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
# import tf

drone_gps = Point()
msg0 = point_list()
i = 0


def gps_callback(data):
    global drone_gps
    global i
    drone_gps.x = data.latitude
    drone_gps.y = data.longitude

    # frame points
    point1 = Point()  # GPS
    point1.x = drone_gps.x
    point1.y = drone_gps.y
    point1.z = 1.57
    point2 = Point()  # pixel 1
    point2.x = 900
    point2.y = 540
    point3 = Point()  # pixel 2
    point3.x = 960
    point3.y = 500
    point4 = Point()  # pixel 3
    point4.x = 1000
    point4.y = 200
    point5 = Point()  # pixel 3
    point5.x = 1050
    point5.y = 750

    if i == 0:
        msg0.points = [point1, point2]
    # elif i == 1:
    #     msg0points = [point1]
    # elif i == 2:
    #     msg0points = [point1]
    else:
        msg0.points = [point1]

    i += 1
    i %= 4


def talker():
    # pub = rospy.Publisher('/drone1/some_topic', point_list, queue_size=10)
    # gps = rospy.Publisher('/drone1/mavros/global_position/global', NavSatFix, queue_size=10)
    # height = rospy.Publisher('/drone1/mavros/local_position/pose',  PoseStamped, queue_size=10)
    # compass_hdg = rospy.Publisher('/drone1/mavros/global_position/compass_hdg', Float64, queue_size=10)
    # pub0 = rospy.Publisher('/drone0/chatter', point_list, queue_size=10)
    pub1 = rospy.Publisher('/drone1/probable_target_locations', point_list, queue_size=10)
    gps_pos = rospy.Subscriber("/drone1/mavros/global_position/global", NavSatFix, gps_callback)
    # pub2 = rospy.Publisher('/drone2/chatter', point_list, queue_size=10)
    # pub3 = rospy.Publisher('/drone3/chatter', point_list, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(4)  # 4hz

    # msg1 = point_list()
    # point1.x = 0
    # point1.y = 0
    # point2.x = 0
    # point2.y = 0
    # point3.x = 0
    # point3.y = 0
    # point4.x = 0
    # point4.y = 0
    # msg1.points = [point1, point2, point3, point4]

    # msg2 = point_list()
    # point1.x = 0
    # point1.y = 0
    # point2.x = 0
    # point2.y = 0
    # point3.x = 0
    # point3.y = 0
    # point4.x = 0
    # point4.y = 0
    # msg2.points = [point1, point2, point3, point4]

    # msg3 = point_list()
    # point1.x = 0
    # point1.y = 0
    # point2.x = 0
    # point2.y = 0
    # point3.x = 0
    # point3.y = 0
    # point4.x = 0
    # point4.y = 0
    # msg3.points = [point1, point2, point3, point4]

    # GPS
    # navsat = NavSatFix()
    # navsat.latitude = 0.00
    # navsat.longitude = 0.00
    # navsat.altitude = 30.00

    # # heading_angle
    # heading_angle = Float64(0.0)

    # pose = PoseStamped()
    # pose.header.stamp = rospy.Time.now()
    # pose.header.frame_id = "map"
    # pose.pose.position.x = 10
    # pose.pose.position.y = 10
    # pose.pose.position.z = 10.0                 # only this used in subscriber

    # quaternion = tf.transformations.quaternion_from_euler(0, 0, 10)
    # pose.pose.orientation.x = quaternion[0]
    # pose.pose.orientation.y = quaternion[1]
    # pose.pose.orientation.z = quaternion[2]
    # pose.pose.orientation.w = quaternion[3]

    while not rospy.is_shutdown():
        # rospy.loginfo(msg)
        # gps.publish(navsat)
        # compass_hdg.publish(heading_angle)
        # height.publish(pose)
        pub1.publish(msg0)
        # pub0.publish(msg0)
        # pub2.publish(msg2)
        # pub3.publish(msg3)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
