#!/usr/bin/env python
import rospy
from waypoint_generator.msg import point_list
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
#import tf


def talker():
    # pub = rospy.Publisher('/drone1/some_topic', point_list, queue_size=10)
    # gps = rospy.Publisher('/drone1/mavros/global_position/global', NavSatFix, queue_size=10)
    # height = rospy.Publisher('/drone1/mavros/local_position/pose',  PoseStamped, queue_size=10)
    # compass_hdg = rospy.Publisher('/drone1/mavros/global_position/compass_hdg', Float64, queue_size=10)
    pub0 = rospy.Publisher('/drone0/ROI_flow_initial', point_list, queue_size=10)
    pub1 = rospy.Publisher('/drone1/ROI_flow_initial', point_list, queue_size=10)
    pub2 = rospy.Publisher('/drone2/ROI_flow_initial', point_list, queue_size=10)
    pub3 = rospy.Publisher('/drone3/ROI_flow_initial', point_list, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10)  # 10hz

    # frame points
    msg0 = point_list()
    point1 = Point()  # bottom right
    point1.x = -35.363359
    point1.y = 149.164596
    point2 = Point()  # top left
    point2.x = -35.363128
    point2.y = 149.164748
    point3 = Point()  # top right
    point3.x = -35.363377
    point3.y = 149.164678
    point4 = Point()  # pi
    point4.x = 0
    point4.y = 0
    msg0.points = [point1, point2, point3, point4]

    msg1 = point_list()
    point1.x = 22.3201176
    point1.y = 87.3013138
    point2.x = 22.3201192
    point2.y = 87.301365
    point3.x = 22.3200485
    point3.y = 87.3013427
    point4.x = 0
    point4.y = 0
    msg1.points = [point1, point2, point3, point4]

    msg2 = point_list()
    point1.x = 0
    point1.y = 0
    point2.x = 0
    point2.y = 0
    point3.x = 0
    point3.y = 0
    point4.x = 0
    point4.y = 0
    msg2.points = [point1, point2, point3, point4]

    msg3 = point_list()
    point1.x = 0
    point1.y = 0
    point2.x = 0
    point2.y = 0
    point3.x = 0
    point3.y = 0
    point4.x = 0
    point4.y = 0
    msg3.points = [point1, point2, point3, point4]

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
        # pub0.publish(msg0)
        msg1 = point_list()
        point1.x = 22.3201176
        point1.y = 87.3013138
        point2.x = 22.3201192
        point2.y = 87.301365
        point3.x = 22.3200485
        point3.y = 87.3013427
        point4.x = 0
        point4.y = 0
        msg1.points = [point1, point2, point3, point4]
        print(msg1)
        pub1.publish(msg1)
        # pub2.publish(msg2)
        # pub3.publish(msg3)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
