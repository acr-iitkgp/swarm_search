#!/usr/bin/env python
import sys
print(sys.path)

#from __future__ import (absolute_import, division, print_function, unicode_literals)
import rospy
# from std_msgs.msg import String
from detection.msg import point_list, local_flags


def point_publisher():

    rospy.init_node('msg_Test', anonymous=True)

    pub = rospy.Publisher('/drone0/flag', local_flags)
    local_flag = local_flags()
    local_flag.search_flag = True
    local_flag.scan_flag = False

	
    pub.publish(local_flag)
    rospy.spin()
point_publisher()
