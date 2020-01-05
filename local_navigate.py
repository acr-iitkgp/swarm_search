#!/usr/bin/env python

import rospy
import mavros
from math import *
import thread
import threading
import time
from mavros.utils import *
from mavros import setpoint
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *


class SetpointPosition:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # publisher for mavros/setpoint_position/local
        self.pub = setpoint.get_pub_position_local(queue_size=10)  # warum 10?
        # subsrciber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), setpoint.PoseStamped, self.reached)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(10)

        msg = setpoint.PoseStamped(
            header=setpoint.Header(
                frame_id="map",  # isn't used anyway
                stamp=rospy.Time.now()),
        )

        while not rospy.is_shutdown():
            #msg.pose.position.x = self.y
            #msg.pose.position.y = self.x
            #msg.pose.position.z = - self.z

            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            self.pub.publish(msg)
            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            print x, y
            print abs(x - y)
            return abs(x - y) < 0.5
        # print topic.pose.position.x
        if is_near('X', topic.pose.position.x, self.x) and is_near('Y', topic.pose.position.y, self.y) and is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()
            rospy.loginfo("setpoint reached!")


def setpoint_demo():
    rospy.init_node('setpoint_position')
    mavros.set_namespace()  # initialize mavros module with default namespace, warum ist das noetig?
    rate = rospy.Rate(10)

    setpoint = SetpointPosition()

    rospy.loginfo("move forward")
    setpoint.set(2, 1.0, 2, 2)
    setpoint.set(2, 3.5, 2, 2)
    setpoint.set(2, 5.5, 2, 2)  # desired position in ENU
    #setpoint.set(2, 2.5, 2, 2)

    rospy.loginfo("Bye!")


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass
