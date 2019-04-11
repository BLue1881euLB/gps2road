#!/usr/bin/env python

import rospy
import roslib
import numpy as np
from nav_msgs.msg import Odometry

pub = rospy.Publisher(rospy.get_param("pub_topic"), Odometry, queue_size=1)

def odom_callback(message):
	global pub

	message.header.frame_id = rospy.get_param("frame_id")
	message.child_frame_id = rospy.get_param("child_frame_id")

	pub.publish(message)

if __name__ == '__main__':

    rospy.init_node('gps_frame_fix')
    sub = rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()