#!/usr/bin/env python

import rospy
from gps_road_estimator import estimate_road

if __name__ == '__main__':
    try:

        road_estimator = estimate_road()
        road_estimator._get_path_points()
        road_estimator._subscribe_to_gps()

        rospy.spin()

    except Exception as e:
        rospy.loginfo(e)