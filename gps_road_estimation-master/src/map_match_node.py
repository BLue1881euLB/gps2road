#!/usr/bin/env python

# ros files
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

# my classes
from path import path
from goal import goal
from map_match import map_match

# shapely files
from shapely.geometry import Point

# global instances
map_m = None
goal_m = None

# ros publishers
odometry_pp_pub = rospy.Publisher(rospy.get_param("/road_estimation/published_topic"), Odometry, queue_size=1) # projected point publisher
goal_status_pub = rospy.Publisher(rospy.get_param("/road_estimation/goal_topic"), GoalStatusArray, queue_size=1) # goal status publisher

def get_path_points():
	rospy.loginfo("Waiting for the path msg ...")

	while True:
		try:
			path = rospy.wait_for_message(rospy.get_param("/road_estimation/path_topic"), Path)
			break
		except Exception as e:
			rospy.loginfo("The following Exception is raised while getting path msg" + e + " ... will try again.")

	path_points = []
	for point in path.poses:
		path_point = (point.pose.position.x, point.pose.position.y)
		path_points.append(path_point)

	# rospy.loginfo(path_points)
	return path_points

def odometry_callback(odom):
	global map_m
	global goal_m
	global odometry_pp_pub
	global goal_status_pub

	odom_point = Point(odom.pose.pose.position.x, 
		               odom.pose.pose.position.y)
	
	matched_point, matched_edge = map_m.get_projected_p(odom_point)
	goal_arr = goal_m.get_goal_status(matched_point, matched_edge)

	odom.pose.pose.position.x = matched_point.x
	odom.pose.pose.position.y = matched_point.y

	odometry_pp_pub.publish(odom)
	goal_status_pub.publish(goal_arr)

	# rospy.loginfo(matched_edge)

	if goal_m.final_goal_is_reached() == True:
		rospy.loginfo("reached final goal ...")
		sub.unregister()

if __name__ == '__main__':
	try:
		rospy.init_node('road_estimation')
		path_points = get_path_points() # path points list 
		
		my_path = path(path_points)
		map_m = map_match(my_path)
		goal_m = goal(path_points)

		sub = rospy.Subscriber(rospy.get_param("/road_estimation/odometry_topic"), Odometry, odometry_callback)
		rospy.loginfo("subscribed ...")
	    
		rospy.spin()
	
	except Exception as e:
		rospy.loginfo(e)
