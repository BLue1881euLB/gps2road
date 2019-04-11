import utm
import math
import rospy
import geopandas
import pandas as pd
import matplotlib.pyplot as plt

from shapely.geometry import Point
from shapely.geometry import LineString

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from road_processing_planning.srv import getPath

class estimate_road:
	def __init__(self):
		self._path_points = [] # used to store the coordinates of way points on the path
		self._matched_points = [] # used to store the projected gps points on the path
		self._prev_matched_edge = None # attribute used for comparison between path segments
		self._max_segment_length = 0.0 # attribute used to store the length of the longest path segment

		self._projected_points_publisher = rospy.Publisher('gps_projected_points', PointStamped, queue_size=1)
		self._projected_point = PointStamped()

		rospy.init_node('gps_road_estimator', anonymous=True) # initialize ros node

	# gps callback function
	def _gps_callback(self, gps):
		if (math.isnan(gps.latitude) == False) or (math.isnan(gps.longitude) == False):
			gps_point = self._gps_to_utm(gps.latitude, gps.longitude) # convert lat and long to utm
			matched_point, matched_edge = self._map_match(gps_point) # match gps points to path
			self._update_pp_msg(gps.header.stamp, matched_point.x, matched_point.y) # update projected point msg
			self._projected_points_publisher.publish(self._projected_point) # publish projected point

			# self._matched_points.append(matched_point) # add received projected point to list

			if self._check_waypoint(matched_edge) == True:
				rospy.loginfo("way point checked")

			distance_to_next_wp = self._distance_to_next_waypoint(matched_edge, matched_point)
			rospy.loginfo("distance to next way point is %f", distance_to_next_wp)

			self._update_matched_edge(matched_edge)
		else:
			rospy.loginfo("gps NaN detected")

	# matches gps points to path
	def _map_match(self, gps_point):
		while True:
			circle = gps_point.buffer(self._max_segment_length)
			possible_matches_index = list(self._nodes_spatial_index.intersection((circle.bounds)))
			possible_matches = self._nodes_gdf.iloc[possible_matches_index]
			precise_matches = possible_matches[possible_matches.intersects(circle)]
			candidate_nodes = list(precise_matches.index)

			if len(candidate_nodes) != 0:
				break
			else:
				rospy.loginfo("cannot find any near nodes, increasing search circle")
				self._max_segment_length += 100

		candidate_edges = []
		for node_id in candidate_nodes:
			# first node
			if node_id == 0:
				point_tuple_out = (node_id, node_id+1)
				candidate_edges.append(point_tuple_out)
			# last node
			elif node_id == len(self._nodes_gdf)-1:
			    point_tuple_in = (node_id-1, node_id)
			    candidate_edges.append(point_tuple_in)
			# any other node
			else:
			    point_tuple_in = (node_id-1, node_id)
			    point_tuple_out = (node_id, node_id+1)
			    candidate_edges.append(point_tuple_in)
			    candidate_edges.append(point_tuple_out)

		distance = []
		length = []
		for edge in candidate_edges:
			line_string = self._edges_gdf[(self._edges_gdf.u == edge[0]) & (self._edges_gdf.v == edge[1])].geometry
			distance.append([line_string.distance(gps_point), edge, line_string])
			
			d = line_string.distance(gps_point)
			length.append(d.iloc[0])

		_, idx = min((length[i],i) for i in xrange(len(length)))
		true_edge = distance[idx][1]
		true_edge_geom = distance[idx][2].item()
		projected_point = true_edge_geom.interpolate(true_edge_geom.project(gps_point)) # projected point

		return projected_point, true_edge_geom

	# covnerts latitude and longitude to utmx and utmy
	def _gps_to_utm(self, lat, lon):
		UTMx, UTMy, _, _ = utm.from_latlon(lat, lon)
		return Point(UTMx, UTMy)

	# creates a message of type pointStamped
	def _update_pp_msg(self, stamp, x, y):
		self._projected_point.header.stamp = stamp
		self._projected_point.point.x = x
		self._projected_point.point.y = y

	# check if a waypoint is passed or not
	def _check_waypoint(self, new_edge):
		if self._prev_matched_edge != None:
			if new_edge != self._prev_matched_edge:
				return True
			else:
				return False
		else:
			return False

	# calculates distance between point and last point on the path segment
	def _distance_to_next_waypoint(self, matched_edge, matched_point):
		next_wp = matched_edge.coords[:][len(matched_edge.coords[:])-1]
		return matched_point.distance(Point(next_wp[0], next_wp[1]))

	# updates the current edge that the projected point lies on currently
	def _update_matched_edge(self, new_edge):
		self._prev_matched_edge = new_edge

	# calls the /path_getter service to get path waypoints
	def _get_path_points(self):
		self._path = Path()

		rospy.loginfo("Waiting for /path_getter service ...")
		rospy.wait_for_service('path_getter')

		try:
			self._path_getter_service = rospy.ServiceProxy('path_getter', getPath)
			self._path = self._path_getter_service(434587,4462624) # service call with hardcoded goal point
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed: %s", e)

		for point in self._path.path.poses:
			path_point = (point.pose.position.x, point.pose.position.y)
			self._path_points.append(path_point)

		rospy.loginfo("Done getting path points ...")

		self._get_path_edges()
		self._get_path_nodes()

		return self._path_points
		
	# creates geodataframes of the path edges
	def _get_path_edges(self):
		path_edges = []
		data = {'id': [], 'u': [], 'v': [], 'geometry': []}

		for i in range(0, len(self._path_points)-1):
			path_edges.append([(self._path_points[i][0], self._path_points[i][1]), (self._path_points[i+1][0], self._path_points[i+1][1])])

		for i in range(0, len(path_edges)):
			data['u'].append(i)
			data['v'].append(i+1)

		for i in range(0, len(path_edges)):
			data['id'].append(i)
			data['geometry'].append(LineString(path_edges[i]))

		edges_df = pd.DataFrame(data, columns = ['id', 'u', 'v', 'geometry'])
		self._edges_gdf = geopandas.GeoDataFrame(edges_df, geometry='geometry')

		length_array = []

		for line in self._edges_gdf.geometry:
		    length_array.append(line.length)

		self._max_segment_length = max(length_array)

		return self._edges_gdf

	# creates a geodataframe of the path nodes
	def _get_path_nodes(self):
		path_nodes = []
		data = {'id': [],'geometry': []}

		for i in range(0, len(self._path_points)):
			data['id'].append(i)
			data['geometry'].append(Point(self._path_points[i][0], self._path_points[i][1]))

		nodes_df = pd.DataFrame(data, columns = ['id', 'geometry'])
		self._nodes_gdf = geopandas.GeoDataFrame(nodes_df, geometry='geometry')

		self._nodes_spatial_index = self._nodes_gdf.sindex

		return self._nodes_gdf

	# creates a subscriber to the sensor msg
	def _subscribe_to_gps(self):
		rospy.loginfo("Waiting for gps points ...")
		self._gps_subscriber = rospy.Subscriber("/ada/fix", NavSatFix, self._gps_callback)

	def _plot_path(self):
		edges = self._get_path_edges()
		nodes = self._get_path_nodes()

		edges.plot(ax=self._axis, color='b')
		nodes.plot(ax=self._axis, marker='.', color='g')

	def _plot_matching_output(self):
		for point in self._matched_points:
			plt.plot(point.x, point.y, 'k.')

# 2700