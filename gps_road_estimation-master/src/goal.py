import rospy

from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray

from shapely.geometry import Point

class goal:
	def __init__(self, path_points):
		self.path_points = path_points
		self.goal_arr = GoalStatusArray()
		self.make_goal_array()

		self.position_uncertainity = 5.0
		
	def make_goal_array(self):
		for path in range(0, len(self.path_points)-1):
			new_goal = GoalStatus()
			
			new_goal.goal_id.id = str(path)
			new_goal.status = 0
			new_goal.text = 'PENDING'

			self.goal_arr.status_list.append(new_goal)

	def get_goal_status(self, matched_point, matched_edge):
		next_wp = Point(self.path_points[matched_edge+1][0], 
			            self.path_points[matched_edge+1][1])
		
		self.curr_dist = self.get_dist(matched_point, next_wp)

		active_msg = 'ACTIVE. Distance to path point is: ' + str(self.curr_dist) + ' meters.'
		succeeded_msg = 'SUCCEEDED. Path point passed.'

		self.goal_arr.header.stamp = rospy.Time.now()

		if (matched_edge == len(self.path_points)-2) and (self.curr_dist < self.position_uncertainity):
			self.goal_arr.status_list[matched_edge].status = 3
			self.goal_arr.status_list[matched_edge].text = succeeded_msg
		elif (matched_edge == len(self.path_points)-2) and (self.curr_dist > self.position_uncertainity):
			self.goal_arr.status_list[matched_edge].status = 1
			self.goal_arr.status_list[matched_edge].text = active_msg
		else:
			self.goal_arr.status_list[matched_edge].status = 1
			self.goal_arr.status_list[matched_edge].text = active_msg

		for idx in range(0, matched_edge):
			self.goal_arr.status_list[idx].status = 3
			self.goal_arr.status_list[idx].text = succeeded_msg

		# rospy.loginfo(matched_edge)
		# rospy.loginfo(self.curr_dist)

		return self.goal_arr

	def final_goal_is_reached(self):
		goal_id = len(self.goal_arr.status_list)-1
		status = self.goal_arr.status_list[goal_id].status
		if status == 3:
			return True
		else:
			return False

	def goal_is_reached(self, goal_id):
		status = self.goal_arr.status_list[goal_id].status
		if status == 3:
			return True
		else:
			return False

	def get_dist(self, point1, point2):
		return point1.distance(point2)
