#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import osmnx as ox

from nav_msgs.msg import Odometry

odom_out = []
odom_pp_out = []

def plot_edges():
    print("getting graph for plot")
    edges = ox.graph_to_gdfs(ox.project_graph(ox.graph_from_place('leganes spain', network_type='drive')), nodes=False, edges=True)
    _, ax = plt.subplots()
    edges.plot(ax=ax)
    print("done plotting open street map")
    
def plot_output():
    for i in range(0, len(odom_out)):
        rospy.loginfo("plotting odometry")
        plt.plot(odom_out[i][0], odom_out[i][1], 'r.')

    for i in range(0, len(odom_pp_out)):
        rospy.loginfo("plotting projected odometry")
        plt.plot(odom_pp_out[i][0], odom_pp_out[i][1], 'k.')

    plt.show()

def odom_cb(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    odom_out.append((x, y))

    if msg.header.seq == 1:
        plot_edges()

    if msg.header.seq >= 1375: # 1375
        odometry_sub.unregister()
        plot_output()

def odom_pp_cb(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    odom_pp_out.append((x, y))

    if msg.header.seq >= 1375: #2700 fix 
        odom_pp_sub.unregister()
    
if __name__ == '__main__':
    try:

        rospy.init_node('mm_output_plot', anonymous=False)
        odometry_sub = rospy.Subscriber(rospy.get_param("road_estimation/odometry_topic"), Odometry, odom_cb)
        odom_pp_sub = rospy.Subscriber(rospy.get_param("road_estimation/published_topic"), Odometry, odom_pp_cb)
        
        rospy.spin()

    except Exception as e:
        print(e)
