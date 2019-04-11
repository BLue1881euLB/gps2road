import geopandas
import pandas as pd

from shapely.geometry import Point
from shapely.geometry import LineString

import matplotlib.pyplot as plt

class path:
	def __init__(self, path_points):
		self.path_points = path_points
		self.make_path_edges()
		self.make_path_nodes()

	def make_path_edges(self):
		path_edges = []
		data = {'id': [], 'u': [], 'v': [], 'geometry': []}

		for i in range(0, len(self.path_points)-1):
			path_edges.append([(self.path_points[i][0], self.path_points[i][1]), (self.path_points[i+1][0], self.path_points[i+1][1])])

		for i in range(0, len(path_edges)):
			data['u'].append(i)
			data['v'].append(i+1)

		for i in range(0, len(path_edges)):
			data['id'].append(i)
			data['geometry'].append(LineString(path_edges[i]))

		edges_df = pd.DataFrame(data, columns = ['id', 'u', 'v', 'geometry'])
		self.path_edges = geopandas.GeoDataFrame(edges_df, geometry='geometry')

		# fig, ax = plt.subplots()
		# self.path_edges.plot(ax=ax)
		# plt.show()

	def make_path_nodes(self):
		path_nodes = []
		data = {'id': [],'geometry': []}

		for i in range(0, len(self.path_points)):
			data['id'].append(i)
			data['geometry'].append(Point(self.path_points[i][0], self.path_points[i][1]))

		nodes_df = pd.DataFrame(data, columns = ['id', 'geometry'])
		self.path_nodes = geopandas.GeoDataFrame(nodes_df, geometry='geometry')

	def get_path_edges(self):
		return self.path_edges

	def get_path_nodes(self):
		return self.path_nodes

	def get_nodes_spatial_idx(self):
		return self.path_nodes.sindex

	def get_edges_spatial_idx(self):
		return self.path_edges.sindex

	def get_edge_length(self, edge):
		return self.path_edges.geometry[edge].length