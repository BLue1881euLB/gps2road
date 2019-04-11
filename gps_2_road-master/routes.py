import os
import numpy as np
import fiona
from bs4 import BeautifulSoup
import json

from shapely.geometry import Point
import geopandas as gpd

def get_activity_paths(activity_type = 'Ride'):
	activities = os.listdir('activity_data/gpx_data')
	paths = []

	for a in activities:
		if activity_type in a:
			paths.append(a)

	return paths

""" parse routes in gpx file to dictionary containing:
		path: source gpx file
		coordinates: list of (longitude, latitude)
		elevation: list of elevation above sea level in meters
		timestamp: time of gps emission
"""
def parse_routes(routePaths):
	routes = []
	for routePath in routePaths:
		with open('activity_data/gpx_data/' + routePath) as f:
			soup = BeautifulSoup(f.read(),'lxml')

		lons, lats, elevs, times = [],[],[],[]

		metadata = soup.find('metadata')
		date = metadata.find('time')
		date = date.get_text()

		for trkpt in soup.find_all('trkpt'):
			lons.append(float(trkpt['lon']))
			lats.append(float(trkpt['lat']))

			elev = trkpt.find('ele')
			elev = float(elev.get_text())
			elevs.append(elev)

			time = trkpt.find('time')
			times.append(time.get_text())

		routes.append({'path':routePath,'coordinates':np.array([lons,lats]).T, \
			'elvation':np.array(elevs),'timestamp':times})

	return routes


def routes_to_shp(routes, path_out="routes_shp/routes.shp", output_type="LineString"):
	def format_coordinates(coordinates):
		formatted = []
		for i in range(coordinates.shape[0]):
			formatted.append((coordinates[i, 0], coordinates[i, 1]))

		return formatted

	def write_line_string(routes, shp):
		for i in range(len(routes)):
			route = routes[i]
			rec = {}
			rec['id'] = str(i)
			rec['geometry'] = {'coordinates': format_coordinates(route['coordinates']),
		                   	'type': 'LineString'}
			rec['properties'] = {'path':route['path'],
								 'time': route['timestamp'][0]}
			shp.write(rec)

	def write_points(routes, shp):
		for i in range(len(routes)):
			route = routes[i]
			for j in range(len(route['coordinates'])):
				rec = {}
				rec['id'] = str(i + j)
				rec['geometry'] = {'coordinates': (route['coordinates'][j, 0], route['coordinates'][j, 1]),
							       'type': 'Point'}
				rec['properties'] = {'path':route['path'],
									 'time': route['timestamp'][i]}
				shp.write(rec)

	shp = fiona.open(path_out, "w",
	                 crs={'init': 'epsg:4269'},
					 driver="ESRI Shapefile",
					 schema={'properties': {'path': 'str:50',
					                        'time': 'str:50'},
					         'geometry': output_type})

	if output_type == 'LineString':
		output_fn = write_line_string
	elif output_type == 'Point':
		output_fn = write_points
	else:
		raise ValueError("Type must be LineString or Point")

	output_fn(routes, shp)
	shp.close()

def routes_to_shp_setup(activity_type='Ride'):
	paths = get_activity_paths(activity_type=activity_type)
	routes = parse_routes(paths)
	routes_to_shp(routes)


def convert_gpx_to_shp(gpx_path, output_type="LineString", overwrite=True):
	base_dir = 'activity_data/{0}_shp_data/'.format(output_type)
	shp_dir = gpx_path[gpx_path.rfind('/') + 1: gpx_path.rfind('.gpx')]
	final_shp_dir = os.path.join(base_dir, shp_dir)
	shp_file = os.path.join(final_shp_dir, 'route.shp')

	if overwrite or (not overwrite and os.path.exists(ship_file)):
		return

	if not os.path.exists(final_shp_dir):
		os.mkdir(final_shp_dir)

	route = parse_routes([gpx_path])
	routes_to_shp(route, shp_file, output_type=output_type)


def convert_all_gpx_to_shp(activity_type='Ride', output_type="LineString",
						   overwrite=True):
	gpx_paths = get_activity_paths(activity_type=activity_type)
	for gp in gpx_paths:
		convert_gpx_to_shp(gp, output_type=output_type, overwrite=overwrite)


def get_geopandas_from_shp(activity_type="Ride", output_type="LineString"):
	DIR = "activity_data/{0}_shp_data/".format(output_type)
	gpx_paths = os.listdir(DIR)

	gpd_dfs = []
	for gp in gpx_paths:
		if activity_type in gp:
			gpd_dfs.append(gpd.read_file(DIR + gp + "/route.shp"))

	return gpd_dfs
