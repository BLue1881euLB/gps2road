from urllib import request
import zipfile
from io import BytesIO
import os
import time
import fiona
import shapely
import numpy as np

from shapely.geometry import shape
from utils import overlap
from fips import get_fips

import geopandas as gpd
import pandas as pd

def get_roads(fips_df = None, states = [], counties = []):
	if fips_df is None:
		fips_df = get_fips(fips_filter = {'state_abbreviation':states,
		                                 'county_name':counties})

	ftp = "ftp://ftp2.census.gov/geo/tiger/TIGER2016/ROADS/tl_2016_{state_fips}{county_fips}_roads.zip"
	dir = 'roads/tiger/{state_fips}{county_fips}_roads'
	errors = {'states':[],'counties': []}

	for i,row in fips_df.iterrows():
		state_fips,county_fips = row['state_fips'],row['county_fips']
		print (state_fips,county_fips)

		try:
			if os.path.exists(dir.format(state_fips = state_fips, county_fips = county_fips)):
				continue

			rsp = request.urlopen(ftp.format(state_fips = state_fips, county_fips = county_fips), timeout = 10)
			zip = zipfile.ZipFile(BytesIO(rsp.read()))
			os.mkdir(dir.format(state_fips = state_fips, county_fips = county_fips))
			zip.extractall(dir.format(state_fips = state_fips, county_fips = county_fips))

		except Exception as e:
			print (e,state_fips,county_fips)
			errors['states'].append(row['state_abbreviation'])
			errors['counties'].append(row['county_name'])

		time.sleep(5)

	if len(errors['states']) > 0:
		time.sleep(300)
		getRoads(states = errors['states'], counties = errors['counties'])


def concatenate_roads(fips_df):
	roads_dfs = []
	template = "roads/tiger/{0}{1}_roads/tl_2016_{2}{3}_roads.shp"
	for _, row in fips_df.iterrows():
		state_fips, county_fips = row["state_fips"], row["county_fips"]
		roads_dfs.append(
		    gpd.read_file(template.format(state_fips, county_fips,
		                                  state_fips, county_fips))
		)

	return pd.concat(roads_dfs)


def union_roads(fips_filter = {}):

	base = 'roads/tiger/{state_fips}{county_fips}_roads/tl_2016_{state_fips}{county_fips}_roads.shp'

	isAllOpen = False
	allBase = ''
	for var in ['state_fips', 'state_abbreviation', 'county_fips', 'county_name']:
		if var in fips_filter:
			allBase += var + '_' + '_'.join(['_'.join(fips_filter[var])])

	allPath = 'union_roads/{}/{}.shp'.format(allBase, allBase)

	if not os.path.exists('union_roads/'):
		os.mkdir('union_roads/')
	if not os.path.exists('union_roads/{}/'.format(allBase)):
		os.mkdir('union_roads/{}/'.format(allBase))

	fips_df = get_fips(fips_filter=fips_filter)

	for i,row in fips_df.iterrows():
		state_fips, county_fips = row['state_fips'],row['county_fips']

		with fiona.open(base.format(state_fips=state_fips, county_fips=county_fips), 'r') as shp:

			if not isAllOpen:
				with fiona.open(allPath, 'w', driver=shp.driver, crs=shp.crs, schema=shp.schema) as concat:
					for rec in shp:
						concat.write(rec)
				isAllOpen = True
			else:
				with fiona.open(allPath,'a') as concat:
					for rec in shp:
						concat.write(rec)

	return allPath


def get_road_map():
	"""
		Inverse of map road to bounded box
		(so map bounded box to road)

		This is used to see if a the bounded box of a ride overlaps with
		the bounded box of a road
	"""
	roadPaths = os.listdir('roads/tiger')

	road_map = {}
	def unpack(l):
		r = [[], []]
		for t in l:
			r[0].append(t[0]); r[1].append(t[1])
		return r

	for rp in roadPaths:
		coord = [[],[]]
		with fiona.open('roads/tiger/' + rp + '/' + 'tl_2016_' + rp + '.shp', 'r') as shp:

			for rec in shp:
				item = rec['geometry']['coordinates']
				if rec['geometry']['type'] == 'LineString':
					t = [item]

				for c in t:
					c = unpack(c)

					coord[0] += c[0]
					coord[1] += c[1]

		road_map[(min(coord[0]), min(coord[1]), max(coord[0]), max(coord[1]))] = \
			{'state_fips': rp[0:2], 'county_fips': rp[2:5]}

	return road_map

def get_roads_in_bounded_box(box, map_box_to_road):

	road_boxes = np.array(list(map_box_to_road.keys()))

	overlap_index = overlap(box, road_boxes)
	overlapped = road_boxes[overlap_index]

	results = []
	for i in range(overlapped.shape[0]):
		t = tuple(overlapped[i])
		results.append(map_box_to_road[t])

	return results
