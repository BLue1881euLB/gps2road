import fiona
import shapely
import numpy as np
from fips import get_fips
import utils
from boundedBoxes import getBoundedBoxes
from roads import get_road_map, get_roads_in_bounded_box, union_roads
import matplotlib.pyplot as plt
import os

from geopy.distance import vincenty

plt.style.use('ggplot')

class Project():
	def __init__(self, routes):
		self.routes = routes

		map_box_to_road = get_road_map()
		self.boundedBoxes, self.boxToRoads, self.boxToRoutes = getBoundedBoxes(self.routes, map_box_to_road)

	def project(self, boundedBox, inc = 1/60.0, interval = 10):
		"""
			https://www2.usgs.gov/faq/categories/9794/3022
			http://www.latlong.net/degrees-minutes-seconds-to-decimal-degrees

			one minute ~ .9 miles
			^ Deg. Decimal = d + (min/60) + (sec/3600)
			==> increase bound by 1/60
		"""
		# TODO: use a more sophisticated projection method such as HMM

		# get relevant roads and routes
		roads, routes = self.boxToRoads[tuple(boundedBox)], self.boxToRoutes[tuple(boundedBox)]

		recs = {}
		boxes = []

# 		map expanded bounded box to the record and create shapely object for each record
		if not os.path.exists(roads):
			return False
		with fiona.open(roads) as shp:
			for rec in shp:
				rec['object'] = shapely.geometry.shape(rec['geometry'])
				box = rec['object'].bounds
				box = (box[0] - inc, box[1] - inc, box[2] + inc, box[3] + inc)
				recs[box] = rec
				boxes.append(box)

		boxes = np.array(boxes)
		roadName = None
		for route in iter(routes):
			route['road name'] = []
			with open('results/' + route['path'] + '_proj.csv', 'w') as results:
				results.write('timestamp,longitude,latitude,roadname,distance\n')
				geom = route['coordinates']
				d = []
# 				TODO: iterate over time interval i.e. sample every 5 seconds not every 5 gps pts
				# sample = utils.sample(geom, interval=5)
				for i in np.arange(0,geom.shape[0]):#,interval):
					point = shapely.geometry.Point((geom[i,0], geom[i,1]))

# 					get all bounded boxes containing the point
					candidates = boxes[utils.point_in_box((point.x, point.y), boxes)]
					if candidates.shape[0] == 0:
						continue
					minimizer = (1000000.0,None)
					for j in range(candidates.shape[0]):
						rec = recs[tuple(candidates[j])]
						proj = rec['object'].project(point)
						rpt = rec['object'].interpolate(proj)
						dist = vincenty((point.x, point.y), (rpt.x, rpt.y)).km
						if dist < minimizer[0]: minimizer = (dist,j)

					roadName = recs[tuple(candidates[minimizer[1]])]['properties']['FULLNAME']
					# TODO handle interval here
					# for j in range(i,i + interval):
					# 	route['road name'].append(roadName)

					results.write(','.join([route['timestamp'][i], str(point.x),str(point.y),roadName if roadName else '',str(minimizer[0]),'\n']))

		return routes
