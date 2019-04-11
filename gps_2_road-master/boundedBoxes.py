import numpy as np
from utils import overlap
from roads import union_roads, get_roads_in_bounded_box

def getBoundedBoxes(routes, map_box_to_road):
	"""
		iterate over routes find all bounded boxes
		objective: minimize area in bounded boxes subject to all route data points
		are contained in bounded boxes

		create mapping from each bounding box to the roads contained in that box and
		the routes contained in that box
	"""
	boundedBoxes = None #np.array([[],[],[],[]]).T
	boxToRoutes = {}
	for route in iter(routes):
		box = getBoundedBox(route['coordinates'])

		if box in boxToRoutes: 	boxToRoutes[box].append(route)
		else: 					boxToRoutes[box] = [route]

		if boundedBoxes is not None and boundedBoxes.shape[0] > 0:
			""" get all already discovered bounded boxes that overlap with route bb """
			overlappedIndex = overlap(box, boundedBoxes)
			overlapped = boundedBoxes[overlappedIndex]
			"""
				if overlap then remove overlapped boxes from boundedBoxes, merge
				over lapped bounded boxes, place new bounded box back in boundedBoxes
			"""
			if overlapped.shape[0] > 0:

				boundedBoxes = boundedBoxes[~overlappedIndex]

				overlapped = np.vstack((box,overlapped))

				update = []
				for i in range(overlapped.shape[0]):
					update += boxToRoutes[tuple(overlapped[i])]
					boxToRoutes.pop(tuple(overlapped[i]), None)

				overlapped = mergeBoundedBoxes(overlapped)

				boxToRoutes[overlapped] = update

				boundedBoxes = np.vstack((boundedBoxes, overlapped))
			else:
				boundedBoxes = np.vstack((boundedBoxes, box))
		else:
			boundedBoxes = np.array([[box[0]],[box[1]],[box[2]],[box[3]]]).T

	box_to_roads = {}
	keep = [True] * boundedBoxes.shape[0]
	keep = np.array(keep)
	for i in range(boundedBoxes.shape[0]):
		bb = tuple(boundedBoxes[i])
		fips = get_roads_in_bounded_box(bb, map_box_to_road)
		# don't have road data for this bounded box
		if len(fips) == 0:
			keep[i] = False
			boxToRoutes.pop(bb)
			continue
		fips_filter = {'state_fips':[f['state_fips'] for f in fips], \
			'county_fips':[f['county_fips'] for f in fips]}
		box_to_roads[bb] = union_roads(fips_filter=fips_filter)
	boundedBoxes = boundedBoxes[keep]
	return boundedBoxes, box_to_roads, boxToRoutes


def getBoundedBox(coord):
	return (np.min(coord[:,0]), np.min(coord[:,1]), np.max(coord[:,0]), np.max(coord[:,1]))

def mergeBoundedBoxes(boxes):
	return (np.min(boxes[:,0]), np.min(boxes[:,1]), np.max(boxes[:,2]), np.max(boxes[:,3]))
