"""
Map Matching
Reference paper: Map-Matching for Low-Sampling-Rate GPS Trajectories
https://www.microsoft.com/en-us/research/publication/map-matching-for-low-sampling-rate-gps-trajectories/
"""
import csv
import numpy as np
import networkx as nx
import scipy.stats
from math import radians, degrees, cos, sin, sqrt, atan2, atan, inf

PROBEFILE = "../probe_data_map_matching/Partition6467ProbePoints.csv"
LINKEFILE = "../probe_data_map_matching/Partition6467LinkData.csv"
OUTPUT = "./Partition6467MatchedPoints.csv"
OUTPUTSLOPE = "./Partition6467SlopeComaparsion.csv"
MPStoKPH = 3.6

class Probe():
	def __init__(self,record):
		self.ID = record[0]
		self.dateTime = record[1]
		self.sourceCode = record[2]
		self.lat = float(record[3])
		self.lon = float(record[4])
		self.alt = float(record[5])
		self.speed = float(record[6])
		self.heading = float(record[7])
		self.interval = 0 # dist(pi,pi-1)
		self.candidates = None

	def getAttr(self):
		return self.__dict__

	def point(self):
		return np.array([self.lat,self.lon])

	def setInterval(self, ival):
		self.interval = ival


class Point():
	def __init__(self,record):
		self.lat = float(record[0])
		self.lon = float(record[1])
		self.alt = None if len(record[2]) < 1 else float(record[2])

	def getAttr(self):
		return self.__dict__

	def getPoint(self):
		return np.array([float(self.lat), float(self.lon)])

	def getNode(self):
		return tuple(np.array([float(self.lat), float(self.lon)]))


class Candidate():
	def __init__(self, cand, dist, segment, fromSpeedLimit, toSpeedLimit, linkPVID):
		self.candidate = cand
		self.dist = float(dist) # distance between probe point to node point
		self.canDist = 0 # distance between probe point to candidate point
		self.segment = segment
		self.prob = None
		self.fromSpeedLimit = fromSpeedLimit
		self.toSpeedLimit = toSpeedLimit
		self.spatial = 0
		self.temporal = 0
		self.score = None
		self.linkPVID = linkPVID
		self.direction = None
		self.prev = None
		self.heading = 0
		self.link = None

	def __str__(self):
		return str(tuple((self.candidate, self.dist)))


def cut_link(shape):
	shapes = []
	for point in shape.split('|'):
		point = Point(point.split('/'))
		shapes.append(point)
	return shapes


def derive_slope(slope):
	slopes = []
	info = dict()
	for slope in slope.split('|'):
		if len(slope) != 0:
			slopeInfo = slope.split('/')
			info['dist'] = float(slopeInfo[0])
			info['slope'] = float(slopeInfo[1])
			slopes.append(info)

	return slopes


def ToPoint(point):
	return np.array([float(point[0]),float(point[1])])


class Link():
	def __init__(self, record):
		self.ID = record[0]
		self.refNodeID = record[1]
		self.nrefNodeID = record[2]
		self.length = float(record[3])
		self.functionalClass = record[4]
		self.direction = record[5]
		self.speed = float(record[6])
		self.fromRefSpeedLimit = float(record[7])
		self.toRefSpeedLimit = float(record[8])
		self.fromRefNumLanes = record[9]
		self.toRefNumLanes = record[10]
		self.multiDigitized = record[11]
		self.urban = record[12]
		self.timeZone = record[13]
		self.shape = cut_link(record[14])
		self.curvature = record[15]
		self.slope = derive_slope(record[16])

	def getAttr(self):
		return self.__dict__

	def getPoint(self):
		return [item.getPoint() for item in self.shape]

	def getNode(self):
		return [item.getNode() for item in self.shape]


def distance(P1, P2, P):
	return np.linalg.norm(np.cross(P2-P1, P1-P))/np.linalg.norm(P2-P1)


# haversine ref distance real earth: https://www.movable-type.co.uk/scripts/latlong.html 
def point_dist(P1, P2):
	lat1, lon1 = P1[0], P1[1]
	lat2, lon2 = P2[0], P2[1]
	lat1, lon1, lat2, lon2 = map(radians, (lat1, lon1, lat2, lon2))
	lat = lat2 - lat1
	lon = lon2 - lon1
	a = sin(lat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(lon / 2) ** 2
	c = 2 * atan2(sqrt(a), sqrt(1-a))
	return 6371000 * c # Earth Radius is 6371000


# http://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
def angle(P1, P2):
	lat1, lon1 = P1[0], P1[1]
	lat2, lon2 = P2[0], P2[1]
	lon = lon2 - lon1
	X = cos(lat2)*sin(lon)
	Y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon)

	return degrees(atan2(X, Y)) % 360


# initial data as Link and Probe
def data_processing():
	# initial probe list 
	probe_list = []
	print('Loading probes file......')

	with open(PROBEFILE, 'r') as f:
		probes = csv.reader(f)
		for index, probe in enumerate(probes):
			# create a new Probe object
			probe = Probe(probe)
			probe_list.append(probe)
			interval = point_dist(probe_list[index].point(),probe_list[index-1].point()) if index != 0 else 0
			probe_list[index].setInterval(interval)
			if index == 30000:
				break
			
	sorted(probe_list,key = lambda probe: probe.dateTime)
	f.close()
	num_probes = len(probe_list)
	print('Loaded', num_probes,'probe points!')


	# initial Link list as graph
	link_list = []
	print('Loading Link file......')

	with open(LINKEFILE, 'r') as f:
		links = csv.reader(f)
		for index, link in enumerate(links):
			link = Link(link)
			link_list.append(link)

	link_list = sorted(link_list, key=lambda link: (link.shape[0].lat, link.shape[0].lon))
	f.close()

	num_link = len(link_list)
	print('Loaded', num_link,'probe points!')

	return probe_list,link_list


#group (n to n-1 pair) eg: [1,2,3] to [(1,2),(2,3)]
def group(lst, n):
	leng = len(lst) - 1
	segments = []
	for i in range(leng):
		segments.append(np.array([lst[i],lst[i+1]]))
	return segments


# get projection point ref: https://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
def projection_point(probe, segment):
	p = np.array(probe)
	a = np.array([segment[0].lat, segment[0].lon])
	b = np.array([segment[1].lat, segment[1].lon])

	dp = np.dot(b-a, p-a)
	if dp <= 0:
		return segment[0].getPoint()

	squared = np.dot(b-a, b-a)
	if dp >= squared:
		return segment[1].getPoint()
	r = dp/squared

	P = a + (b - a) * r
	return P


# find candidate for each probe point
def find_candidate(links, probe):
	candidate_points = []
	link_list = []
	point = probe.point() 
	for link in links:
		dist = point_dist(point, link.getPoint()[0])
		if dist < link.length:
			link_list.append(link)
			segments = group(link.shape, 2)
			for segment in segments:

				candidate = projection_point(point, segment)
				dst = point_dist(candidate, point)
				if dst < 20:
					fromSpeedLimit = link.fromRefSpeedLimit
					toSpeedLimit = link.toRefSpeedLimit
					ID = link.ID
					Can = Candidate(candidate, dist, segment, fromSpeedLimit, toSpeedLimit, ID)
					Can.prob = scipy.stats.norm.pdf(dst, 0, 20)
					Can.canDist = dst
					Can.heading = probe.heading
					Can.link = link
					candidate_points.append(Can)
	return candidate_points, link_list


# spatial and temporal analysis
def spatial_temporal_analysis(prevCandidates, currCandidates, probe):
	for Ci_ in prevCandidates:

		#create start point

		graph = nx.Graph()
		ci_ = tuple(Ci_.candidate)
		ci_SegmentStart = Ci_.segment[0].getNode()
		ci_SegmentEnd = Ci_.segment[1].getNode()
		speed_list = []

		for Ci in currCandidates:
			ci = tuple(Ci.candidate)
			if ci_ == ci:

				Ci.temporal = 1.0
				Ci.spatial =Ci_.spatial
				Ci.direction = Ci_.direction
				continue

			ciSegmentStart= Ci.segment[0].getNode()
			ciSegmentEnd= Ci.segment[1].getNode()
			# calulate heading
			ang = angle(ciSegmentStart,ciSegmentEnd)

			if abs(Ci.heading - ang) < 90:
				Ci.direction = 'F' if Ci.link.direction == 'B' else Ci.link.direction
			else:
				Ci.direction = 'T' if Ci.link.direction == 'B' else Ci.link.direction


			if ci_SegmentStart == ciSegmentStart:
				
				graph.add_edge(ci_,ci)
				speed_list.append(Ci_.toSpeedLimit)
			else:
				
				graph.add_edge(ci_,ci_SegmentEnd, speedlimit = Ci_.fromSpeedLimit)
				graph.add_edge(ci_SegmentEnd,ciSegmentStart, speedlimit = Ci.fromSpeedLimit)
				graph.add_edge(ciSegmentStart,ci, speedlimit = Ci.fromSpeedLimit)
				speed_list.extend([Ci_.fromSpeedLimit,Ci.fromSpeedLimit,Ci.fromSpeedLimit])

			# shortest path len
			sp = nx.shortest_path(graph,source=ci_,target=ci)
			
			W = 0
			for i in range(len(sp)-1):
				w = point_dist(sp[i],sp[i+1]) 
				W += w


			# temporal analysis
			Vkph = W/5*MPStoKPH

			total = 0
			totalV = 0
			totalS = 0
			for speed in speed_list:
				total += Vkph*speed
				totalV += Vkph**2
				totalS += speed**2

			Ft = total/(np.sqrt(totalV)*np.sqrt(totalS)) if totalS != 0 else 1.0 # only one way to go(since speed no limit)
			Ci.temporal = Ft

			# spatial analysis
			d = probe.interval
			V = d/W
			N = Ci.prob
			
			Fs = N * V

			Ci.spatial = Fs

		print(Ci_)


# candidate preparation
def candidate_prepare(probes,links):

	candidate_list = []
	candidate_links = []

	for index,probe in enumerate(probes):
		# point = probe.point()
		point = probe
		candidate_points = None
		# print(probe.interval)
		

		if probe.interval < 1 and index != 0:
			# print('probe point',index + 1,'stop')
			if probe.ID != probes[index-1].ID: # if phone change, go back all link to find
				candidate_points, link_list = find_candidate(links, point)
				continue
			candidate_points, link_list = find_candidate(candidate_links[index-1], point)
			candidate_links.append(link_list)
		else:
			# print('probe point',index + 1)
			candidate_points, link_list = find_candidate(links, point)
			candidate_links.append(link_list)

		probe.candidates = candidate_points
		candidate_list.append(candidate_points)

		if index !=0:
			print('probe point', index, 'Candidates:')
			spatial_temporal_analysis(candidate_list[index-1], candidate_list[index], probe)

	# for I,C in enumerate(candidate_list):
	# 	print(I)
	# 	for c in C: 
	# 		print(c.spatial)
	# 		print(c.temporal)

	return candidate_list


# ST-matching
def result_matching(probe_list, candidates):

	# ST-matching
	finalCandidates = []
	for c in candidates[0]:
		c.score = c.prob

	for index,probe in enumerate(probe_list):

		maxCand = None
		if len(candidates[index]) != 0 and index != 0:
			for currCand in candidates[index]:
				currMax = -inf
				for prevCand in candidates[index-1]:
					F =  currCand.spatial * currCand.temporal
					alt = prevCand.prob + F

					if alt > currMax:
						currMax = alt
						currCand.prev = prevCand
						maxCand = currCand

					currCand.score = currMax

		elif index == 0:
			currMax = inf
			for currCand in candidates[0]:
				if currCand.dist < currMax:
					maxCand = currCand

		finalCandidates.append(maxCand)

	# for i,c in enumerate(finalCandidates):
	# 	print(i)
	# 	if c is not None:
	# 		print(c.direction)
	# 		print(c.canDist)
	return finalCandidates


def output_result(probe_list, candidates):
	with open(OUTPUT, 'w', newline ='') as f:
		result = csv.writer(f)
		for P,C in zip(probe_list,candidates):
			if C is not None:
				ID, dateTime, sourceCode, lat, lon, alt, speed, heading = P.ID, P.dateTime, P.sourceCode, P.lat, P.lon, P.alt, P.speed, P.heading
				linkPVID, direction, distFromRef, distFromLink = C.linkPVID, C.direction, C.dist, C.canDist
				result.writerow((ID, dateTime, sourceCode, lat, lon, alt, speed, heading, linkPVID, direction, distFromRef, distFromLink))
	f.close()


# slope
def evaluate_slope(link_list):

	outputs = []

	with open(OUTPUT, 'r') as f:
		matchPoints = csv.reader(f)

		for i,point in enumerate(matchPoints):
			matchLink = list(filter(lambda l : l.ID == point[8], link_list))[0]
			
			linkSlopes = matchLink.slope
			if len(linkSlopes) != 0:

				distRef = float(point[10])
				distLink = float(point[11])

				# a**2 = b**2 + c**2
				canToRef = sqrt(abs(distRef**2 - distLink**2))

				seg_slope = None
				
				for seg in linkSlopes:
					if canToRef < seg['dist']:
						seg_slope = seg


				refIndex = None
				refNodes = group(matchLink.shape,2)
				for index, seg in enumerate(refNodes):
					d = point_dist(seg[0].getPoint(), seg[1].getPoint())
					if seg_slope['dist'] > d:
						refIndex = index

				ref = matchLink.shape[refIndex]

				# print(ref.alt)

				point_alt = float(point[5])

				# # ref_alt = float(matchLink)
				high = ref.alt - point_alt if point[9] == 'T' else point_alt - ref.alt
				# print(point_alt)
				
				# print(ref.alt)
				actual_slope = atan(high/distRef)
				dif = abs(seg_slope['slope'] - actual_slope)
				outputs.append((point[0], point[1], point[2], point[3], point[4], point[5], point[6], point[8], actual_slope, seg_slope['slope'], dif))
	f.close()

	# out put result
	with open(OUTPUTSLOPE, 'w', newline='') as f:
		
		out = csv.writer(f)
		for output in outputs:
			out.writerow((output))

	f.close()
	pass


def main():
	# 1.Data Processing
	probe_list, link_list = data_processing()

	# 2.Candidate Preparation
	# 3.Spatial Analysis
	# 4.temporal Analysis
	candidate_list = candidate_prepare(probe_list, link_list)

	# 5.Result Matching
	final_candidates = result_matching(probe_list, candidate_list)

	# 6.Output result
	output_result(probe_list, final_candidates)

	# 7.Evaluate slope and output
	evaluate_slope(link_list)

	pass


if __name__ == "__main__":
	main()
