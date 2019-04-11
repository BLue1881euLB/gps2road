
# INPUT DATA DEFAULT PATH
PROBEFILE = "../probe_data_map_matching/Partition6467ProbePoints.csv"
LINKEFILE = "../probe_data_map_matching/Partition6467LinkData.csv"
OUTPUT = "./Partition6467MatchedPoints.csv"
OUTPUTSLOPE = "./Partition6467SlopeComaparsion.csv"


# OUTPUT

Output will be located in src folder

Map matching output is the following file:

Partition6467MatchedPoints.csv	The subset of probe points in partion 6467 that were successfully map-matched to a link.

MatchedPoints Record Format:

	sampleID, dateTime, sourceCode, latitude, longitude, altitude, speed, heading, linkPVID, direction, distFromRef, distFromLink

		sampleID	is a unique identifier for the set of probe points that were collected from a particular phone.
		dateTime	is the date and time that the probe point was collected.
		sourceCode	is a unique identifier for the data supplier (13 = Nokia).
		latitude	is the latitude in decimal degrees.
		longitude	is the longitude in decimal degrees.
		altitude	is the altitude in meters.
		speed		is the speed in KPH.
		heading		is the heading in degrees.
		linkPVID	is the published versioned identifier for the link.
		direction	is the direction the vehicle was travelling on thelink (F = from ref node, T = towards ref node).
		distFromRef	is the distance from the reference node to the map-matched probe point location on the link in decimal meters.
		distFromLink	is the perpendicular distance from the map-matched probe point location on the link to the probe point in decimal meters.

Slope comparison is the following file:

Partition6467SlopeComparison.csv	The subset of candidates in partion 6467 that were successfully compare to Link slope.

Slope comparison Record Format:

	sampleID, dateTime, sourceCode, latitude, longitude, altitude, speed, linkPVID, slope, link_slope, Deviation

		sampleID	is a unique identifier for the set of probe points that were collected from a particular phone.
		dateTime	is the date and time that the probe point was collected.
		sourceCode	is a unique identifier for the data supplier (13 = Nokia).
		latitude	is the latitude in decimal degrees.
		longitude	is the longitude in decimal degrees.
		altitude	is the altitude in meters.
		speed		is the speed in KPH.
		linkPVID	is the published versioned identifier for the link.
		slope		is the slope calculated by match point and its distance from the reference point.
		link_slope	is the actual slope in the matching segment.
		Deviation	is the deviation between slope and actual slope.