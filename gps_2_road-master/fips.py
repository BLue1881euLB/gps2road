import pandas as pd
from urllib import request
import zipfile
from io import BytesIO
import os

import geopandas as gpd

def get_fips(fips_filter = {}):
	"""
		start at widest filter and move down
	"""
	fips = pd.read_csv('state_county_fips.txt', dtype = 'object')
	if fips_filter:
		if 'state_fips' in fips_filter and len(fips_filter['state_fips']) > 0:
			fips = fips.loc[fips['state_fips'].isin(set(fips_filter['state_fips'])),]
		if 'state_abbreviation' in fips_filter and len(fips_filter['state_abbreviation']) > 0:
			fips = fips.loc[fips['state_abbreviation'].isin(fips_filter['state_abbreviation']),]
		if 'county_fips' in fips_filter and len(fips_filter['county_fips']) > 0:
			fips = fips.loc[fips['county_fips'].isin(fips_filter['county_fips']),]
		if 'county_name' in fips_filter and len(fips_filter['county_name']) > 0:
			fips = fips.loc[fips['county_name'].isin(fips_filter['county_name']),]

	return fips


def get_county_shapes():
	if not os.path.exists("cb_2016_us_county_500k/cb_2016_us_county_500k.shp"):
		url = "http://www2.census.gov/geo/tiger/GENZ2016/shp/cb_2016_us_county_500k.zip"
		rsp = request.urlopen(url, timeout = 10)
		zip = zipfile.ZipFile(BytesIO(rsp.read()))

		os.mkdir("cb_2016_us_county_500k")
		zip.extractall("cb_2016_us_county_500k")

	return gpd.read_file("cb_2016_us_county_500k/cb_2016_us_county_500k.shp")


def get_overlapping_counties(route):
	"""
	expects geopandas dataframe from data from
	convert_gpx_to_shp, outputtype="Points"
	"""
	cs = get_county_shapes()
	overlap = cs[route.geometry.intersects(cs.geometry)]

	fips = pd.read_csv('state_county_fips.txt', dtype = 'object')
	fips_keep = pd.merge(overlap, fips, left_on=["COUNTYFP", "STATEFP"], right_on=["county_fips", "state_fips"])

	return fips_keep.loc[:, ["county_fips", "state_fips", "state_abbreviation",
	                         "county_name"]]
