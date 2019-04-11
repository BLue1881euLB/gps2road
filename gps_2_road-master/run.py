import os
from datetime import datetime

import routes
import roads
import gpd_project

import geopandas as gpd
import fips

def runner():
	times = open('times.csv', 'w')
	times.write("n_points,proj_time,tot_time,\n")
	print("converting route gpx files to shp files...")
	routes.convert_all_gpx_to_shp(activity_type='Ride', output_type="Point")
	print("reading route shp files in as dataframes...")
	route_gpds = routes.get_geopandas_from_shp(activity_type="Ride",
	                                            output_type="Point")
	print("projecting routes...")
	for route_gpd in route_gpds:
		start = datetime.now()
		print("\troute: ", route_gpd.iloc[0].path)
		print("\tgetting road data...")
		print("\t\toverlapping_counties...")
		fips_df = fips.get_overlapping_counties(route_gpd)
		if len(fips_df) == 0:
		    print("\t\tno overlapping counties-->continue")
		    continue
		print("\t\tfetch tiger data...")
		roads.get_roads(fips_df=fips_df)
		print("\t\tconcatenate road shp files to geopandas dataframes")
		roads_df = roads.concatenate_roads(fips_df=fips_df)
		project_start = datetime.now()
		n_points = len(route_gpd)
		print(f"\tprojecting route ({n_points} points)...")
		proj = gpd_project.Project(roads_df)
		route_gpd["road_name"] = route_gpd.geometry.apply(proj.project_point)
		project_end = datetime.now()
		project_time = (project_end - project_start).seconds
		print(f'\tprojection took {project_time} seconds',)

		print("\twriting results...")
		route_name = route_gpd.iloc[0].path.split('.')[0]
		if not os.path.exists("results/" + route_name):
		    os.mkdir("results/" + route_name)
		route_gpd.to_file("results/" + route_name)

		finish = datetime.now()
		total_time = (finish - start).seconds
		print(f'\troute took {total_time} seconds')
		times.write(f"{n_points},{project_time},{total_time}\n")
	times.close()

if __name__ == '__main__':
	runner()
