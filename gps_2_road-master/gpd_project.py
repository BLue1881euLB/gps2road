import geopandas as gpd
import matplotlib.pyplot as plt
from shapely.strtree import STRtree
from shapely.geometry import MultiLineString
from geopy.distance import vincenty


class Project():
    def __init__(self, roads):
        self.roads = roads
        self.sindex = self.roads.sindex

    def project_point(self, point, tol=0.00001):
        hits_idx = []
        hits_idx = list(self.sindex.intersection(point.buffer(tol).bounds))
        if len(hits_idx) == 0:
            if tol < 1:
                return self.project_point(point, tol=tol*10)
            else:
                return "FAIL"
        hits = self.roads.iloc[hits_idx]
        return self.min_dist(point, hits)


    def min_dist(self, point, hits):

        def project(hit):
            geom = hit.geometry
            proj = geom.project(point)
            rpt = geom.interpolate(proj)
            dist = vincenty((point.x, point.y), (rpt.x, rpt.y)).km
            return dist

        projections = hits.apply(project, axis=1)
        loc = hits.index.get_loc(projections.idxmin())

        return hits.iloc[loc].FULLNAME


def map_match(route, roads):
    proj = Project(roads)
    route["road_name"] = route.geometry.apply(proj.project_point)
    return route


if __name__ == "__main__":
    route_path = "/Users/HANK/Documents/activities/gps_2_road/activity_data/Point_shp_data/20170827-121415-Ride"
    route = gpd.read_file(route_path)
    road_path = "/Users/HANK/Documents/activities/gps_2_road/roads/tiger/11001_roads"
    roads = gpd.read_file(road_path)
    route = map_match(route, roads)
