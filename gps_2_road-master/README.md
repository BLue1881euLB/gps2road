# gps_2_road
Project gps data points to the nearest road

These scripts were created in order to project GPS points from my bike rides onto the nearest road.  

Steps

1.  **Gather:** We need gps points and roads.  I wrote this with Strava's gpx files in mind.  I use county level road data gathered by the US Census Bureau (ftp://ftp2.census.gov/geo/tiger/TIGER2016/ROADS/).  For now, one must enter the regions for which road data is needed (i.e. states, counties).  They will be downloaded with roads.py.

2.  **Format:** Parse the gpx files into a more eaily usable form.  Create bounded boxes for the roads and routes.  This allows us to quickly determine which counties overlap with a route and which routes overlap with eachother.

3.  **Project:** Break the projection problem into chunks.  Each "chunk" is a bounded box containing all overlapping rides.  For each box, get the roads that overlap with the box.  For each ride in chunk, project the k-th gps emission to the nearest road.

Python Version: 3.5

Packages:

1.  Numpy -- 1.11.1
2.  Fiona -- 1.7.0.post2
3.  Shapely -- 1.5.17
4.  Pandas -- 0.18.1
5.  BeautifulSoup -- 4.5.1

TODO:

0.  Automate environment creation and directory setup
1.  Automate the road gathering process.
2.  Implement a clever projection method such as HMM.
3.  Sample based on time interval rather than every k-th emission
