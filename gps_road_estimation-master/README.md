# GPS Road Estimation:

gps_road_estimation is a **ROS** package written in **Python** that matches vehicle/robot position to open street maps' roads.

## 1. Config
go to 'gps_road_estiamtion/params/road_estimation.yaml' and configure the R-tree search radius and edit the topic names if needed

## 2. Running the package
run using `$ roslaunch gps_road_estimation odom_road_estimator.launch`

### 2.1 Subscribed Topics:

1. 'gps/odometry' (odometry msg)
2. 'route_points' (path msg)

### 2.2 Published Topics:

1. 'ada/projected_odometry' (odometry msg)
2. 'ada/goal_status' (action lib goal status array msg)

## 3. ROS Dependencies:

1. /gps_umd (package) install using `$ sudo apt-get install ros-kinetic-gps-umd`
2. /road_processing_planning (package) run using `$ roslaunch road_processing_planning route_points.launch`

## 4. Python Dependencies:

1. utm install `$ sudo pip2 install utm`
2. geopandas install `$ sudo pip2 install geopandas`
3. pandas install `$ sudo pip2 install pandas`
4. shapely install `$ sudo pip2 install shapely`

## Important note:

Feel free to use the "git issues" to report any bugs or issues while using the package, i will be glad to fix it.