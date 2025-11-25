# voronoi route planner

A package containing a route planner implemented as a ROS2 C++ node. It uses the A* algorithm to search the voronoi diagram of a pointcloud map for a collision-free route from a start position to a goal position. A class for the voronoi graph used in the search is also exposed as library header files by the package.

## Installation & Dependencies
This package depends on:
- rclcpp
- geometry_msgs
- PCL
- pcl_conversions
- pcl_ros
- Boost
- cca_interfaces

To install, please copy the directory into the workspace and install with a build tool. This package was produced with a ROS2 toolchain as a C++/CMake package for colcon using `ament_cmake`:
```bash
colcon build --package-select voronoi_route_planner
```

## Usage
Route planning node:
```bash
ros2 run voronoi_route_planner RoutePlanner
```
Inclusion of voronoi graph in C++:
```C++
#include <voronoi_route_planner/Voronoi_graph.hpp>
```

## RoutePlanner Node

The planner is implemented as a ROS2 service `Global/get_plan` _(cca_interfaces/srv/GlobalPlan)_ that receives a start and end point, providing a sequence (an ordered list) of points in response. These returned points represent the path made of the intervening straight edges between the specified voronoi vertices that should be followed to reach the end from the start.

### Parameters

- **resolution**: The smallest distance that must be sampled from the point cloud to determine occupancy.
- **conflict_radius**: The minimum permissible distance between an edge or vertex of the voronoi graph and a point of the map pointcloud.
- **map_height**: The height at which the map is sampled.
- **floor_height**: The height below which all points must be ignored as the floor.

### Topics
#### Subscriptions

- **map** _(sensor_msgs/msg/PointCloud2)_: A map of the environment.

### Services

- **Global/get_plan** _(cca_interfaces/srv/GlobalPlan)_: A route through the map is planned between the specified points via vertices of the map's voronoi diagram.

## Voronoi_graph.hpp class key members

This class, upon construction, generates a voronoi diagram of the given map. The map is provided as an octree for efficient occupancy querying. Collision-free vertices from any position can be obtained from the member function `Voronoi_graph::edgesFrom` depending on the point passed in:

1. If the queried position passed into the function is not on a vertex of the voronoi diagram, all reachable collision-free vertices are returned.
2. If the queried position passed into the function is a vertex of the voronoi diagram, only the vertices connected to it are returned.
3. In either case, if the point passed as the `end` to the class constructor is accessible via a permissible straight edge, it is returned along with those of the previous conditions.