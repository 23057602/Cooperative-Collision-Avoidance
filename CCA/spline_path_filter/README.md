# spline path filter

A package containing a spline-based path filter implemented as a ROS2 Python node.

## Installation & Dependencies
This package depends on:
- rclpy
- cca_interfaces
- spline_works

To install, please copy the directory into the workspace and install with a build tool. This package was produced with a ROS2 toolchain as a C++/CMake package for colcon using `ament_cmake`:
```bash
colcon build --package-select spline_path_filter
```

## Usage
Filter node:
```bash
ros2 run spline_path_filter Path_filter.py
```

## Path_filter.py Node

The filter is implemented as a ROS2 service `Path_filter/filter` _(cca_interfaces/srv/PathFilter)_ that receives a sequence (an ordered list) of points and responds with an ordered list of cubic splines. These returned splines represent the smooth paths fit to the points following them from the first point to the last point.

### Services

- **Path_filter/filter** _(cca_interfaces/srv/PathFilter)_: A smooth curve along the provided points is given.