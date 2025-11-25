# Dijkstra Collision Avoidance

A package containing a local avoidance planner implemented as two ROS2 C++ nodes. It uses the Dijkstra's algorithm to search a tree of evasive manoeuvres for a collision-free route from the initial position at the time of planning along the long-term route for a set amount of time. A class for the manoeuvre graph used in the search is also exposed as library header files by the package.

## Installation & Dependencies
This package depends on:
- rclcpp
- sensor_msgs
- nav_msgs
- PCL
- pcl_conversions
- pcl_ros
- cca_interfaces
- stanley_controller
- spline_works

To install, please copy the directory into the workspace and install with a build tool. This package was produced with a ROS2 toolchain as a C++/CMake package for colcon using `ament_cmake`:
```bash
colcon build --package-select dijkstra_collision_avoidance
```

## Usage
Prediction planning node:
```bash
ros2 run dijkstra_collision_avoidance TrajectoryPrediction
```
Avoidance planning node:
```bash
ros2 run dijkstra_collision_avoidance AvoidancePlanner
```
Inclusion of manoeuvre graph in C++:
```C++
#include <dijkstra_collision_avoidance/Manoeuvre_graph.hpp>
```

## AvoidancePlanner Node

The planner is implemented on a shared ROS2 topic `/predicted_trajectory` _(cca_interfaces/msg/DriveForecast)_ that receives the predicted trajectory of itself and other agents, providing a sequence (an ordered list) of future poses each. These points represent the intentions of each agent, that are compared to predict and resolve collisions cooperatively. Cooperation is performed using a structured exchange (protocol) of formatted messages.

### Parameters

- **resolution**: The smallest distance that must be sampled from the point cloud to determine occupancy.
- **conflict_radius**: The distance presumed to represent the geometry of the vehicle.
- **separation_distance**: The minimum permissible distance between the vehicles.
- **vehicle_id**: Identifies the vehicle in cooperative communication.
- **fixed_frame**: Specifies the map frame.
- **robot_frame**: Specifies the robot's base frame.
- **map_height**: The height at which the map is sampled.
- **floor_height**: The height below which all points must be ignored as the floor.
- **resolution_horizon**: The total amount of time allowed for evasive actions.
- **evasive_step_time**: The length of time allowed for one manoeuvre.
- **evasive_turn_angle**: The change in heading undertaken for evasive purposes.
- **planning_margin**: An error margin added to the conflict radius for planning.
- **replan_timeout**: A debounce time to prevent rapid replanning due to triggering of the planner many times from the same conflict or backlogged messages.

### Topics
#### Subscriptions

- **map** _(sensor_msgs/msg/PointCloud2)_: A map of the environment.
- **plan** _(cca_interfaces/msg/RefSeq)_: The long term route plan control reference.
- **_action/feedback** _(cca_interfaces/action/Drive_FeedbackMessage)_: Guidance controller action server control reference execution feedback.
- **/predicted_trajectory** _(cca_interfaces/msg/DriveForecast)_: trajectory prediction.

#### Publishers

- **resolution** _(cca_interfaces/msg/OverRefSeq)_: Short term avoidance manoeuvre control reference.
- **/predicted_trajectory** _(cca_interfaces/msg/DriveForecast)_: trajectory prediction.

### Services

- **toggle_publishing** _(cca_interfaces/srv/StopPrediction Client)_: Prevents trajectory forecast messages from interfering with collision resolution protocol by temporarily suspending broadcasting.

## TrajectoryPrediction Node

The forecaster/broadcaster is implemented on a shared ROS2 topic `/predicted_trajectory` _(cca_interfaces/msg/DriveForecast)_, generating the predicted trajectory of itself, providing a sequence (an ordered list) of future poses each. These points represent the intentions of each agent.

### Parameters

- **resolution**: The smallest distance that must be sampled from the point cloud to determine occupancy.
- **conflict_radius**: The distance presumed to represent the geometry of the vehicle.
- **separation_distance**: The minimum permissible distance between the vehicles.
- **vehicle_id**: Identifies the vehicle in cooperative communication.
- **fixed_frame**: Specifies the map frame.
- **robot_frame**: Specifies the robot's base frame.
- **map_height**: The height at which the map is sampled.
- **floor_height**: The height below which all points must be ignored as the floor.
- **prediction_horizon**: The amount of time for which a local trajectory is calculated.
- **forecast_frequency**: The frequency with which the predictions are published.

### Topics
#### Subscriptions

- **map** _(sensor_msgs/msg/PointCloud2)_: A map of the environment.
- **plan** _(cca_interfaces/msg/RefSeq)_: The long term route plan control reference.
- **_action/feedback** _(cca_interfaces/action/Drive_FeedbackMessage)_: Guidance controller action server control reference execution feedback.
- **/predicted_trajectory** _(cca_interfaces/msg/DriveForecast)_: trajectory prediction.
- **resolution** _(cca_interfaces/msg/OverRefSeq)_: Short term avoidance manoeuvre control reference.

#### Publishers

- **/predicted_trajectory** _(cca_interfaces/msg/DriveForecast)_: trajectory prediction.

### Services

- **toggle_publishing** _(cca_interfaces/srv/StopPrediction Server)_: Prevents trajectory forecast messages from interfering with collision resolution protocol by temporarily suspending broadcasting.

## Manoeuvre_graph.hpp class key members

The map is provided as an octree for efficient occupancy querying. Collision-free manoeuvre references from the initial position can be obtained from the member function `Fate_graph::edgesFrom`.