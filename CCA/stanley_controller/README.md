# stanley controller

A package containing a modified stanley controller implemented as a ROS2 Python node. A class for kinematic trajectory prediction are also exposed as library header files.

## Installation & Dependencies
This package depends on:
- rclcpp
- rclpy
- numpy
- geometry_msgs
- nav_msgs
- cca_interfaces
- spline_works

To install, please copy the directory into the workspace and install with a build tool. This package was produced with a ROS2 toolchain as a C++/CMake package for colcon using `ament_cmake`:
```bash
colcon build --package-select stanley_controller
```

## Usage
Controller node:
```bash
ros2 run stanley_controller Controller.py
```
Inclusion of controller model in C++:
```C++
#include <stanley_controller/Controller_model.hpp>
```

## Controller.py Node

The controller is implemented as a ROS2 action server `Controller_Server/follow_reference` _(cca_interfaces/action/Drive)_ that receives a long term motion reference, offers progress feedback during execution, providing a log of execution on completion.

### Parameters

- **controller_frequency**: The frequency at which the control law will be evaluated and published.
- **controller_error_gain**: Stanley controller cross-track sensitivity gain.
- **controller_speed_gain**: Stanley controller speed sensitivity gain.
- **controller_follow_gain**: In/Along-track controller reference lead/lag sensitivity gain.
- **top_speed**: In/Along-track controller maximum catch up speed.
- **crawl_speed**: In/Along-track controller minimum speed allowed while waiting for the progress reference to catch up.
- **steering_limit**: Maximum steering angle in radians.
- **vehicle_base**: Distance between the front and back wheels. This used to convert the control law's steering angle into an angular rate.

### Topics
#### Subscriptions

- **pose** _(nav_msgs/msg/Odometry)_: The current position of the vehicle.
- **ovr** _(cca_interfaces/msg/OverRefSeq)_: Local path reference that will temporarily _override_ the reference plan requested to through the action server.

#### Publishers

- **cmd_vel** _(geometry_msgs/msg/Twist)_: The linear and angular rate needed to follow the reference.
- **ctrl_st** _(cca_interfaces/msg/DriveState)_: Exposes information about the controller state evaluated during the current iteration.

## Controller_model.hpp class key members
This class iteratively models the controlled ideal kinematics of the controller. It is referred to as a "conductor" instead of just a controller because it also includes the node's motion reference switching logic, a step up from just a controller.

1. **Conductor_model::setInit(...)**: Sets the model's starting/initial state.
2. **Conductor_model::initialize()**: Sets the model's current state to the starting/initial state given.
3. **Conductor_model::runStepNext()**: Runs a single controller iteration, updating the model's predicted next pose, progress, etc.
3. **Conductor_model::getNextTwist()**: Returns the control law Twist command of the current time step.