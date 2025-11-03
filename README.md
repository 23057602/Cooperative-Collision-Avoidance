# Cooperative-Collision-Avoidance
Codebase for Cooperative Collision Avoidance of the CSIR's Voyager UGV
## ROS2
### Dependencies
Other required packages outside this repository include:
- compile time:
    - Eigen3
    - PCL
    - pcl_ros
    - Boost
    - GSL
    - ROS2:
        - tf2_ros
        - std_msgs
        - nav2_msgs
        - geometry_msgs
        - sensor_msgs
- runtime:
    - rtabmap
    - robot_localization
    - domain_bridge

Install (Assuming [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) is already installed):

1. Update apt
```bash
sudo apt update -y
```

```bash
sudo apt upgrade -y
```
2. Install dependancies
```bash
sudo apt install libpcl-dev ros-$ROS_DISTRO-pcl-ros libboost-all-dev libeigen3-dev libgsl-dev ros-$ROS_DISTRO-rtabmap-ros ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-domain-bridge ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup -y
```
3. Clone the repo
4. Move the cca packages into your work space
5. Build the added packages
6. Source your ROS installation and workspace
7. Run any of the lunches in the cca_launch package

The system can be started on the voyager with the command:
```bash
ros2 launch cca_launch sys.launch.py
```
A virtual agent with the configurable vehicle_id (defaults to param file if unspecified) can be started with the command:
```bash
ros2 launch cca_launch virtual.launch.py vehicle_id:=agent_name
```
Please note that the nodes will launch in the ROS_DOMAIN of the shell. A new domain must be exported to launch in another domain.
```bash
export ROS_DOMAIN_ID=2; ros2 launch cca_launch virtual.launch.py vehicle_id:=vehicle_0
```

### Packages
Packages in order of dependancy:
- cca_interfaces
    - features definitions for the message types used
- cca_utilities
    - contains library code and tools used by nodes
- cca_nodes
    - contains the system's executable nodes
- cca_launch
    - instantiates and configures the system runtime/application