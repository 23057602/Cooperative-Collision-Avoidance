cmake_minimum_required(VERSION 3.8)
project(dijkstra_collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cca_interfaces REQUIRED)
find_package(stanley_controller REQUIRED)
find_package(spline_works REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#create library
add_library(${PROJECT_NAME} SHARED
  src/Map_subscriber.cpp
  src/ForeBroadcaster.cpp
  src/Manoeuvre_graph.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs geometry_msgs nav_msgs PCL pcl_conversions pcl_ros cca_interfaces stanley_controller spline_works
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs nav_msgs PCL pcl_conversions pcl_ros cca_interfaces stanley_controller spline_works
)

target_link_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

#Params
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#create executable
add_executable(TrajectoryPrediction src/Trajectory_forecast.cpp)
target_link_libraries (TrajectoryPrediction ${PROJECT_NAME})
ament_target_dependencies(TrajectoryPrediction rclcpp stanley_controller spline_works cca_interfaces)

add_executable(AvoidancePlanner src/LocalPlanner.cpp)
target_link_libraries (AvoidancePlanner ${PROJECT_NAME})
ament_target_dependencies(AvoidancePlanner rclcpp stanley_controller spline_works cca_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
