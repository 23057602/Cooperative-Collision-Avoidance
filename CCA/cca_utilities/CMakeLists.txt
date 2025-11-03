cmake_minimum_required(VERSION 3.8)
project(cca_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(GSL REQUIRED)
find_package(Boost 1.66 REQUIRED)

find_package(cca_interfaces REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSL_INCLUDE_DIRS}
  )
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${GSL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
ament_python_install_package(${PROJECT_NAME})

#create library
add_library(${PROJECT_NAME} SHARED
  src/Conductor_model.cpp
  src/Fate_graph.cpp
  src/ForeBroadcaster.cpp
  src/Map_subscriber.cpp
  src/Spline_works.cpp
  src/Voronoi_graph.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs PCL Eigen3 pcl_conversions pcl_ros Boost geometry_msgs nav_msgs cca_interfaces GSL
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp sensor_msgs PCL Eigen3 pcl_conversions pcl_ros Boost geometry_msgs nav_msgs cca_interfaces GSL
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSL_LIBRARIES}
)

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
