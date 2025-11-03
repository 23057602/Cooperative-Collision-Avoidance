cmake_minimum_required(VERSION 3.8)
project(cca_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cca_interfaces REQUIRED)
find_package(cca_utilities REQUIRED)

#Cpp executables
add_executable(Scout src/Scout.cpp)
ament_target_dependencies(Scout rclcpp Eigen3 cca_interfaces nav_msgs cca_utilities)
add_executable(GlobalPlanner src/GlobalPlanner.cpp)
ament_target_dependencies(GlobalPlanner rclcpp geometry_msgs cca_interfaces cca_utilities)
add_executable(LocalPlanner src/LocalPlanner.cpp)
ament_target_dependencies(LocalPlanner rclcpp cca_interfaces cca_utilities)

install(TARGETS
  Scout
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  GlobalPlanner
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  LocalPlanner
  DESTINATION lib/${PROJECT_NAME}
)

#Python executables
install(PROGRAMS
  scripts/Conductor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/path_filter.py
  DESTINATION lib/${PROJECT_NAME}
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
