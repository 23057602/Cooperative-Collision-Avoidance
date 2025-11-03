cmake_minimum_required(VERSION 3.8)
project(cca_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# transfer dependent directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

#Python executables
install(PROGRAMS
  scripts/Voyager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/VirtualVoyager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/TfAsOdom.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/DemoPlot.py
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
