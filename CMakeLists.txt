cmake_minimum_required(VERSION 3.8)
project(mecanum_multi_nav)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch models worlds config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/motion_smoke_test
    scripts/formation_demo
    scripts/trajectory_coordinator
    scripts/synthetic_lidar
    scripts/tf_aggregator
    scripts/robot_markers
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
