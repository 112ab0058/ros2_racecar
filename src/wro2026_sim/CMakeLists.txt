cmake_minimum_required(VERSION 3.8)
project(wro2026_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/gazebo_ground_truth_odom.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
