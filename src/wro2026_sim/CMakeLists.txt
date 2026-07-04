cmake_minimum_required(VERSION 3.8)
project(wro2026_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds launch models config maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/gazebo_ground_truth_odom.py
  scripts/line_detector.py
  scripts/line_follower.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
