cmake_minimum_required(VERSION 3.5)
project(kortex_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/test_huge_joint_space_jump.py
  scripts/pick_and_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
