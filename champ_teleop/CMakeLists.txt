cmake_minimum_required(VERSION 3.5)
project(champ_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  champ_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

