cmake_minimum_required(VERSION 3.5)
project(champ_gazebo)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz_ros2_control REQUIRED)

set(dependencies
  rclcpp
  urdf
  champ
  champ_msgs
  ros_gz_sim
  gz_ros2_control
)

install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION bin
)

# Replaces the disabled Gazebo Classic contact_sensor node below.
install(PROGRAMS
  scripts/foot_contacts_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)

# NOTE: contact_sensor.cpp uses Gazebo Classic APIs and is commented out
# for Ignition Fortress. A new implementation using Ignition transport
# would be needed. For now, contact detection can be handled differently.
# add_executable(contact_sensor src/contact_sensor.cpp)
# target_link_libraries(contact_sensor ${GAZEBO_LIBRARIES})
# ament_target_dependencies(contact_sensor ${dependencies})

# install(TARGETS 
#   contact_sensor
#     DESTINATION lib/${PROJECT_NAME}
# )
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
