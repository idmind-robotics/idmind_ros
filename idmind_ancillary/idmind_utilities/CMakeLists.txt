cmake_minimum_required(VERSION 2.8.3)
project(idmind_utilities)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  PoseInit.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(wheel_speed_calibration src/wheel_speed_calibration.cpp)
add_executable(pose_initializer src/pose_initializer.cpp)

add_dependencies(wheel_speed_calibration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_initializer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(wheel_speed_calibration ${catkin_LIBRARIES})
target_link_libraries(pose_initializer ${catkin_LIBRARIES})
