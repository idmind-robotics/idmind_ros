cmake_minimum_required(VERSION 2.8.3)
project(idmind_interaction)

find_package(catkin REQUIRED COMPONENTS
  idmind_serial
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Projector.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(idmind_interaction src/idmind_interaction.cpp)
add_executable(mouth_encoder src/mouth_encoder.cpp)

add_dependencies(idmind_interaction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(mouth_encoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_interaction idmind_serial ${catkin_LIBRARIES})
target_link_libraries(mouth_encoder ${catkin_LIBRARIES})
