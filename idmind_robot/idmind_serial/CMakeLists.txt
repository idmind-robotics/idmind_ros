cmake_minimum_required(VERSION 2.8.3)
project(idmind_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(idmind_serial src/${PROJECT_NAME}/idmind_serial.cpp)

add_dependencies(idmind_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_serial ${catkin_LIBRARIES})
