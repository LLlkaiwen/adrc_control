cmake_minimum_required(VERSION 2.8.3)
project(adrc_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES pid 
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(pid
  src/pid.cpp
)

add_library(adrc
  src/adrc/adrc_base.cpp
  src/adrc/adrc_firstorder.cpp
  src/adrc/adrc_secondorder.cpp
  src/adrc/adrc_cascade.cpp)

install(TARGETS pid adrc
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)