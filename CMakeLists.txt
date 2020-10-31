cmake_minimum_required(VERSION 2.4)

project(action_package_debian_ros)


find_package(catkin)

catkin_package(
  LIBRARIES sample_lib
  CATKIN_DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sample_lib
  src/main.cpp
)


install(TARGETS sample_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )