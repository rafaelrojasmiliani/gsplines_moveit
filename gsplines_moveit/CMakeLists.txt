cmake_minimum_required(VERSION 3.0.2)
project(gsplines_moveit)

find_package(
  catkin REQUIRED COMPONENTS gsplines_msgs gsplines_ros moveit_core
                             moveit_ros_planning_interface roscpp rospy)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include/
  LIBRARIES
  gsplines_moveit
  CATKIN_DEPENDS
  roscpp
  rospy
  gsplines_ros
  gsplines_msgs)

add_library(gsplines_moveit src/gsplines_moveit.cpp)
include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} include/)
target_link_libraries(gsplines_moveit ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

install(
  TARGETS gsplines_moveit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/gsplines_moveit
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/..)
