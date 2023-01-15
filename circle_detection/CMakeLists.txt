cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(circle_detection)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES circle_detection
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # include
)

add_executable(circle_detection src/circle_detection.cpp)

target_link_libraries(circle_detection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}
)

install(TARGETS circle_detection
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

