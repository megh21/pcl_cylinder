# cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# project(cylinder_segmentation)

# find_package(PCL 1.2 REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${catkin_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# # link_directories(${catkin_LIBRARY_DIRS})

# add_definitions(${PCL_DEFINITIONS})
# # add_definitions(${catkin_DEFINITIONS})
# set (LIBS ${catkin_LIBRARIES} ${PCL_LIBRARIES})
# FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
# FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

# add_executable (cylinder_segmentation circle2d_det.cpp)
# target_link_libraries (cylinder_segmentation ${LIBS}  )
# # target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES})


cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cylinder_segmentation)

find_package(catkin REQUIRED COMPONENTS
    # ros
    pcl_conversions
    pcl_ros
    std_msgs
    roscpp
    rospy
    geometry_msgs
    message_generation
)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

add_executable (cylinder_segmentation src/cylinder_segmentation_base.cpp)

target_link_libraries(cylinder_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
