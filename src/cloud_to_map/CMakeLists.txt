cmake_minimum_required(VERSION 2.8.3)
project(cloud_to_map)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  nav_msgs
  ##pcl
  pcl_ros
  roscpp
)
find_package(OpenCV 3 REQUIRED)
#find_package(CUDA REQUIRED)
#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/cloud_to_map_node.cfg
)


find_package(PCL 1.8 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cloud_to_map
#  CATKIN_DEPENDS dynamic_reconfigure nav_msgs pcl pcl_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  #${CUDA_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(cloud_to_map_node src/cloud_to_map_node.cpp)
add_dependencies(cloud_to_map_node ${PROJECT_NAME}_gencfg)
target_link_libraries(cloud_to_map_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


