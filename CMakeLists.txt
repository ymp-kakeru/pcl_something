cmake_minimum_required(VERSION 3.5.1)
project(pcl_something)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED COMPONENTS
	core
	features
	filters
	io
	segmentation
	surface
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  pcl_msgs
  tf
  tf_conversions
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

set(LIBS
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(pcl_grid_node src/pcl_grid.cpp)
add_executable(planeDetect_node src/plane_segment.cpp)
add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
target_link_libraries(pcl_grid_node ${LIBS})
target_link_libraries(planeDetect_node ${LIBS})