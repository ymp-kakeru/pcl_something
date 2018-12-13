cmake_minimum_required(VERSION 3.5.1)
project(pcl_something)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED COMPONENTS
	core
	filters
	io
	segmentation
  pcl_conversions
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  tf
  tf_conversions
)

catkin_package(
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
add_executable(planeDetect src/plane_segment.cpp)
add_executable(example src/example.cpp)
add_executable(euclidean_clustering src/euclidean_cluster.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
target_link_libraries(pcl_grid_node ${LIBS})
target_link_libraries(planeDetect ${LIBS})
target_link_libraries(euclidean_clustering ${LIBS})