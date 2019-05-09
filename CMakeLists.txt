cmake_minimum_required(VERSION 3.5.1)
project(pcl_something)

set(CMAKE_CXX_STANDARD 14)
#set(VTK_DIR )

find_package(PCL REQUIRED COMPONENTS
	core
	filters
	io
	segmentation
  pcl_conversions
  octree
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

find_package(VTK REQUIRED)

catkin_package(
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

include(${VTK_USE_FILE})

set(LIBS
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#[[add_executable(pcl_grid_node src/pcl_grid.cpp)
add_executable(planeDetect src/plane_segment.cpp)
add_executable(example src/example.cpp)
add_executable(euclidean_clustering src/euclidean_cluster.cpp)
add_executable(supervoxel_clustering src/supervoxel_clustering.cpp)
add_executable(euclidean_alone src/euclid_alone.cpp)
]]
add_executable(dbscan src_now/voxel_dbscan.cpp)

#[[target_link_libraries(example ${catkin_LIBRARIES})
target_link_libraries(pcl_grid_node ${LIBS})
target_link_libraries(planeDetect ${LIBS})
target_link_libraries(euclidean_clustering ${LIBS})
target_link_libraries(euclidean_alone ${LIBS})
target_link_libraries(supervoxel_clustering ${LIBS})
]]

target_link_libraries(dbscan &{[LIBS]})