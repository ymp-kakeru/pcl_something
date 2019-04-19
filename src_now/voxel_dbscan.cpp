#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

// #include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkPolyLine.h> //for drawing graph line 

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

#define debug() std::cerr << __LINE__ << std::endl;

class VoxelDBSCAN()
{
public:
  VoxelDBSCAN();
  void GenerateOctree();
  PointCloudT cloud_in;

private:
  float resolution_tree = 0.05f;
}

VoxelDBSCAN::VoxelDBSCAN():
{

}

void VoxelDBSCAN::GenerateOctree():
{
  pcl::octree::OctreePointCloud octree(resolution_tree);
  octree.setInputCloud(cloud_in);
  //octree.defineBoundingBox();
  octree.addPointsFromInputCloud();


}