#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

// #include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
//#include <pcl/octree/octree_search.h>
//#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_density.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkPolyLine.h> //for drawing graph line 

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN>::Ptr PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL>::Ptr PointCloudL;

#define debug() std::cerr << __LINE__ << std::endl;

class VoxelDBSCAN
{
public:
  VoxelDBSCAN();
  void GenerateOctree();
  PointCloudT cloud_in;

  const double resolution_tree ;
private:

};

VoxelDBSCAN::VoxelDBSCAN():
  resolution_tree{10.0f}
{

}

void VoxelDBSCAN::GenerateOctree()
{
  /* generate sample point cloud data */
  VoxelDBSCAN::cloud_in -> width = 1000;
  VoxelDBSCAN::cloud_in -> height = 1;
  VoxelDBSCAN::cloud_in -> points.resize(cloud_in->width * cloud_in->height);

  for(size_t i=0; i < cloud_in->points.size();++i)
  {
    cloud_in->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }
  /* make octree  */
  pcl::octree::OctreePointCloudDensity<PointT> octree(resolution_tree);
  std::vector<pcl::PointIndices> ite_octree ;
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  std::cout << octree.getVoxelDensityAtPoint() << std::endl ;
  /*for(std::vector<pcl::PointIndices>::const_iterator it=octree.leaf_begin(); it!=octree.leaf_end(); ++it)
  {
     std::cout << octree.getVoxelDensityAtPoint(octree.leaf_begin()) << std::endl ;
  }*/
}


int main(int argc, char const *argv[])
{
  /* code */
  VoxelDBSCAN dbscan;
  dbscan.GenerateOctree();
  return 0;
}