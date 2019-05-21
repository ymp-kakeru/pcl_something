#include <iostream>
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h> //octreeの各種テンプレートの使用にはここのマクロ？が必要らしい。Densityだけっぽいけど
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
  
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN>::Ptr PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL>::Ptr PointCloudL;

#define debug() std::cerr << __LINE__ << std::endl;

int main(int argc, char const *argv[])
{
  /* code */
  double resolution_tree = 10.1f;
  std::string FileName = "/home/ymp/catkin_ws/src/pcl_something/table_scene_lms400.pcd";
  pcl::PCDReader reader;  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
  reader.read (FileName, *cloud);  
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*  

  pcl::octree::OctreePointCloudDensity<PointT> octree(resolution_tree);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  pcl::PointCloud<PointT>::iterator it;
  pcl::octree::OctreePointCloudDensity<PointT>::LeafNodeIterator leaf_it(&octree);
  while(*++leaf_it)
  {
    std::cout << leaf_it.operator*() << std::endl;
//    std::cout << octree.getVoxelDensityAtPoint(leaf_it.operator*()) << std::endl ;
//    std::cout << octree.getVoxelDensityAtPoint(*it) << std::endl ;
  }

  return 0;
}