#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
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

class VoxelDBSCAN
{
public:
  VoxelDBSCAN();
  void GenerateOctree();

  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in;
  std::string FileName ;
  const double resolution_tree ;

private:

};

VoxelDBSCAN::VoxelDBSCAN():
  resolution_tree{10.0f},FileName{"/home/ymp/catkin_ws/src/pcl_something/hill_map.pcd.pcd"}
{
  /* include point cloud data */
  debug();
  pcl::PCDReader reader;
  reader.read(FileName,*cloud_in);
  debug();
  std::cout << "PointCloud before filtering has: " << cloud_in->points.size () << " data points." << std::endl; //*  
  std::cout << "get PointCloud" << std::endl;

}

void VoxelDBSCAN::GenerateOctree()
{
  /* make octree  */
/*  pcl::octree::OctreePointCloudDensity<PointT> octree(resolution_tree);
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();
*/
  /* octree leaf iterator */
//  std::vector<int> indexVector;
/*  pcl::PointCloud<PointT>::iterator it;
  pcl::octree::OctreePointCloudDensity<PointT>::LeafNodeIterator leaf_it(&octree);
  while(*++leaf_it)
  {
//    std::cout << leaf_it.operator*() << std::endl;
//    std::cout << octree.getVoxelDensityAtPoint(leaf_it.operator*()) << std::endl ;
//    std::cout << octree.getVoxelDensityAtPoint(PointT(1,1,1)) << std::endl ;
  }
*/
/*  for(pcl::octree::OctreePointCloud<PointT>:: node=octree.leaf_begin(); node!=octree.leaf_end(); ++node)
  {
     std::cout << octree.getVoxelDensityAtPoint(it) << std::endl ;
  }*/

//  std::cout << octree.getResolution() << std::endl ;
}


int main(int argc, char const *argv[])
{
  /* code */
  VoxelDBSCAN dbscan;
  dbscan.GenerateOctree();
  return 0;
}