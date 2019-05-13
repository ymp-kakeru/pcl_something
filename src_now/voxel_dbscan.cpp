#include <iostream>
//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_impl.h> //octreeの各種テンプレートの使用にはここのマクロ？が必要らしい。Densityだけっぽいけど
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_nodes.h>
#include <pcl/octree/octree_iterator.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vtkPolyLine.h> //for drawing graph line 
  
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
  const PointCloudT cloud_in;

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
  cloud_in -> width = 1000;
  cloud_in -> height = 1;
  cloud_in -> points.resize(cloud_in->width * cloud_in->height);

  for(size_t i=0; i < cloud_in->points.size();++i)
  {
    cloud_in->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
  }

  /* make octree  */
  pcl::octree::OctreePointCloudDensity<PointT> octree(resolution_tree);
  octree.setInputCloud(cloud_in);
  octree.addPointsFromInputCloud();

  /* octree leaf iterator */
  std::vector<int> indexVector;
//  pcl::octree::OctreePointCloud<PointT>::LeafNode leaf_it(octree);
/*  while(leaf_it.operator++())
  {
//    pcl::octree::OctreePointCloud<PointT>::LeafNode *node = *leaf_it;
//    std::cout << octree.getVoxelDensityAtPoint(leaf_it.operator*()) << std::endl ;
//    std::cout << octree.getVoxelDensityAtPoint(PointT(1,1,1)) << std::endl ;
  }*/

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