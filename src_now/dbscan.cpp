#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

class DBSCAN
{
public:
  DBSCAN();

  PointCloudT cloud_in;
  PointCloudT cloud_out;

  int N; //neighborhood


}

DBSCAN::DBSCAN():
{

}