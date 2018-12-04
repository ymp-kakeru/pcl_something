#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/point_cloud.h>

typedef pcl::PointXYZ PointT;

class PCL_segmentation
{
public:
  PCL_segmentation();

  void planeDetect(pcl::PointCloud<PointT>::Ptr cloud, double threshold);
  void planeRemove(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);
  pcl::PointCloud<PointT>::Ptr cloud(pcl::PointCloud<PointT>);
  pcl::ModelCoefficients::Ptr coefficients(pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(pcl::PointIndices);

private:
void cloud_Cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //rosメッセージであるPointCloud2型からpclで使用される型への変換
  pcl::fromROSMsg (*input, &cloud);
  //voxelgridによるダウンサンプリング
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCLoud(input);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(*cloud);
  //RANSACを用いて平面検出および平面除去
//  planeDetect(&cloud, threshold);
//  planeRemove(&cloud, inliers, negative);
}

  double threshold;
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub ;
//  ros::Publishr map_pub;
};

PCL_segmentation::PCL_segmentation():
  threshold{0.1}
{
  cloud_sub = nh_.subscribe("/input_cloud",10,&PCL_segmentation::cloud_Cb,this);
  //map_pub = nh_.advertise<>();
}


/*void planeDetect(pcl::PointCloud<PointT>::Ptr cloud, double threshold)
{
  //inliers : 平面抽出で得た点群のインデックス inliers->indices[0]
  //coefficients : 三次元平面方程式の未知数？？
  inliers = &PCL_segmentation::inliers;
  coefficients = &PCL_segmentation::coefficients;

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(cloud);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(threshold);
  seg.segment(*inliers, *coefficients); 

   if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  std::cerr << "Model coefficients:" << coefficients->values[0] << "x"
                                     << coefficients->values[1] << "y"
                                     << coefficients->values[2] << "z"
                                     << coefficients->values[3] << std::endl ;

  

}

void planeRemove(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative)
{

  inliers = &PCL_segmentation::inliers;
  pcl::ExtractIndices<PointT> extract ;
  extract.setInputCloud(cloud); //入力点群
  extract.setIndices(inliers); //指定平面
  extract.setNegative(true); //true:remove Plane , false:remove without Plane
  extract.filter(*cloud); //出力点群
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "plane_segment");
  //PCL_segmentation pcl_segmentation;
  ros::spin();
}
