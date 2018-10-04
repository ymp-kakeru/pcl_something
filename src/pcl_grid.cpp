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

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/point_cloud.h>

using namespace std;

/*class PCL_Grid{

public:
	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
	void planeDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold);
	void planeRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative);

	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub = nh_.subscribe("hokuyo3d/hokuyo_cloud2",1,cloud_cb);
	ros::Pubisher grided_map = nh_.advertis<sensor_msgs::PointCloud2>("pcl_grid/grid_map",1) ;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

}*/

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
	//////////前処理関数/////////////////////////////////////////////////
	//-VoxelGridによるダウンサンプリング->launchファイル内でnodeletで実行								//
	//-PCL2 > PCLXYZ(平面処理を行うために一応。しなくてもいいのか？？)//
	/////////////////////////////////////////////////////////////////////

	//pcl::PCLPointCloud2::Ptr pcl_pcl2(new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr pcl2_f(new pcl::PCLPointCloud2);

	//pcl_conversions::toPCL(*input,pcl2_f);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pclt_f(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::fromPCLPointCloud2(pcl2_f, *pclt_f);

}
/*
void planeDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double threshold)
{
	//inliers : 平面抽出で得た点群のインデックス inliers->indices[0]
	//coefficients : 三次元平面方程式の未知数？？

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setInputCloud(cloud);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThershold(threshold);
	seg.segment(*inliers, *coefficients); 
}

void planeRemove(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers, bool negative)
{
	pcl::ExtractIndices<pcl::PointXYZ> extract ;
	extract.setInputCloud(cloud); //入力点群
	extract.setIndices(inliers); //指定平面
	extract.setNegative(true); //true:remove Plane , false:remove without Plane
	extract.filter(*cloud); //出力点群
}
void gridPlane()
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "pcl_grid");
	ros::NodeHandle nh_;
	ros::Subscriber cloud_sub = nh_.subscribe("hokuyo3d/hokuyo_cloud2",1,cloud_cb);
	ros::Publisher grided_map = nh_.advertise<sensor_msgs::PointCloud2>("pcl_grid/grid_map",1) ;

	//PCL_Grid();
	ros::spin();
}