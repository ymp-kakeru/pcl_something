#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZRGB PointTrgb;

#define debug() std::cerr << __LINE__ << std::endl;

class EuclideanClustering
{
public:
  EuclideanClustering();

  sensor_msgs::PointCloud2 cloud_clustered;

  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointTrgb> cloud_cluster;
  pcl::EuclideanClusterExtraction<PointT> ec;
  pcl::visualization::CloudViewer viewer;  
  pcl::PCDWriter writer;  

  //pcl::search::KdTree<PointT>::Ptr tree  ;

  // pcl::search::KdTree<pcl::PointXYZ> tree;  
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointIndices>::const_iterator iterator;
  std::vector<int>::const_iterator pit;

private:
  void euclidean_clustering(const sensor_msgs::PointCloud2ConstPtr& input);
  ros::NodeHandle nh_;
  ros::Subscriber map_sub;
  ros::Publisher cluster_pub;

  double tolerance; //許容誤差 最小・最大クラスタサイズ
  int min_cluster,max_cluster;
};

EuclideanClustering::EuclideanClustering():
  tolerance{0.2},min_cluster{100},max_cluster{25000},viewer{"cluster viewer"}
{
//  debug();
  nh_.getParam("/euclidean_clustering/tolerance", tolerance);
  nh_.getParam("/euclidean_clustering/min_cluster", min_cluster);
  nh_.getParam("/euclidean_clustering/max_cluster", max_cluster);
  map_sub = nh_.subscribe("/cloud_seg",1,&EuclideanClustering::euclidean_clustering,this);
  cluster_pub = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_clustered",100);
}

void EuclideanClustering::euclidean_clustering(const sensor_msgs::PointCloud2ConstPtr& input)
{
//  debug();
  pcl::fromROSMsg (*input, cloud);
  pcl::search::KdTree<PointT>::Ptr tree  (new pcl::search::KdTree<PointT>);

  tree->setInputCloud (cloud.makeShared());

  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (min_cluster);
  ec.setMaxClusterSize (max_cluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud.makeShared());
  ec.extract (cluster_indices);

  int j = 0;
  float colors[6][3] ={{255, 0, 0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};  
  pcl::copyPointCloud(cloud, cloud_cluster);  //(in,out)
//  debug();
  for(iterator = cluster_indices.begin(); iterator != cluster_indices.end(); ++iterator)
  {
    cloud_cluster.points[*pit].r = colors[j%6][0];  
    cloud_cluster.points[*pit].g = colors[j%6][1];  
    cloud_cluster.points[*pit].b = colors[j%6][2];  
  }
//  debug();
  std::cout << "PointCloud representing the Cluster: " << cloud_cluster.points.size () << " data points." << std::endl;  
  std::stringstream ss;  
  ss << "cloud_cluster_" << j << ".pcd";  
  writer.write<pcl::PointXYZRGB> (ss.str (), cloud_cluster, false);
  viewer.showCloud (cloud_cluster.makeShared());   
  pcl::toROSMsg(cloud_cluster,cloud_clustered);
  cluster_pub.publish (cloud_clustered);
}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "euclidean_cluster_extraction");
  EuclideanClustering euclidean_clustering;
  ros::spin();
}