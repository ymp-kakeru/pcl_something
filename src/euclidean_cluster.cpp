#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;

class EuclideanClustering
{
public:
  EuclideanClustering();

private:
  void euclidean_clustering(const sensor_msgs::PointCloud2ConstPtr& input);
  ros::NodeHandle nh_;
  ros::Subscriber map_sub;

  pcl::PointCloud<PointT> cloud;
  pcl::PointCloud<PointT> cloud_cluster;
  pcl::EuclideanClusterExtraction<PointT> ec ;
  pcl::search::KdTree<PointT> tree;

  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointIndices>::const_iterator v
  std::vector<int>::const_iterator pit;

  double tolerance, min_cluster, max_cluster; //許容誤差 最小・最大クラスタサイズ
};

EuclideanClustering::EuclideanClustering():
  tolerance{0.05},min_cluster{100},max_cluster{10000}
{
  nh_.getParam("/euclidean_clustering/tolerance", tolerance);
  nh_.getParam("/euclidean_clustering/min_cluster", min_cluster);
  nh_.getParam("/euclidean_clustering/max_cluster", max_cluster);
  map_sub = nh_.subscribe("cloud_seg",1,euclidean_clustering);
}

void euclidean_clustering(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::fromROSMsg (*inpu, cloud);
  tree->setInputCloud (cloud);

  ec.setClusterTolerance (tolerance);
  ec.setMinClusterSize (min_cluster);
  ec.setMaxClusterSize (max_cluster);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud.makeShared());
  ec.extract (cluster_indices);

  int j = 0;
  for (v = cluster_indices.begin(); v != cluster_indices.end(); ++v)
  {
    for(pit = v->indices.begin(); pit != v->indices.end(); ++pit)
    {
      cloud_cluster->points.push_buck (cloud->points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std :: cout << "PointCloud representing the Cluster: " << cloud_cluster -> points . size () << " data points." << std :: endl ;
      std :: stringstream ss ;
      ss << "cloud_cluster_" << j << ".pcd" ;
      writer . write < pcl :: PointXYZ > ( ss . str (), * cloud_cluster , false ); //*
      j ++ ;
    }
  }
}

int main(int argc, char const *argv[])
{
  ros::init (int argc, char** argv);
  EuclideanClustering euclidean_clustering;
  ros::spin();
}