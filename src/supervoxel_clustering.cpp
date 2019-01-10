#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/console.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h> //for drawing graph line 

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PointCloudN;
typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointT> PointCloudL;

void addSupervoxelConnectionsToViewer(PointT & supervoxel_center , PointCloudT & adjacent_supervoxel_centers,
                                      std::string supervoxel_name, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);

int main(int argc, char** argv)
{
  if(argc<2){
  std::cerr << "Syntax is: pcd-file" << "\n" 
            << "--NT disables the single cloud transform" << "\n" //シングルビュー変換を無効にする
            << "-v : voxel resolution" << "\n"                   //ボクセルサイズ．オクトツリー構造のリーフサイズを決定 [m]
            << "-s : seed resolution" << "\n"                    //シードサイズ．スーパーボクセルの大きさ
            << "-c : color weight" << "\n"                       //色の重み．どのくらいの色がスーパーボクセルの形状に影響するか
            << "-z : spatial weight" << "\n"                     //空間項の重み．値を大きくするとボクセルが規則的な形状になる．
            << "-n : normal weight" << << std::endl;             //法線の重み．サーフェス法線がどれだけボクセル形状に影響するか
  return(1);
  }
/*****************************************/
/****** Load PCD and Set Parameters ******/
/*****************************************/
  PointCloudT::Ptr cloud = boost::shared_ptr<PointCloudT>(new PointCloudT());
  std::cerr << "Loading PCD data...\n" << std::endl;
  if ( pcl::io::loadPCDFile <PointT>(argv[ 1 ],*cloud))
  {
    pcl::console::print_error("Error loading cloud file! \n");
    return (1);
  }
  bool disabele_transform = pcl::console::find_switch(argc,argv,"--NT");
  
  float voxel_resolution = 0.010f;
  bool voxel_res_specified = pcl::console::find_switch(argc,argv,"-v");
  if(voxel_res_specified) pcl::console::parse(argc,argv,"-v",voxel_resolution);

  float seed_resolution = 0.1f;
  if(pcl::console::find_switch(argc,argv,"-s")) pcl::console::parse(argc,argv,"-s",seed_resolution);

  float color_importance = 0.010f;
  if(pcl::console::find_switch(argc,argv,"-c")) pcl::console::parse(argc,argv,"-c",color_importance);

  float spatial_importance = 0.4f;
  if(pcl::console::find_switch(argc,argv,"-z")) pcl::console::parse(argc,argv,"-z",spatial_importance);

  float normal_importance = 1.0f;
  if(pcl::console::find_switch(argc,argv,"-n")) pcl::console::parse(argc,argv,"-n",normal_importance);

/***************************************/
/********* SuperVoxel Clustering *******/
/***************************************/
  pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
  if(disabele_transform)
  super.setUseSingleCameraTransform(false);
  super.setInputCloud(cloud);
  super.setColorImportance(color_importance);
  super.setSpatialImportance(spatial_importance);
  super.setNormalImportance(normal_importance);

  std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

  std::cerr << "Extracting SuperVoxels " << std::endl;
  super.extract(supervoxel_clusters);
  std::cerr << "Found SuperVoxels :: " << supervoxel_clusters.size() << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
  viewer -> setBackgroundColor(0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud(); //オクトツリーから出力されたセントロイド
  viewer -> addPointCloud(voxel_centroid_cloud, "voxel centroids");
  viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "voxel centroids");
  viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.95, "voxel centroids");

  PointCloudL::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud();
  viewer -> addPointCloud(labeled_voxel_cloud, "labeled voxels");
  viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "labeled voxels");

  PointCloudN::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud(supervoxel_clusters);
  viewer -> addPointCloudNormals<PointN>(sv_normal_cloud, 1, 0.05f, "supervoxel normals");

  std::cerr << "Getting Supervoxel adjacency" << std::endl;
  std::multimap <uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency(supervoxel_adjacency); //スーパーボクセル隣接リストの抽出

  //マルチマップ処理を繰り返して、各スーパーボクセルの隣接ボクセルの重心の点群を作成する
  std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
  for(;label_itr != supervoxel_adjacency.end();)
  {
    //First get the label
    uint32_t supervoxel_label = label_itr -> first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);
    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
    for(; adjacent_itr != supervoxel_adjacency.equal_range(supervoxel_label).second ; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr -> second);
      adjacent_supervoxel_centers.push_back(neighbor_supervoxel -> centroid_);
    }
    //Now we make a name for this polygon
    std :: stringstream ss ;
    ss << "supervoxel_" << supervoxel_label ;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer ( supervoxel -> centroid_ , adjacent_supervoxel_centers , ss . str (), viewer );
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency . upper_bound ( supervoxel_label );
  }
  while ( ! viewer -> wasStopped ())
  {
    viewer -> spinOnce ( 100 );
  }
  return ( 0 );
}

void addSupervoxelConnectionsToViewer(PointT & supervoxel_center , PointCloudT & adjacent_supervoxel_centers,
                                      std::string supervoxel_name, boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer < vtkPoints > points = vtkSmartPointer < vtkPoints >:: New ();
  vtkSmartPointer < vtkCellArray > cells = vtkSmartPointer < vtkCellArray >:: New ();
  vtkSmartPointer < vtkPolyLine > polyLine = vtkSmartPointer < vtkPolyLine >:: New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT :: iterator adjacent_itr = adjacent_supervoxel_centers . begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers . end (); ++ adjacent_itr )
  {
    points -> InsertNextPoint ( supervoxel_center . data );
    points -> InsertNextPoint ( adjacent_itr -> data );
  }
  // Create a polydata to store everything in
  vtkSmartPointer < vtkPolyData > polyData = vtkSmartPointer < vtkPolyData >:: New ();
  // Add the points to the dataset
  polyData -> SetPoints ( points );
  polyLine -> GetPointIds  () -> SetNumberOfIds ( points -> GetNumberOfPoints ());
  for ( unsigned int i = 0 ; i < points -> GetNumberOfPoints (); i ++ )
    polyLine -> GetPointIds () -> SetId ( i , i );
  cells -> InsertNextCell ( polyLine );
  // Add the lines to the dataset
  polyData -> SetLines ( cells );
  viewer -> addModelFromPolyData ( polyData , supervoxel_name );
}
