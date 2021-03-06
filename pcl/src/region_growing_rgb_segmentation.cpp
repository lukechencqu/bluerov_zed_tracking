#include <iostream>
#include <thread>
#include <vector>
#include <stdlib.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

using namespace std::chrono_literals;
using namespace std;

int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
//   if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/home/chenlu/log/initial_RGB.pcd", *cloud) == -1 )
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (argv[1], *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

    // https://pointclouds.org/documentation/classpcl_1_1_region_growing_r_g_b.html#details
  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  if(argc == 6){
    reg.setDistanceThreshold (stof(argv[2]));
    reg.setPointColorThreshold (stof(argv[3]));
    reg.setRegionColorThreshold (stof(argv[4]));
    reg.setMinClusterSize (stof(argv[5]));
  }else{
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);
  }



  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);



  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText ("raw cloud", 10, 10, "raw cloud", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0., 0., 0., v2);
    viewer->addText ("segmentated cloud", 10, 10, "segmentated cloud", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (colored_cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb2, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    viewer->addCoordinateSystem (1.0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }


  return (0);
}