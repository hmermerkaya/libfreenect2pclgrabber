#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/radius_outlier_removal.h>

 float default_rad    = 0.03f;
int    default_min = 50;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
int
main (int argc, char** argv)
{
/*  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> (argv[1], *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZRGB> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

*/



  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  float rad = default_rad;
  int min= default_min;
  parse_argument (argc, argv, "-r", rad);
  parse_argument (argc, argv, "-min", min);


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> (argv[p_file_indices[0]], *cloud);



  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (cloud);
  radius_outlier_removal.setRadiusSearch (rad);
  radius_outlier_removal.setMinNeighborsInRadius (min);
  radius_outlier_removal.filter (*cloud_filtered);



  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> (argv[p_file_indices[1]], *cloud_filtered, false);















  return 0;
}
