#define PCL_NO_PRECOMPILE


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


std::string  default_axis="z";
float default_min    = -100;
float     default_max = 100;

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;



struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float normal_x;
    float normal_y;
    float normal_z;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
)


boost::shared_ptr<pcl::PointCloud<MyPointType>>
thresholdDepth (const boost::shared_ptr<pcl::PointCloud<MyPointType>> & input,  float min_depth, float max_depth, std::string  axis="z")
{
 // std::vector<int> mapping;
  // pcl::removeNaNFromPointCloud(*input, *input, mapping);

  pcl::PassThrough<MyPointType> pass_through;
  pass_through.setInputCloud (input);
  pass_through.setFilterFieldName (axis);
  pass_through.setKeepOrganized (false); 
  pass_through.setFilterLimits (min_depth, max_depth);
  boost::shared_ptr<pcl::PointCloud<MyPointType>> thresholded (new pcl::PointCloud<MyPointType>);
  pass_through.filter (*thresholded);

  return (thresholded);
}




int
main (int argc, char** argv)
{
/*  pcl::PointCloud<MyPointTypeRGB>::Ptr cloud (new pcl::PointCloud<MyPointTypeRGB>);
  pcl::PointCloud<MyPointTypeRGB>::Ptr cloud_filtered (new pcl::PointCloud<MyPointTypeRGB>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<MyPointTypeRGB> (argv[1], *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<MyPointTypeRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<MyPointTypeRGB> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<MyPointTypeRGB> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

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
  std::string axis = default_axis;
  float min = default_min;
  float max = default_max;
  parse_argument (argc, argv, "-field", axis);
  parse_argument (argc, argv, "-min", min);
  parse_argument (argc, argv, "-max", max);

 pcl::PointCloud<MyPointType>::Ptr cloud (new pcl::PointCloud<MyPointType>);
  pcl::PointCloud<MyPointType>::Ptr cloud_filtered (new pcl::PointCloud<MyPointType>);
 // pcl::PointCloud<MyPointTypeRGB>::Ptr cloud (new pcl::PointCloud<MyPointTypeRGB>);
  //pcl::PointCloud<MyPointTypeRGB>::Ptr cloud_filtered (new pcl::PointCloud<MyPointTypeRGB>);


  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<MyPointType> (argv[p_file_indices[0]], *cloud);

  cloud_filtered=thresholdDepth(cloud, min, max, axis);


  pcl::PCDWriter writer;
  writer.write<MyPointType> (argv[p_file_indices[1]], *cloud_filtered, true);















  return 0;
}
