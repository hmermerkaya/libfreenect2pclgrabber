/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>


#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_polynomial_order = 2;
bool default_use_polynomial_fit = true;
double default_search_radius = 0.0,
    default_sqr_gauss_param = 0.0;

    int depth = 8;
int solver_divide = 8;
int iso_divide = 8;
float point_weight = 4.0f;


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>

downsample (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
 & input, float leaf_size)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud (input);
  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
 downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
  voxel_grid.filter (*downsampled);

  return (downsampled);
}


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>
removeOutliers (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>  & input, float radius, int min_neighbors)
{
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> radius_outlier_removal;
  radius_outlier_removal.setInputCloud (input);
  radius_outlier_removal.setRadiusSearch (radius);
  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> inliers (new pcl::PointCloud<pcl::PointXYZRGB>);
  radius_outlier_removal.filter (*inliers);

  return (inliers);
}

void
clusterObjects (const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>  & input, 
                float cluster_tolerance, int min_cluster_size, int max_cluster_size,
                std::vector<pcl::PointIndices> & cluster_indices_out)
{  
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);

  ec.setInputCloud (input);
  ec.extract (cluster_indices_out);
}

void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X          = sphere radius to be used for finding the k-nearest neighbors used for fitting (default: ");
  print_value ("%f", default_search_radius); print_info (")\n");
  print_info ("                     -sqr_gauss_param X = parameter used for the distance based weighting of neighbors (recommended = search_radius^2) (default: ");
  print_value ("%f", default_sqr_gauss_param); print_info (")\n");
  print_info ("                     -use_polynomial_fit X = decides whether the surface and normal are approximated using a polynomial or only via tangent estimation (default: ");
  print_value ("%d", default_use_polynomial_fit); print_info (")\n");
  print_info ("                     -polynomial_order X = order of the polynomial to be fit (implicitly, use_polynomial_fit = 1) (default: ");
  print_value ("%d", default_polynomial_order); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input, pcl::PCLPointCloud2 &output,
         double search_radius, bool sqr_gauss_param_set, double sqr_gauss_param,
         bool use_polynomial_fit, int polynomial_order)
{

  PointCloud<PointXYZRGB>::Ptr xyz_cloud_pre (new pcl::PointCloud<PointXYZRGB> ()),
      xyz_cloud (new pcl::PointCloud<PointXYZRGB> ());
  fromPCLPointCloud2 (*input, *xyz_cloud_pre);

  // Filter the NaNs from the cloud
  for (size_t i = 0; i < xyz_cloud_pre->size (); ++i)
    if (pcl_isfinite (xyz_cloud_pre->points[i].x))
      xyz_cloud->push_back (xyz_cloud_pre->points[i]);
  xyz_cloud->header = xyz_cloud_pre->header;
  xyz_cloud->height = 1;
  xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->size ());
  xyz_cloud->is_dense = false;
  
  
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new PointCloud<PointXYZRGB>());
  pcl::PassThrough<pcl::PointXYZRGB> filter;

  filter.setFilterFieldName ("z");
  filter.setFilterLimits (0.8, 2.5);
  filter.setInputCloud(xyz_cloud);
  filter.filter(*filtered);
  std::cout << "passthrough filter complete" << std::endl;

  filtered=removeOutliers(filtered, 0.02,30);
  filtered=downsample(filtered,0.006);


    std::vector<pcl::PointIndices> filtered_cluster_indices;


   clusterObjects (filtered, 0.05 , 8000, 250000, filtered_cluster_indices);
   pcl::console::print_info ("Found %lu clusters\n", filtered_cluster_indices.size ());

    PointCloud<PointXYZRGB>::Ptr  temp_cloud (new PointCloud<PointXYZRGB>());
   pcl::copyPointCloud (*filtered, filtered_cluster_indices[0], *temp_cloud);
   filtered = temp_cloud;

  PointCloud< PointXYZRGBNormal>::Ptr xyz_cloud_smoothed (new PointCloud<PointXYZRGBNormal> ());

  MovingLeastSquares<PointXYZRGB,  PointXYZRGBNormal> mls;
  mls.setInputCloud (filtered);
  mls.setSearchRadius (search_radius);
  if (sqr_gauss_param_set) mls.setSqrGaussParam (sqr_gauss_param);
  mls.setPolynomialFit (use_polynomial_fit);
  mls.setPolynomialOrder (polynomial_order);

// mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal>::SAMPLE_LOCAL_PLANE);
// mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZRGBNormal>::RANDOM_UNIFORM_DENSITY);
 // mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal>::VOXEL_GRID_DILATION);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZRGB,PointXYZRGBNormal >::NONE);
 // mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
 // mls.setUpsamplingRadius (0.005);
 // mls.setUpsamplingStepSize (0.003);
 // mls.setDilationIterations (2);
 // mls.setDilationVoxelSize (0.01f);

  search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
  mls.setSearchMethod (tree);
  mls.setComputeNormals (true);

  PCL_INFO ("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial fitting %d, polynomial order %d\n",
            mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialFit(), mls.getPolynomialOrder());
  TicToc tt;
  tt.tic ();
  mls.process (*xyz_cloud_smoothed);


/*
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treee (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  ne.setSearchMethod (treee);

   // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>); 


 pcl::concatenateFields(*xyz_cloud_smoothed, *cloud_normals, *cloud_with_normals); 
*/

  /*pcl::PolygonMesh mesh;
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setDepth (depth);
  poisson.setScale(1.);
  poisson.setSolverDivide (solver_divide);
  poisson.setIsoDivide (iso_divide);
  poisson.setPointWeight (point_weight);
  poisson.setInputCloud (xyz_cloud_smoothed);
  poisson.reconstruct(mesh);
  pcl::io::saveVTKFile ("smoothed.vtk", mesh);
*/

/*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

 pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;
  gp3.setSearchRadius (0.01);

  // Set typical values for the parameters
  gp3.setMu (2.);
  gp3.setMaximumNearestNeighbors (30);
  gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  gp3.setInputCloud (xyz_cloud_smoothed);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

pcl::io::saveVTKFile ("smoothed.vtk", triangles);*/

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", xyz_cloud_smoothed->width * xyz_cloud_smoothed->height); print_info (" points]\n");
// pcl::io::savePCDFile ("smoothed.pcd", *xyz_cloud_smoothed,false);
 //pcl::io::savePLYFile ("smoothed.ply", *xyz_cloud_smoothed );

//  pcl::io::savePolygonFileVTK(rawname+".vtk", triangles,true);

  toPCLPointCloud2 (*xyz_cloud_smoothed, output);
 pcl::io::saveVTKFile ("smoothed.vtk", output );
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output,  Eigen::Vector4f::Zero (),
                        Eigen::Quaternionf::Identity (), true);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}

/* ---[ */
int
main (int argc, char** argv)
{
  print_info ("Moving Least Squares smoothing of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  double search_radius = default_search_radius;
  double sqr_gauss_param = default_sqr_gauss_param;
  bool sqr_gauss_param_set = true;
  int polynomial_order = default_polynomial_order;
  bool use_polynomial_fit = default_use_polynomial_fit;

  parse_argument (argc, argv, "-radius", search_radius);
  if (parse_argument (argc, argv, "-sqr_gauss_param", sqr_gauss_param) == -1)
    sqr_gauss_param_set = false;
  if (parse_argument (argc, argv, "-polynomial_order", polynomial_order) != -1 )
    use_polynomial_fit = true;
  parse_argument (argc, argv, "-use_polynomial_fit", use_polynomial_fit);


  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Do the smoothing
  pcl::PCLPointCloud2 output;
  compute (cloud, output, search_radius, sqr_gauss_param_set, sqr_gauss_param,
           use_polynomial_fit, polynomial_order);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}