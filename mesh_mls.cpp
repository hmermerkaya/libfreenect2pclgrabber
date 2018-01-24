#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace std;
using namespace boost::filesystem;


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

int
main (int argc, char** argv)
{
   // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
   
   

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new PointCloud<PointXYZRGB>());
  pcl::PassThrough<pcl::PointXYZRGB> filter;

  filter.setFilterFieldName ("z");
  filter.setFilterLimits (0., 10.3);

  PointCloud<PointXYZRGB>::Ptr cloud_smoothed (new PointCloud<PointXYZRGB>());

  MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
  mls.setSearchRadius(0.05);
  mls.setPolynomialFit(true);
  mls.setPolynomialOrder(2);
  //mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal>::SAMPLE_LOCAL_PLANE);
  // mls.setUpsamplingRadius(0.005);
  // mls.setUpsamplingStepSize(0.003);





  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal > norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (50);


  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;
  gp3.setSearchRadius (0.05);

  // Set typical values for the parameters
  gp3.setMu (3.2);
  gp3.setMaximumNearestNeighbors (50);
  gp3.setMaximumSurfaceAngle(M_PI/2); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  path p(argc>1? argv[1] : ".");
  std::chrono::high_resolution_clock::time_point tnow, tpost;


   try
  {
    if (exists(p))    // does p actually exist?
    {
      if (is_regular_file(p))        // is p a regular file?
        cout << p << " size is " << file_size(p) << '\n';

      else if (is_directory(p))      // is p a directory?
      {
        cout << p << " is a directory containing:\n";

        typedef std::vector<path> vec;             // store paths,
        vec v;                                // so we can sort them later

        copy(directory_iterator(p), directory_iterator(), back_inserter(v));

        sort(v.begin(), v.end());             // sort, since directory iteration
        vec::const_iterator it(v.begin()) ;  
        int i=0;                        // is not ordered on some file systems
        while(it!=v.end()) {
      //  for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
      //  { 

              cout << (*it) << '\n';
              std::string file = (*it) .string();

      //  for ( std::vector<directory_entry>::const_iterator it = v.begin(); it != v.end();  ++ it )
      //  {
            
              tnow = std::chrono::high_resolution_clock::now();

              //boost::this_thread::sleep (boost::posix_time::milliseconds (50));  
              if ( pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud) == -1) //* load the file
                {
                  PCL_ERROR ("Couldn't read pcd file  \n");
                  continue;
                }

                filter.setInputCloud(cloud);
                filter.filter(*filtered);
                cout << "passthrough filter complete" << endl;

                filtered=removeOutliers(filtered, 0.05,80);
                filtered=downsample(filtered,0.002);

            /*    std::vector<pcl::PointIndices> filtered_cluster_indices;
  
          
               clusterObjects (filtered, 0.05 , 8000, 50000, filtered_cluster_indices);
               pcl::console::print_info ("Found %lu clusters\n", filtered_cluster_indices.size ());

                PointCloud<PointXYZRGB>::Ptr  temp_cloud (new PointCloud<PointXYZRGB>());
               pcl::copyPointCloud (*filtered, filtered_cluster_indices[0], *temp_cloud);
               filtered = temp_cloud;*/
                cout << "begin moving least squares" << endl;
                mls.setInputCloud(filtered);
                mls.process(*cloud_smoothed);

              /*  boost::filesystem::change_extension(file, "").string();
                size_t lastindex = file.find_last_of("."); 
                string rawname = file.substr(0, lastindex); 
                pcl::io::savePCDFile (rawname+"_smoothed.pcd", *cloud_smoothed,false);*/
                                
              //  norm_est.setInputCloud (filtered);

                norm_est.setInputCloud (cloud_smoothed);
                norm_est.compute (*cloud_with_normals);
                pcl::copyPointCloud (*cloud_smoothed, *cloud_with_normals);

                tree2->setInputCloud (cloud_with_normals);

                gp3.setInputCloud (cloud_with_normals);
                gp3.setSearchMethod (tree2);
                gp3.reconstruct (triangles);

                // Additional vertex information
               // std::vector<int> parts = gp3.getPartIDs();
               // std::vector<int> states = gp3.getPointStates();

                boost::filesystem::change_extension(file, "").string();
                size_t lastindex = file.find_last_of("."); 
                string rawname = file.substr(0, lastindex); 
               //pcl::io::savePolygonFileVTK(rawname+".vtk", triangles,true);
                pcl::io::savePLYFileBinary(rawname+".ply", triangles);   
                // pcl::io::saveVTKFile (rawname+".vtk", triangles);

 
              

              tpost = std::chrono::high_resolution_clock::now();
              std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;
             

              i++;
              ++it;
         

        }





      }
      else
        cout << p << " exists, but is neither a regular file nor a directory\n";
    }
    else
      cout << p << " does not exist\n";
  }

  catch (const filesystem_error& ex)
  {
    cout << ex.what() << '\n';
  }




  
  return (0);
}