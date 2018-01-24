
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


using namespace pcl;
using namespace std;

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
   PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
   if(io::loadPCDFile<PointXYZRGB> (argv[1], *cloud) == -1){
      cout << "fail" << endl;

   } else {

      cout << "loaded" << endl;

      cout << "begin passthrough filter" << endl;
      PointCloud<PointXYZRGB>::Ptr filtered(new PointCloud<PointXYZRGB>());
      PassThrough<PointXYZRGB> filter;

      filter.setFilterFieldName ("z");
      filter.setFilterLimits (1., 2.2);
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;

      filtered=removeOutliers(filtered, 0.05,80 );
    //  filtered=downsample(filtered,0.002);
      std::cout<<"filtered size: "<<filtered->size()<<std::endl;

      std::vector<pcl::PointIndices> filtered_cluster_indices;
  
  
       clusterObjects (filtered, 0.05 , 5000, 50000, filtered_cluster_indices);
       pcl::console::print_info ("Found %lu clusters\n", filtered_cluster_indices.size ());

        PointCloud<PointXYZRGB>::Ptr  temp_cloud (new PointCloud<PointXYZRGB>());
       pcl::copyPointCloud (*filtered, filtered_cluster_indices[0], *temp_cloud);
       filtered = temp_cloud;
       std::cout<<"filtered size: "<<filtered->size()<<std::endl;



      cout << "begin moving least squares" << endl;
      MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal> mls;
      mls.setInputCloud(filtered);
      mls.setSearchRadius(0.05);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      //mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGBNormal>::SAMPLE_LOCAL_PLANE);
     // mls.setUpsamplingRadius(0.005);
     // mls.setUpsamplingStepSize(0.003);

      PointCloud<PointXYZRGBNormal>::Ptr cloud_smoothed (new PointCloud<PointXYZRGBNormal>());
      mls.process(*cloud_smoothed);
      cout << "MLS complete" << endl;
      pcl::io::savePCDFile ("mesh_mls.pcd", *cloud_smoothed,false);

     // return 0;

      // cout << "begin normal estimation" << endl;
      // NormalEstimationOMP<PointXYZ, Normal> ne;
      // ne.setNumberOfThreads(8);
      // ne.setInputCloud(filtered);
      // ne.setRadiusSearch(0.01);
      // Eigen::Vector4f centroid;
      // compute3DCentroid(*filtered, centroid);
      //  ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

      // PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
      // ne.compute(*cloud_normals);
      // cout << "normal estimation complete" << endl;
      // cout << "reverse normals' direction" << endl;

      // for(size_t i = 0; i < cloud_normals->size(); ++i){
      // 	cloud_normals->points[i].normal_x *= -1;
      // 	cloud_normals->points[i].normal_y *= -1;
      // 	cloud_normals->points[i].normal_z *= -1;
      // }

      // cout << "combine points and normals" << endl;
      // PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
      // concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

      // cout << "begin poisson reconstruction" << endl;
      // Poisson<PointNormal> poisson;
      // poisson.setDepth(9);
      // poisson.setInputCloud(cloud_smoothed_normals);
      // PolygonMesh mesh;
      // poisson.reconstruct(mesh);
      // pcl::io::saveVTKFile ("mesh_poisson.vtk", mesh);

      // //io::savePLYFile(argv[2], mesh);

   }
  return (0);
}