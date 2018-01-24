#include <fstream>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <stdio.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace pcl::visualization;

typedef PointNormal PointT;
typedef PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//CloudPtr src, tgt;

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, tgt, result;
bool rejection = true;
bool visualize = false;

boost::shared_ptr<PCLVisualizer> vis;

////////////////////////////////////////////////////////////////////////////////
void
findCorrespondences (const CloudPtr &src,
                     const CloudPtr &tgt,
                     Correspondences &all_correspondences)
{
  //CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
  //CorrespondenceEstimation<PointT, PointT> est;
// std::cout<<"findCorrespondences "<<std::endl;
  CorrespondenceEstimationBackProjection<PointT, PointT, PointT> est;
  est.setInputSource (src);
  est.setInputTarget (tgt);

  est.setSourceNormals (src);
  est.setTargetNormals (tgt);

  est.setKSearch (40);  


  est.determineCorrespondences (all_correspondences);

  //est.determineReciprocalCorrespondences (all_correspondences);
















}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                          const CloudPtr &src,
                          const CloudPtr &tgt,
                          Correspondences &remaining_correspondences)
{
  


//boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
  




/*
  pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;

  corr_rej_dist.setInputCorrespondences(all_correspondences);
  corr_rej_dist.setMaximumDistance(1);
  corr_rej_dist.getCorrespondences(remaining_correspondences);

  CorrespondencesPtr remaining_correspondences_temp3 (new Correspondences);
  corr_rej_dist.getCorrespondences (*remaining_correspondences_temp3);*/


  CorrespondenceRejectorOneToOne corr_rej_one_to_one;
  corr_rej_one_to_one.setInputCorrespondences(all_correspondences);
  corr_rej_one_to_one.getCorrespondences(remaining_correspondences);



  CorrespondencesPtr remaining_correspondences_temp1 (new Correspondences);
  corr_rej_one_to_one.getCorrespondences (*remaining_correspondences_temp1);


/*pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> corr_rej_sac;
  corr_rej_sac.setInputCloud (src);
  corr_rej_sac.setTargetCloud (tgt);
  corr_rej_sac.setInlierThreshold (0.1);
  corr_rej_sac.setMaxIterations (100);
  corr_rej_sac.setInputCorrespondences (remaining_correspondences_temp2);
  corr_rej_sac.getCorrespondences (remaining_correspondences);

return;
*/

 /* CorrespondenceRejectorMedianDistance rej;
 rej.setMedianFactor (558.79241104);

  //rej.setMedianFactor (3);

  

  rej.setInputCorrespondences (remaining_correspondences_temp1);

  rej.getCorrespondences (remaining_correspondences);*/
//return ;

  CorrespondenceRejectorDistance rej;

 rej.setInputSource<PointT> (src);
  rej.setInputTarget<PointT> (tgt);
  rej.setMaximumDistance (1.);    // 1m
  rej.setInputCorrespondences (remaining_correspondences_temp1);
  rej.getCorrespondences (remaining_correspondences);
  return;
 
  
  CorrespondencesPtr remaining_correspondences_temp2 (new Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp2);
  
  pcl::registration::CorrespondenceRejectorTrimmed corr_rej_trimmed;
  corr_rej_trimmed.setOverlapRatio(0.2);
  corr_rej_trimmed.setInputCorrespondences(remaining_correspondences_temp2);
  corr_rej_trimmed.getCorrespondences(remaining_correspondences);

 return;


  CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp);
  PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

  // Reject if the angle between the normals is really off
  CorrespondenceRejectorSurfaceNormal rej_normals;
  rej_normals.setThreshold (acos (deg2rad (45.0)));
  rej_normals.initializeDataContainer<PointT, PointT> ();
  rej_normals.setInputSource<PointT> (src);
  rej_normals.setInputNormals<PointT, PointT> (src);
  rej_normals.setInputTarget<PointT> (tgt);
  rej_normals.setTargetNormals<PointT, PointT> (tgt);
  rej_normals.setInputCorrespondences (remaining_correspondences_temp);
  rej_normals.getCorrespondences (remaining_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
findTransformation (const CloudPtr &src,
                    const CloudPtr &tgt,
                    const CorrespondencesPtr &correspondences,
                    Eigen::Matrix4f &transform)
{
  TransformationEstimationPointToPlaneLLS<PointT, PointT, float> trans_est;
  trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
}

////////////////////////////////////////////////////////////////////////////////
void
view (const CloudConstPtr &src, const CloudConstPtr &tgt, const CorrespondencesPtr &correspondences)
{
  if (!visualize || !vis) return;
  PointCloudColorHandlerCustom<PointT> green (tgt, 0, 255, 0);
  if (!vis->updatePointCloud<PointT> (src, "source"))
  {
    vis->addPointCloud<PointT> (src, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointT> (tgt, green, "target")) vis->addPointCloud<PointT> (tgt, green, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
  pcl::console::TicToc tt;
  tt.tic ();
  if (!vis->updateCorrespondences<PointT> (src, tgt, *correspondences, 1)) 
    vis->addCorrespondences<PointT> (src, tgt, *correspondences, 1, "correspondences");
  tt.toc_print ();
  vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
  //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  vis->spin ();
}

////////////////////////////////////////////////////////////////////////////////
void
icp (const PointCloud<PointT>::Ptr &src, 
     const PointCloud<PointT>::Ptr &tgt,
     Eigen::Matrix4f &transform)
{
  CorrespondencesPtr all_correspondences (new Correspondences), 
                     good_correspondences (new Correspondences);

 // std::cout<<"icp 1"<<std::endl;
  PointCloud<PointT>::Ptr output (new PointCloud<PointT>);

 // PointCloud<pcl::PointXYZRGB> output (new   PointCloud<pcl::PointXYZRGB> );
  *output = *src;
 // std::cout<<"icp 2"<<std::endl;
  Eigen::Matrix4f final_transform (Eigen::Matrix4f::Identity ());

  int iterations = 0;
  DefaultConvergenceCriteria<float> converged (iterations, transform, *good_correspondences);
  converged.setMaximumIterations(10000);
  //converged.setFailureAfterMaximumIterations(true);

   //converged.setMaximumIterationsSimilarTransforms(30);
   //converged.setRotationThreshold(1.1);
   //converged.setRelativeMSE(0.01);
  // ICP loop
  do
  {
    // Find correspondences
   //if (iterations==0)
    findCorrespondences (output, tgt, *all_correspondences);
    PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

   printf("Number of correspondences found: %d \n", all_correspondences->size ());
   // std::cout<<"icp 4"<<std::endl;
    if (rejection)
    {
      // Reject correspondences
      rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
      PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
       printf("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
     //  *all_correspondences=*good_correspondences;
    }
    else
      *good_correspondences = *all_correspondences;

    
    // Find transformation
    findTransformation (output, tgt, good_correspondences, transform);
   // std::cout<<"icp 5"<<std::endl;
    // Obtain the final transformation    
    final_transform = transform * final_transform;

   // std::cout<<"transform "<<transform<<std::endl;

    // Transform the data
    transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());

    // Check if convergence has been reached
    ++iterations;
  
    // Visualize the results
    view (output, tgt, good_correspondences);

   // if (converged) std::cout<<"iteration "<< iterations<<std::endl;
 // std::cout<<"getRelativeMSE "<<converged.getRelativeMSE()<<std::endl;

  }
  while (!converged);
 std::cout<<"Convergence state "<<converged.getConvergenceState()<<std::endl;

 // while (!converged);
  transform = final_transform;




}

////////////////////////////////////////////////////////////////////////////////
  void
  saveTransform (const std::string &file, const Eigen::Matrix4f &transform)
  {
    ofstream ofs;
    ofs.open (file.c_str (), ios::trunc | ios::binary);
    //for (int i = 0; i < 4; ++i)
    //  for (int j = 0; j < 4; ++j)
      //  ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (double));  
              ofs.write (reinterpret_cast<const char*>(&transform), sizeof (Eigen::Matrix4f ));  

    ofs.close ();
  }



void readTransform(const std::string &file,Eigen::Matrix4f &transform ){



fstream binary_file(file.c_str(),ios::binary|ios::in);
//while (binary_file.good()) {

binary_file.read(reinterpret_cast<char *>(&transform),sizeof(Eigen::Matrix4f));
//}

binary_file.close();


std::cout<<"tranform read \n "<<transform<<std::endl;

}

/* ---[ */
int
main (int argc, char** argv)
{
  // Check whether we want to enable debug mode
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, tgt;
 
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_1(new pcl::PointCloud<pcl::PointXYZRGB>);
 // PointCloud<PointT>::Ptr result (new PointCloud<PointT>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

  bool debug = false;
  parse_argument (argc, argv, "-debug", debug);
  if (debug)
    setVerbosityLevel (L_DEBUG);

  parse_argument (argc, argv, "-rejection", rejection);
  parse_argument (argc, argv, "-visualization", visualize);
  if (visualize)
    vis.reset (new PCLVisualizer ("Registration example"));

  // Parse the command line arguments for .pcd and .transform files
  std::vector<int> p_pcd_file_indices, p_tr_file_indices;
  p_pcd_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_pcd_file_indices.size () != 2)
  {
    print_error ("Need one input source PCD file and one input target PCD file to continue.\n");
    print_error ("Example: %s source.pcd target.pcd output.transform\n", argv[0]);
    return (-1);
  }
  p_tr_file_indices = parse_file_extension_argument (argc, argv, ".transform");
  if (p_tr_file_indices.size () != 1)
  {
    print_error ("Need one output transform file to continue.\n");
    print_error ("Example: %s source.pcd target.pcd output.transform\n", argv[0]);
    return (-1);
  }
  
  // Load the files
  print_info ("Loading %s as source and %s as target...\n", argv[p_pcd_file_indices[0]], argv[p_pcd_file_indices[1]]);

 // src.reset (new PointCloud<PointT>);
  //tgt.reset (new PointCloud<PointT>);
   src.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
  tgt.reset (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (loadPCDFile (argv[p_pcd_file_indices[0]], *src) == -1 || loadPCDFile (argv[p_pcd_file_indices[1]], *tgt) == -1)
  {
    print_error ("Error reading the input files!\n");
    return (-1);
  }


  std::vector<int> indices; 
  pcl::removeNaNFromPointCloud(*src,*src, indices); 

  pcl::removeNaNFromPointCloud(*tgt,*tgt, indices); 

    //pcl::copyPointCloud (*src, *src_1);
  //  src_1=src->makeShared();
   //pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_1(new pcl::PointCloud<pcl::PointXYZRGB>(*src));

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_1(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>> (*src));
  Cloud::Ptr points_with_normals_src (new Cloud);
  Cloud::Ptr points_with_normals_tgt (new Cloud);

//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, PointT> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);

  norm_est.compute (*points_with_normals_tgt);
      pcl::io::savePCDFile ("trg.pcd", *tgt, true);
  pcl::io::savePCDFile ("points_with_normals_tgt.pcd", *points_with_normals_tgt, true);

  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

     pcl::io::savePCDFile ("trgAf.pcd", *tgt, true);
  pcl::io::savePCDFile ("points_with_normals_tgtAf.pcd", *points_with_normals_tgt, true);


    //  pcl::io::savePCDFile ("points_with_normals_tgt", *tgt, true);

  // Compute the best transformtion
  Eigen::Matrix4f transform;
 // icp (src, tgt, transform);
  icp (points_with_normals_src, points_with_normals_tgt , transform);
  saveTransform (argv[p_tr_file_indices[0]], transform);
  std::cout<<"transform \n"<<transform<<std::endl;
Eigen::Matrix4f transform_new;
readTransform(argv[p_tr_file_indices[0]],transform_new);


 // pcl::io::savePCDFile ("src_1.pcd", *, true);
    pcl::io::savePCDFile ("trg_withnormals.pcd", *points_with_normals_tgt, true);

  pcl::transformPointCloud (*src, *result, transform.cast<float> ());

//transformPointCloudWithNormals (*points_with_normals_src, *result, transform.cast<float> ());
//pcl::copyPointCloud (*result, *src);

  pcl::io::savePCDFile ("result.pcd", *result, true);



 /*pcl::visualization::PCLVisualizer vis;

   // cloud->sensor_orientation_.w() = 0.0;
   // cloud->sensor_orientation_.x() = 1.0;
  //  cloud->sensor_orientation_.y() = 0.0;
   // cloud->sensor_orientation_.z() = 0.0; 

    vis.addPointCloud (src);
   // vis.addPointCloud (result);
    vis.resetCamera ();
    vis.spin ();
*/



  cerr.precision (15);
  std::cerr << transform << std::endl;
}
