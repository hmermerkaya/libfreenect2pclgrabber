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
//using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
using namespace pcl::visualization;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

//typedef pcl::PointXYZ PointNT;
//typedef pcl::PointCloud<PointNT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//CloudPtr src, tgt;

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, tgt, result;
bool rejection = true;
bool visualize = false;


int all_corr;
int good_corr;
//float distan=0.8;

//boost::shared_ptr<PCLVisualizer> vis;

////////////////////////////////////////////////////////////////////////////////





void
findCorrespondences (const CloudPtr &src,
                     const CloudPtr &tgt,
                     pcl::Correspondences &all_correspondences)
{

//CorrespondenceEstimationNormalShooting<PointNT, PointNT, PointNT> est;
 // CorrespondenceEstimation<PointNT, PointNT> est;
// std::cout<<"findCorrespondences "<<std::endl;
 CorrespondenceEstimationBackProjection<PointNT, PointNT, PointNT> est;

  est.setInputSource (src);
  est.setInputTarget (tgt);

  est.setSourceNormals (src);
 est.setTargetNormals (tgt);

  est.setKSearch (30);  


  est.determineCorrespondences (all_correspondences);

  //est.determineReciprocalCorrespondences (all_correspondences);


/*
  pcl::registration::CorrespondenceEstimationNormalShooting <PointNT, PointNT, PointNT> ce;
  ce.setInputSource (src);
  ce.setKSearch (40);
  ce.setSourceNormals (src);
  ce.setInputTarget (tgt);
  ce.determineCorrespondences (all_correspondences);*/












}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences (const pcl::CorrespondencesPtr &all_correspondences,
                          const CloudPtr &src,
                          const CloudPtr &tgt,
                          pcl::Correspondences &remaining_correspondences,float distan)
{
  


//boost::shared_ptr<pcl::Correspondences> correspondences_result_rej_sac (new pcl::Correspondences);
  




/*
  pcl::registration::CorrespondenceRejectorDistance corr_rej_dist;

  corr_rej_dist.setInputCorrespondences(all_correspondences);
  corr_rej_dist.setMaximumDistance(1);
  corr_rej_dist.getCorrespondences(remaining_correspondences);

  CorrespondencesPtr remaining_correspondences_temp3 (new Correspondences);
  corr_rej_dist.getCorrespondences (*remaining_correspondences_temp3);*/


 /* CorrespondenceRejectorOneToOne corr_rej_one_to_one;
  corr_rej_one_to_one.setInputCorrespondences(all_correspondences);
  corr_rej_one_to_one.getCorrespondences(remaining_correspondences);



  pcl::CorrespondencesPtr remaining_correspondences_temp1 (new pcl::Correspondences);
  corr_rej_one_to_one.getCorrespondences (*remaining_correspondences_temp1);
*/

/*pcl::registration::CorrespondenceRejectorSampleConsensus<PointNT> corr_rej_sac;
  corr_rej_sac.setInputCloud (src);
  corr_rej_sac.setTargetCloud (tgt);
  corr_rej_sac.setInlierThreshold (0.01);
  corr_rej_sac.setMaxIterations (1000);
  corr_rej_sac.setInputCorrespondences (all_correspondences);
  corr_rej_sac.getCorrespondences (remaining_correspondences);
 Eigen::Matrix4f transform_res_from_SAC = corr_rej_sac.getBestTransformation ();
 std::cout<<"transform_res_from_SAC "<<transform_res_from_SAC<<std::endl;
return;
*/

 /*CorrespondenceRejectorMedianDistance rej;
 rej.setMedianFactor (8.79241104);

  //rej.setMedianFactor (3);

  

  rej.setInputCorrespondences (all_correspondences);

  rej.getCorrespondences (remaining_correspondences);*/
//return ;


  CorrespondenceRejectorDistance rej;

 rej.setInputSource<PointNT> (src);
  rej.setInputTarget<PointNT> (tgt);
 
  rej.setMaximumDistance (distan);    
 // distan*=0.95;
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
 return; 
 
  
  pcl::CorrespondencesPtr remaining_correspondences_temp2 (new pcl::Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp2);
  
  pcl::registration::CorrespondenceRejectorTrimmed corr_rej_trimmed;
  corr_rej_trimmed.setOverlapRatio(0.8);
  corr_rej_trimmed.setInputCorrespondences(remaining_correspondences_temp2);
  corr_rej_trimmed.getCorrespondences(remaining_correspondences);

 return;


  pcl::CorrespondencesPtr remaining_correspondences_temp (new pcl::Correspondences);
  rej.getCorrespondences (*remaining_correspondences_temp);
  PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());

  // Reject if the angle between the normals is really off
  CorrespondenceRejectorSurfaceNormal rej_normals;
  rej_normals.setThreshold (acos (pcl::deg2rad (45.0)));
  rej_normals.initializeDataContainer<PointNT, PointNT> ();
  rej_normals.setInputSource<PointNT> (src);
  rej_normals.setInputNormals<PointNT, PointNT> (src);
  rej_normals.setInputTarget<PointNT> (tgt);
  rej_normals.setTargetNormals<PointNT, PointNT> (tgt);
  rej_normals.setInputCorrespondences (remaining_correspondences_temp);
  rej_normals.getCorrespondences (remaining_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
findTransformation (const CloudPtr &src,
                    const CloudPtr &tgt,
                    const pcl::CorrespondencesPtr &correspondences,
                    Eigen::Matrix4f &transform)
{
  TransformationEstimationPointToPlaneLLS<PointNT, PointNT, float> trans_est;
  trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
}

////////////////////////////////////////////////////////////////////////////////
/*void
view (const CloudConstPtr &src, const CloudConstPtr &tgt, const CorrespondencesPtr &correspondences)
{
  if (!visualize || !vis) return;
  PointCloudColorHandlerCustom<PointNT> green (tgt, 0, 255, 0);
  if (!vis->updatePointCloud<PointNT> (src, "source"))
  {
    vis->addPointCloud<PointNT> (src, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointNT> (tgt, green, "target")) vis->addPointCloud<PointNT> (tgt, green, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
  pcl::console::TicToc tt;
  tt.tic ();
  if (!vis->updateCorrespondences<PointNT> (src, tgt, *correspondences, 1)) 
    vis->addCorrespondences<PointNT> (src, tgt, *correspondences, 1, "correspondences");
  tt.toc_print ();
  vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
  //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  vis->spin ();
}
*/
////////////////////////////////////////////////////////////////////////////////
void
icp (const pcl::PointCloud<PointNT>::Ptr &src, 
     const pcl::PointCloud<PointNT>::Ptr &tgt,
     Eigen::Matrix4f &transform,float distan)
{
  pcl::CorrespondencesPtr all_correspondences (new pcl::Correspondences), 
                     good_correspondences (new pcl::Correspondences);

 // std::cout<<"icp 1"<<std::endl;
  pcl::PointCloud<PointNT>::Ptr output (new pcl::PointCloud<PointNT>);

 // PointCloud<pcl::PointXYZRGB> output (new   PointCloud<pcl::PointXYZRGB> );
  *output = *src;
 // std::cout<<"icp 2"<<std::endl;
  Eigen::Matrix4f final_transform (Eigen::Matrix4f::Identity ());

  int iterations = 0;
  DefaultConvergenceCriteria<float> converged (iterations, transform, *good_correspondences);
  converged.setMaximumIterations(150);
  //converged.setFailureAfterMaximumIterations(true);

//converged.setMaximumIterationsSimilarTransforms(30);
   //converged.setRotationThreshold(1.1);
   //converged.setRelativeMSE(0.01);
  // ICP loop
  float distan_bck=distan;
  do
  {


     if (iterations<20) distan=100;
   //  else if (iterations >= 20 && iterations < 50 ) distan=1;
     else distan=distan_bck;
    // Find correspondences
   //if (iterations==0)
    findCorrespondences (output, tgt, *all_correspondences);
    PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());

   //printf("Number of correspondences found: %d \n", all_correspondences->size ());
   // std::cout<<"icp 4"<<std::endl;
    if (rejection)
    {
      // Reject correspondences
      rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences,distan);
      PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
      // printf("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
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
    //view (output, tgt, good_correspondences);
   if (iterations%10==0) {
     //if (all_corr/2>good_corr) break;
    printf("Number of correspondences found: %d \n", all_correspondences->size ());

    printf("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
    std::cout<<"distance: "<<distan<<std::endl;
   }

 // if (all_correspondences->size ()/4 > good_correspondences->size () ) break;
   // if (converged) std::cout<<"iteration "<< iterations<<std::endl;
 // std::cout<<"getRelativeMSE "<<converged.getRelativeMSE()<<std::endl;
  // if (all_correspondences->size ()==0) return;
  
//  if (distan<=0.07) distan=0.07;
  }
  while (distan==100 || !converged );
 // std::cout<< "Number of correspondences found: "<< all_correspondences->size ()<<std::endl;
  //std::cout<< "Number of correspondence remaining after rejection "<< good_correspondences->size ()<<std::endl;

 std::cout<<"Convergence state "<<converged.getConvergenceState()<<std::endl;

  all_corr= all_correspondences->size() ;
  good_corr=good_correspondences->size();
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
