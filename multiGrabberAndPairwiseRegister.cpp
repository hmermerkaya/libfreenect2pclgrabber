/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo  Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include "icp.h"
#include "filters.h"
#include "segmentation.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//typedef std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>() makePair;
//typedef pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

struct ObjectRecognitionParameters
{
  // Filter parameters
  float min_depth;
  float max_depth;
  float downsample_leaf_size;
  float outlier_rejection_radius;
  int outlier_rejection_min_neighbors;

  // Segmentation parameters
  float plane_inlier_distance_threshold;
  int max_ransac_iterations;
  float cluster_tolerance;
  int min_cluster_size;
  int max_cluster_size;

  // Feature estimation parameters
  float surface_normal_radius;
  float keypoints_min_scale;
  float keypoints_nr_octaves;
  float keypoints_nr_scales_per_octave;
  float keypoints_min_contrast;
  float local_descriptor_radius;

  // Registration parameters
  float initial_alignment_min_sample_distance;
  float initial_alignment_max_correspondence_distance;
  int initial_alignment_nr_iterations;
  float icp_max_correspondence_distance;
  float icp_outlier_rejection_threshold;
  float icp_transformation_epsilon;
  int icp_max_iterations;
};




bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}


struct K2G_generator{
public:
	K2G_generator(Processor freenectprocessor, bool mirroring, char ** argv): freenectprocessor_(freenectprocessor), mirroring_(mirroring), argv_(argv),n_(0){}
    K2G * operator ()(){return new K2G(freenectprocessor_, mirroring_, argv_[n_++ + 2]);}
private:
	unsigned int n_;
	Processor freenectprocessor_;
	bool mirroring_;
	char ** argv_;
};

struct PlySaver{

  PlySaver(std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> & clouds, bool binary, bool use_camera, std::vector<K2G *> & kinects): 
           binary_(binary), use_camera_(use_camera), clouds_(clouds), kinects_(kinects){}

  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> & clouds_;
  std::vector<K2G *> & kinects_;
  bool binary_;
  bool use_camera_;
};

void
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  std::string pressed;
  pressed = event.getKeySym();
  PlySaver * s = (PlySaver*)data;
  if(event.keyDown())
  {
    if(pressed == "s")
    {
      
      pcl::PLYWriter writer;
      std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
      std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
      cv::Mat color;
      for(size_t i = 0; i < s->kinects_.size(); ++i){
      	writer.write ("cloud_"+ std::to_string(i) + "_" + now + ".ply", *(s->clouds_[i]), s->binary_, s->use_camera_);
        s->kinects_[i]->getColor(color);
      	cv::imwrite("color_" + std::to_string(i) + "_" + now + ".jpg", color);
      }
      std::cout << "saved " << "cloud and color " + now << std::endl;
    }
    if(pressed == "x")
    {
        for(auto & k : s->kinects_){
        	k->storeParameters();
        }
        std::cout << "stored calibration parameters" << std::endl;
    }
  }
}

int main(int argc, char *argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively" << std::endl;
  std::cout << "followed by the kinect2 serials and a last 1 for mirroring" << std::endl;
  std::cout << "Press \'s\' to store both clouds and color images." << std::endl;
  std::cout << "Press \'x\' to store both calibrations." << std::endl;

  if(argc < 3){
    std::cout << "Wrong syntax! specify at least processor and one serial" << std::endl;
    return -1;
  }


  // Set te processor to input value or to default OPENGL
  Processor freenectprocessor = OPENGL;
  freenectprocessor = static_cast<Processor>(atoi(argv[1]));

  // check if mirroring is enabled
  bool mirroring = atoi(argv[argc - 1]) == 1 ? true : false;
  if(mirroring)
  	std::cout << "mirroring enabled" << std::endl;

  // Count number of input serials which represent the number of active kinects
  int kinect2_count = mirroring ? argc - 3 : argc - 2;
  std::cout << "loading " << kinect2_count << " devices" << std::endl;

  // Initialize container structures
  std::vector<K2G *> kinects(kinect2_count);
  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> clouds(kinect2_count);
  //std::map<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloudPairs;
  //cloudPairs.insert ( std::pair<char,int>('a',100) );

 //pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB> ), source, target,targetToSrc;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
  std::vector<Eigen::Matrix4f> Transforms;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer->setBackgroundColor (255, 255, 255);

  // Generate all the kinect grabbers
  K2G_generator kinect_generator(freenectprocessor, mirroring, argv);
  std::generate(kinects.begin(), kinects.end(), kinect_generator);


  //Obtain filters parameters
  ObjectRecognitionParameters params;
  ifstream params_stream;


 

  /*double min_depth, max_depth;
  bool threshold_depth = pcl::console::parse_2x_arguments (argc, argv, "-t", min_depth, max_depth) > 0;
  if (threshold_depth)
  {
    size_t n = cloud->size ();
    cloud = thresholdDepth (cloud, min_depth, max_depth);
    pcl::console::print_info ("Eliminated %lu points outside depth limits\n", n - cloud->size ());
  }
*/

/*
  if (is_file_exist("output.transform") ) 
   readTransform("output.transform", GlobalTransform);
  else return 0;
  std::cout <<"GlobalTransform \n" << GlobalTransform<<std::endl;*/
  
//for(size_t i = 0; i < kinect2_count-1; ++i)  cloudPairs.insert(std::make_pair(kinects[i]->getCloud(),kinects[i+1]->getCloud()));
//for(auto & k : kinects)
  //  k->shutDown();

 // cloudPairs.insert(std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>(kinects[i-1]->getCloud(),kinects[i]->getCloud()));

 //int i=0;

PointCloud::Ptr src, tgt;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_orj(new pcl::PointCloud<pcl::PointXYZRGB>), tgt_orj(new pcl::PointCloud<pcl::PointXYZRGB>);
//
Transforms.resize(kinect2_count-1);
for(size_t i = 0; i < kinect2_count-1; ++i) {
// for (size_t i=0;i<cloudPairs.size();i++) {
  //std::cout<<"pointcloud size "<<k.first->size()<<std::endl;
   
   
   
 //   src=k.first;tgt=k.second;
/*float distan;
    std::cout<<"Enter correspondece rejector distance of "<<i+1<<". pair in meters : ";
    cin>>distan;*/


  string file="output_"+std::to_string(i)+".transform";
  if (is_file_exist(file.c_str()) ) {
    Eigen::Matrix4f transform; 
    readTransform(file,transform);
    Transforms.at(i)=transform;
    GlobalTransform=transform;
    continue;
  } 



  while (1) {
    
    

    src.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
    tgt.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
   // src=k.first;tgt=k.second;
    src= kinects[i]->getCloud();
    tgt= kinects[i+1]->getCloud();
   // std::cout<<"sizes "<<src->size()<< " "<<tgt->size();
    // clouds[i].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    // clouds[i+1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    // *clouds[i]=*src;
    // *clouds[i+1]=*tgt;

   // std::vector<int> indices; 
   // pcl::removeNaNFromPointCloud(*src,*src, indices); 

    //pcl::removeNaNFromPointCloud(*tgt,*tgt, indices); 

    //Open the file that has filter parameters

    params_stream.open ("filters_"+std::to_string(i+1)+".txt");
    if (params_stream.is_open())
    {
      params_stream >> params.min_depth;
      params_stream >> params.max_depth;
      params_stream >> params.downsample_leaf_size;
      params_stream >> params.outlier_rejection_radius;
      params_stream >> params.outlier_rejection_min_neighbors;
      params_stream >> params.icp_max_correspondence_distance  ;
      params_stream.close ();
    } else {
      std::cout<<"filters_"<<i+1<<".txt file does not exist. To continue registering "<<i+1<<". pair of clouds, first create the file and then press \"R(r)\" or to skip registering this pair of clouds, press  any key other than \"r\"  "<<std::endl;
      char c=cin.get();
     if (c=='r' || c=='R') {
      cin.ignore();
      continue;
      }
     else break;

    }
    // char c=getchar();
    //char c = fgetc(stdin);
    // std::cout<<"c "<<c<<std::endl;
     



    size_t n = src->size ();
// Threshold depth
    src = thresholdDepth (src, params.min_depth, params.max_depth);
    pcl::console::print_info ("Eliminated %lu points outside depth limits\n", n - src->size ());
    n = tgt->size();
    tgt = thresholdDepth (tgt, params.min_depth, params.max_depth);
    pcl::console::print_info ("Eliminated %lu points outside depth limits\n", n - tgt->size ());

 // Remove outliers

    n = src->size ();
    src = removeOutliers (src, params.outlier_rejection_radius, (int)params.outlier_rejection_min_neighbors);
    pcl::console::print_info ("Removed %lu outliers\n", n - src->size ());
    n=tgt->size();
    tgt = removeOutliers (tgt, params.outlier_rejection_radius, (int)params.outlier_rejection_min_neighbors);
    pcl::console::print_info ("Removed %lu outliers\n", n - tgt->size ());
    

//Downsample
    n = src->size ();
    src = downsample (src, params.downsample_leaf_size);
    pcl::console::print_info ("Downsampled from %lu to %lu points\n", n, src->size ());
    n=tgt->size();
    tgt = downsample (tgt, params.downsample_leaf_size);
    pcl::console::print_info ("Downsampled from %lu to %lu points\n", n, tgt->size ());

 
    //Segmentation

  params_stream.open ("segmentation_"+std::to_string(i+1)+".txt");
    
    if (params_stream.is_open())
  {
    params_stream >> params.plane_inlier_distance_threshold;
    params_stream >> params.max_ransac_iterations;
    params_stream >> params.cluster_tolerance;
    params_stream >> params.min_cluster_size;
    params_stream >> params.max_cluster_size;
    params_stream.close ();
  }
  else
  {
     std::cout<<"segmentation_"<<i+1<<".txt file does not exist. To continue registering "<<i+1<<". pair of clouds, first create the file and then press \"R(r)\" or to skip registering this pair of clouds, press  any key other than \"r\"  "<<std::endl;
      char c=cin.get();
     if (c=='r' || c=='R') {
      cin.ignore();
      continue;
      }
     else break;

  }

  

// Subtract the dominant plane
     n = src->size ();
    src = findAndSubtractPlane (src,params.plane_inlier_distance_threshold, (int) params.max_ransac_iterations );
    pcl::console::print_info ("Subtracted %lu points along the detected plane\n", n - src->size ());

   

    n = tgt->size ();
    tgt = findAndSubtractPlane (tgt,params.plane_inlier_distance_threshold, (int) params.max_ransac_iterations );
    pcl::console::print_info ("Subtracted %lu points along the detected plane\n", n - tgt->size ());

  // Cluster points


   std::vector<pcl::PointIndices> src_cluster_indices;
  
  
    clusterObjects (src, params.cluster_tolerance , (int) params.min_cluster_size, (int)params.max_cluster_size, src_cluster_indices);
    pcl::console::print_info ("Found %lu clusters\n", src_cluster_indices.size ());

    PointCloudPtr temp_cloud (new PointCloud);
    pcl::copyPointCloud (*src, src_cluster_indices[0], *temp_cloud);
    src = temp_cloud;
    std::cout<<"src size: "<<src->size()<<std::endl;


   std::vector<pcl::PointIndices> tgt_cluster_indices;
  
  
    clusterObjects (tgt, params.cluster_tolerance , (int) params.min_cluster_size, (int)params.max_cluster_size, tgt_cluster_indices);
    pcl::console::print_info ("Found %lu clusters\n", tgt_cluster_indices.size ());

     temp_cloud.reset(new PointCloud);
    pcl::copyPointCloud (*tgt, tgt_cluster_indices[0], *temp_cloud);
    tgt = temp_cloud;

    std::cout<<"tgt size: "<<tgt->size()<<std::endl;




    Cloud::Ptr points_with_normals_src (new Cloud);
    Cloud::Ptr points_with_normals_tgt (new Cloud);

//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    pcl::NormalEstimation<pcl::PointXYZRGB, PointNT> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (40);
  
    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);

    norm_est.compute (*points_with_normals_tgt);
    //pcl::io::savePCDFile ("trg.pcd", *tgt, true);
    //pcl::io::savePCDFile ("points_with_normals_tgt.pcd", *points_with_normals_tgt, true);

    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //pcl::io::savePCDFile ("trgAf.pcd", *tgt, true);
    //pcl::io::savePCDFile ("points_with_normals_tgtAf.pcd", *points_with_normals_tgt, true);
   
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta =M_PI/1.; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,2) = -sin(theta);
  transform_1 (2,0) = sin (theta);
  transform_1 (2,2) = cos (theta);
   
 // transform_1 (2,3) = 0.05;

/*transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
*/
   pcl::transformPointCloudWithNormals(*points_with_normals_tgt,*points_with_normals_tgt,transform_1);

 // pcl::transformPointCloud(*tgt,*tgt,transform_1);

      //  pcl::io::savePCDFile ("points_with_normals_tgt", *tgt, true);

    // Compute the best transformtion
    Eigen::Matrix4f transform ;//= Eigen::Matrix4f::Identity(); ;
   // icp (src, tgt, transform);
    
   icp (points_with_normals_src, points_with_normals_tgt , transform, params.icp_max_correspondence_distance);


    std::cout<<"All and good correspondeces: "<<all_corr<<" "<<good_corr<<std::endl;
   // if (all_corr/4 > good_corr) continue; 
    transform= transform_1*transform;
   // pcl::transformPointCloud (*tgt, *tgt, transform_1);
    pcl::transformPointCloud (*tgt, *tgt, transform.inverse());

    std::cout<<"transform \n"<<transform.inverse()<<std::endl;
   if (tgt->size() < src->size()) all_corr=tgt->size();
   else all_corr=src->size();

    if (all_corr*0.8 > good_corr) continue; 


     src->sensor_orientation_.w() = 0.0;         
     src->sensor_orientation_.x() = 1.0;    
     src->sensor_orientation_.y() = 0.0;        
     src->sensor_orientation_.z() = 0.0;  

     tgt->sensor_orientation_.w() = 0.0;
     tgt->sensor_orientation_.x() = 1.0;
     tgt->sensor_orientation_.y() = 0.0;
     tgt->sensor_orientation_.z() = 0.0; 

   
     viewer->removeAllPointClouds();

  PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue (src, 0, 0, 255);
  PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (tgt, 255, 0, 0);

     viewer->addPointCloud<pcl::PointXYZRGB>(src, blue, "src");

     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "src");

   viewer->addPointCloud<pcl::PointXYZRGB>(tgt, red, "tgt");

   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tgt");


    viewer->spin();

std::cout<<"Are you done with registering "<<i+1<<". pair of clouds? Yes(y) or No(n)? ";

 /*   char c=cin.get();
    if (c=='y') {
      cin.ignore();
      GlobalTransform=transform.inverse()*GlobalTransform;
      Transforms.push_back(GlobalTransform);
      break;
    }
    else if (c=='n'){
      cin.ignore();
      continue;
    }
    else {

      cin.ignore();
      continue;
    }
*/

char c;
  cin>>c;

   if (c=='y') {
      GlobalTransform=GlobalTransform*transform.inverse();
      std::cout<<"Global transform \n"<<GlobalTransform<<std::endl;

      Transforms.at(i)=GlobalTransform;
      saveTransform (file, GlobalTransform);
      break;
    }
    else if (c=='n'){
      continue;
    }
    else {

      continue;
    }

   
   // GlobalTransform=GlobalTransform*transform.inverse();
    //saveTransform (argv[p_tr_file_indices[0]], transform);
 
    //Eigen::Matrix4f transform_new;
    //readTransform(argv[p_tr_file_indices[0]],transform_new);


 // pcl::io::savePCDFile ("src_1.pcd", *, true);
   // pcl::io::savePCDFile ("trg_withnormals.pcd", *points_with_normals_tgt, true);

    //pcl::transformPointCloud (*src, *result, transform.cast<float> ());

//transformPointCloudWithNormals (*points_with_normals_src, *result, transform.cast<float> ());
//pcl::copyPointCloud (*result, *src);

   // pcl::io::savePCDFile ("result.pcd", *result, true);



}


//  i++;

}



  //viewer->resetCamera();//moveAllPointClouds();

 //viewer->resetStoppedFlag ();

//viewer->initCameraParameters ();


//std::cout<<"Transforms size "<<Transforms.size();

//std::ofstream FILE("./output.transforms", std::ios::out | std::ofstream::binary);
//std::copy(Transforms.begin(), Transforms.end(), std::ostreambuf_iterator<Eigen::Matrix4f>(FILE));

size_t sz = Transforms.size();
if (sz>0) {
  std::ofstream FILE("./output.transforms", std::ios::out | std::ofstream::binary);
  FILE.write(reinterpret_cast<const char*>(&sz), sizeof(sz));
  FILE.write(reinterpret_cast<const char*>(&Transforms[0]), sz * sizeof(Transforms[0]));
  FILE.close ();
}

 Eigen::Matrix4f trans=Eigen::Matrix4f::Identity();
 trans(0,3)=0.05;
 trans(1,3)=-0.025;
  trans(2,3)=0.05;
//trans=Transforms.at(0)*trans;

//std::cout<<"trans \n"<<trans<<std::endl;

viewer->removeAllPointClouds();
for (int i=0;i<Transforms.size();i++)  std::cout<<"Transform "<<i<< "\n"<<Transforms.at(i)<<std::endl;

  for(size_t i = 0; i < kinect2_count; ++i)
  {
  	
    clouds[i] = kinects[i]->getCloud();

  	clouds[i]->sensor_orientation_.w() = 0.0;
  	clouds[i]->sensor_orientation_.x() = 1.0;
  	clouds[i]->sensor_orientation_.y() = 0.0;
  	clouds[i]->sensor_orientation_.z() = 0.0; 

  	viewer->addPointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(clouds[i]), "sample cloud_" + i);
  	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud_" + i);
  }

  // Add keyboards callbacks
  PlySaver ps(clouds, false, false, kinects);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
   std::cout << "starting cycle" << std::endl;

   std::chrono::high_resolution_clock::time_point tnow, tpost;


  // PCL_INFO ("Press any key to continue .\n");
  
  // viewer->spin();
  // //cin.get();


  // Start visualization cycle

   viewer->resetStoppedFlag ();

  while(!viewer->wasStopped()){

    viewer->spinOnce ();

    //tnow = std::chrono::high_resolution_clock::now();



    /* for(size_t i = 0; i < kinect2_count; ++i) {
      
      clouds[i] = kinects[i]->updateCloud(clouds[i]);
      if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

      }*/

    //tpost = std::chrono::high_resolution_clock::now();
    //std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;

    for(size_t i = 0; i < kinect2_count; ++i) {
      clouds[i] = kinects[i]->updateCloud(clouds[i]);

    //  clouds[i] = kinects[i]->getCloud();

    //  clouds[i]= thresholdDepth (clouds[i], 0.4, 2.3);
     // clouds[i]=removeOutliers(clouds[i], 0.03, 50);
    //  clouds[i]=downsample(clouds[i], 0.005);


      if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );


      viewer->updatePointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (clouds[i]), "sample cloud_" + i);
    }
  }

  // Close all kinect grabbers
  for(auto & k : kinects)
  	k->shutDown();

  return 0;
}

