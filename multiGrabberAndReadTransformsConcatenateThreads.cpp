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
//#include "icp.h"
#include "filters.h"
//#include "segmentation.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/thread/condition.hpp>

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//typedef std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>() makePair;
//typedef pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

boost::mutex io_mutex;

void threadPointCloud (K2G* kinect, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pointCloud  , Eigen::Matrix4f transform) {
     //boost::mutex::scoped_lock buff_lock (io_mutex);

    pointCloud=kinect->getCloud();
    //pointCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB> (*kinect->getCloud()));

    //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> 

  //  std::cout<<" pointCloud address before" <<pointCloud<<std::endl;
   //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>   
  // pointCloud= thresholdDepth (pointCloud, 1.3, 2.0);
  //  std::cout<<" pointCloud address after" <<pointCloud<<std::endl;

    // pointCloud_1=removeOutliers(pointCloud_1,0.05,50);
   // pointCloud=pointCloud_1;
   // pcl::transformPointCloud (*pointCloud_1, *pointCloud_1, transform);

    //pointCloud = pointCloud_1;

   //std::cout<<" pointCloud point number before:" <<pointCloud->size()<<std::endl;
/*
  boost::shared_ptr<pcl::PassThrough<pcl::PointXYZRGB>> pass_through(new pcl::PassThrough<pcl::PointXYZRGB>);
  pass_through->setInputCloud (pointCloud);
  pass_through->setFilterFieldName ("z");
  pass_through->setFilterLimits (1., 2.5);
  //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> thresholded (new pcl::PointCloud<pcl::PointXYZRGB>);
  pass_through->filter (*pointCloud);*/

   //pointCloud=thresholded;

   //std::cout<<" pointCloud point number after:" <<pointCloud->size()<<std::endl;
  // pcl::io::savePCDFile("pointCloud.pcd",*pointCloud,true);

//  pointCloud.reset (new pcl::PointCloud<pcl::PointXYZRGB> (*thresholded));
 // *pointCloud=*thresholded;
   pcl::transformPointCloud (*pointCloud, *pointCloud, transform);

  // std::cout<<" pointCloud point number after:" <<pointCloud->size()<<std::endl;
//thresholded.reset();
}

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
  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> clouds_1(kinect2_count);

  //std::map<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloudPairs;
  //cloudPairs.insert ( std::pair<char,int>('a',100) );

 //pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB> ), source, target,targetToSrc;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
  std::vector<Eigen::Matrix4f> Transforms;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
//  viewer->setBackgroundColor (0, 0, 0);
  viewer->setBackgroundColor (0,204,204);

  // Generate all the kinect grabbers
  K2G_generator kinect_generator(freenectprocessor, mirroring, argv);
  std::generate(kinects.begin(), kinects.end(), kinect_generator);

  
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> allPointClouds(new pcl::PointCloud<pcl::PointXYZRGB>);
 // boost::shared_ptr<pcl::PCLPointCloud2 > allPointClouds;
  //viewer->resetCamera();//moveAllPointClouds();

 //viewer->resetStoppedFlag ();

//viewer->initCameraParameters ();


//std::cout<<"Transforms size "<<Transforms.size();

//std::ofstream FILE("./output.transforms", std::ios::out | std::ofstream::binary);
//std::copy(Transforms.begin(), Transforms.end(), std::ostreambuf_iterator<Eigen::Matrix4f>(FILE));




if (is_file_exist("output.transforms") ) {

size_t sz ;
fstream FILE("output.transforms",ios::binary|ios::in);
//while (binary_file.good()) {
FILE.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
std::cout<<"sz "<<sz<<std::endl;
Transforms.resize(sz);
FILE.read(reinterpret_cast<char*>(&Transforms[0]), sz * sizeof(Transforms[0]));
FILE.close();




}  else return 0;
  

for (int i=0;i<Transforms.size();i++)  std::cout<<"Transform "<<i<< "\n"<<Transforms.at(i)<<std::endl;

  for(size_t i = 0; i < kinect2_count; ++i)
  {
  	
    //clouds_1[i] = kinects[i]->getCloud();
    //clouds[i]=thresholdDepth (clouds_1[i], 0., 2);
    clouds[i] = kinects[i]->getCloud();
    if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );
  	clouds[i]->sensor_orientation_.w() = 0.0;
  	clouds[i]->sensor_orientation_.x() = 1.0;
  	clouds[i]->sensor_orientation_.y() = 0.0;
  	clouds[i]->sensor_orientation_.z() = 0.0; 
    //pcl::copyPointCloud(*clouds[i],*allPointClouds);
   *allPointClouds+=*clouds[i];
  	
  }
  
  viewer->addPointCloud<pcl::PointXYZRGB>(allPointClouds, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(allPointClouds), "allPointClouds");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "allPointClouds");
  //  viewer->spin();

  // Add keyboards callbacks
//  PlySaver ps(clouds, false, false, kinects);
 // viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
   std::cout << "starting cycle" << std::endl;

   std::chrono::high_resolution_clock::time_point tnow, tpost;


//viewer->resetCameraViewpoint("sample cloud_1");


  // Start visualization cycle

  //return 0; 
//  allPointClouds.reset();

//MLS Method

   /*  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
      mls.setSearchRadius(0.01);
      mls.setPolynomialFit(true);
      mls.setPolynomialOrder(2);
      mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
     mls.setUpsamplingRadius(0.005);
      mls.setUpsamplingStepSize(0.003);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr allPointCloudsSmoothed (new pcl::PointCloud<pcl::PointXYZRGB>());*/
     
     // cout << "MLS complete" << endl;
     // pcl::io::savePCDFile ("mesh_mls.pcd", *cloud_smoothed,true);


// 
/*//GREEDY PROJECTION 

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal > norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (50);
  
  norm_est.setInputCloud (cloud);
  norm_est.compute (*cloud_with_normals);
  pcl::copyPointCloud (*cloud, *cloud_with_normals);



  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
 // pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
 // pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.05);

  // Set typical values for the parameters
  gp3.setMu (2.);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
//  pcl::io::savePCDFile ("cloud_with_normals.pcd", *cloud_with_normals, true);
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
*/
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>  );

  while(!viewer->wasStopped()) {

    allPointClouds.reset(new pcl::PointCloud<pcl::PointXYZRGB>);  
    // clouds[0].reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
    //     clouds[1].reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
    // clouds[2].reset(new pcl::PointCloud<pcl::PointXYZRGB>); 
    // clouds[3].reset(new pcl::PointCloud<pcl::PointXYZRGB>); 

   // allPointCloudsSmoothed.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer->spinOnce ();

    tnow = std::chrono::high_resolution_clock::now();



     /*for(size_t i = 0; i < kinect2_count; ++i) {
      
      clouds[i] = kinects[i]->updateCloud(clouds[i]);
      if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

      }*/

   
    //  std::vector<int> indices; 


   // pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

      boost::thread_group threads;

      for(size_t i = 0; i < kinect2_count; ++i) {
       // clouds[i] = kinects[i]->updateCloud(clouds[i]);
       // threads.create_thread(boost::bind(threadPointCloud,kinects[i],clouds[i]));

         if (i>0)  threads.create_thread(boost::bind(threadPointCloud,kinects[i],boost::ref(clouds[i]),Transforms.at(i-1)));

          else threads.create_thread(boost::bind(threadPointCloud,kinects[i],boost::ref(clouds[i]),Eigen::Matrix4f::Identity ()));

      }

    //  threads.interrupt_all();
      threads.join_all();

    for(size_t i = 0; i < kinect2_count; ++i)  *allPointClouds+=*clouds[i];
      

    /*  boost::thread threads[kinect2_count];

       for(int i = 0; i < kinect2_count; i++) {
        if (i>0)  threads[i] =  boost::thread(boost::bind(threadPointCloud,kinects[i],boost::ref(clouds[i]),Transforms.at(i-1)));
      
        else threads[i] =  boost::thread(boost::bind(threadPointCloud,kinects[i],boost::ref(clouds[i]),Eigen::Matrix4f::Identity ()));

      }

      for(int i = 0; i < kinect2_count; i++) {
        threads[i].join();
        //delete threads[i];
       // *allPointClouds+=*clouds[i];  

        //delete threads[i];
      //  delete threads[i];
    }

    //delete [] threads;
    for(size_t i = 0; i < kinect2_count; ++i) {
      *allPointClouds+=*clouds[i]; 
      //delete threads[i];
    }*/
/*
      for(size_t i = 0; i < kinect2_count; ++i) { 
       // if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

      //  std::cout<<"point size :"<<clouds[i]->size()<<std::endl;
       *allPointClouds+=*clouds[i];  

        delete threads[i];
     }*/
/*
     for(size_t i = 0; i < kinect2_count; ++i) { 
        clouds_1[i] = kinects[i]->updateCloud(clouds_1[i]); 
        clouds[i]= thresholdDepth (clouds_1[i], .4, 5.1);
       // clouds[i]=removeOutliers(clouds[i],0.05,50);

        // clouds[i] = kinects[i]->updateCloud(clouds[i]); 
        //indices.clear();
        //  pcl::removeNaNFromPointCloud(*clouds[i],*clouds[i], indices);

          if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );
          *allPointClouds+=*clouds[i];

      }
    */

 /* std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

    for(size_t i = 0; i < kinect2_count-3; ++i) {

      kdtree.setInputCloud(clouds[i+1]);
      

      for (pcl::PointCloud< pcl::PointXYZRGB >::const_iterator idx=clouds[i]->begin();idx != clouds[i]->end (); ++idx) {
       // std::cout<<"idx->x() "<<idx->x;
        pointIdxRadiusSearch.clear();pointRadiusSquaredDistance.clear();
         if ( kdtree.radiusSearch (*idx, 0.05, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) {

          // for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j) {
               pcl::ExtractIndices< pcl::PointXYZRGB > eifilter (true);
                eifilter.setInputCloud (clouds[i+1]);
                boost::shared_ptr <std::vector<int> > indices(new std::vector<int>(pointIdxRadiusSearch));
                eifilter.setIndices(indices);
                eifilter.setNegative (true);
                eifilter.setKeepOrganized (true);
               // eifilter.setIndices (std::make_shared<std::vector<int>>(pointIdxRadiusSearch));
                eifilter.filter (*clouds[i+1]);


               // eifilter.filterDirectly (cloud_in);
         //  }

            //clouds[i+1]->points.erase(clouds[i+1]->points.begin()+ pointIdxRadiusSearch[j] );

       //  std::cout<<" cloud[i+1] size"<<clouds[i+1]->size()<<std::endl;
          // for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j)
          //       std::cout << "    "  << clouds[i+1]->points[ pointIdxRadiusSearch[j] ].x 
          //             << " " << clouds[i+1]->points[ pointIdxRadiusSearch[j] ].y 
          //             << " " << clouds[i+1]->points[ pointIdxRadiusSearch[j] ].z 
          //             << " (squared distance: " << pointRadiusSquaredDistance[j] << ")" << std::endl;
           }



       }

     

     //kdtree.setInputCloud(*clouds[i+1])



      }*/

    //*allPointClouds=*clouds[1];

      //allPointClouds= downsample (allPointClouds, 0.08);
   // allPointClouds = thresholdDepth (allPointClouds, 0.6, 2.2);

     // mls.setInputCloud(allPointClouds);

     // mls.process(*allPointCloudsSmoothed);

   // pcl::io::savePCDFile("allPointClouds.pcd",*allPointClouds,true);




     viewer->updatePointCloud<pcl::PointXYZRGB>(allPointClouds, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (allPointClouds), "allPointClouds");
      tpost = std::chrono::high_resolution_clock::now();
      std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;



  }

  // Close all kinect grabbers
  for(auto & k : kinects)
  	k->shutDown();

  return 0;
}

