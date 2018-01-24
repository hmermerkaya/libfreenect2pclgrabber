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
//#include "filters.h"
//#include "segmentation.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/transforms.h>

#include <boost/thread/thread.hpp>
//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloud;
//typedef pcl::PointNormal PointNormalT;
//typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//typedef std::pair<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>>() makePair;
//typedef pcl::PointCloud<PointNT>::Ptr PointCloudPtr;

/*struct threadPointCloud {

threadPointCloud(boost::shared_ptr<K2G> kinect, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pointCloud):
pointCloud_(pointCloud) {

  kinect=kinect_;
}

  

void operator() () {

  pointCloud_ = kinect_->updateCloud(pointCloud_);

}


boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>  pointCloud_;

boost::shared_ptr<K2G> kinect_;


};*/
void threadPointCloud (K2G* kinect, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pointCloud  ,Eigen::Matrix4f transform) {


      pointCloud = kinect->updateCloud(pointCloud);

    pcl::transformPointCloud (*pointCloud, *pointCloud, transform);

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
  //std::map<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloudPairs;
  //cloudPairs.insert ( std::pair<char,int>('a',100) );

 //pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB> ), source, target,targetToSrc;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
  std::vector<Eigen::Matrix4f> Transforms;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  // Generate all the kinect grabbers
  K2G_generator kinect_generator(freenectprocessor, mirroring, argv);
  std::generate(kinects.begin(), kinects.end(), kinect_generator);

 


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
  	//if (i==1) continue;
    clouds[i] = kinects[i]->getCloud();
    if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

  	clouds[i]->sensor_orientation_.w() = 0.0;
  	clouds[i]->sensor_orientation_.x() = 1.0;
  	clouds[i]->sensor_orientation_.y() = 0.0;
  	clouds[i]->sensor_orientation_.z() = 0.0; 

  	viewer->addPointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(clouds[i]), "sample cloud_" + i);
  	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud_" + i);
  }

  // Add keyboards callbacks
  PlySaver ps(clouds, false, false, kinects);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
   std::cout << "starting cycle" << std::endl;

   std::chrono::high_resolution_clock::time_point tnow, tpost;


  //viewer->resetCameraViewpoint("sample cloud_1");


  // Start visualization cycle

   float sum=0;
   int say=0;
  

  while(!viewer->wasStopped()){
    say++;
    viewer->spinOnce ();

    tnow = std::chrono::high_resolution_clock::now();



     /*for(size_t i = 0; i < kinect2_count; ++i) {
      
      clouds[i] = kinects[i]->updateCloud(clouds[i]);
      if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

      }*/

    boost::thread_group threads;

    for(size_t i = 0; i < kinect2_count; ++i) {
      //  if (i==1) continue;
     // clouds[i] = kinects[i]->updateCloud(clouds[i]);
     // threads.create_thread(boost::bind(threadPointCloud,kinects[i],clouds[i]));
       if (i>0)  threads.create_thread(boost::bind(threadPointCloud,kinects[i],clouds[i],Transforms.at(i-1)));
       else threads.create_thread(boost::bind(threadPointCloud,kinects[i],clouds[i],Eigen::Matrix4f::Identity ()));

    }

    threads.join_all();

      if (say%5000000==0) { cv::Mat color;
        std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
        std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
       for(size_t i = 0; i < kinect2_count; ++i) {   kinects[i]->getColor(color);
         cv::imwrite("color_" + std::to_string(i) + "_" + now + ".jpg", color);
       }    
     }
   tpost = std::chrono::high_resolution_clock::now();
 //std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;

  sum=sum+std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000;
   std::cout<<"average: "<<sum/say<<std::endl;

   for(size_t i = 0; i < kinect2_count; ++i) {
    
 // //  if ( i>0) pcl::transformPointCloud (*clouds[i], *clouds[i], Transforms.at(i-1) );

    viewer->updatePointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (clouds[i]), "sample cloud_" + i);

  }

  


  }

  // Close all kinect grabbers
  for(auto & k : kinects)
  	k->shutDown();

  return 0;
}

