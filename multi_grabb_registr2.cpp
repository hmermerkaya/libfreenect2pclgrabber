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
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

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

void readTransform(const std::string &file,Eigen::Matrix4f &transform ){



      fstream binary_file(file.c_str(),ios::binary|ios::in);
      //while (binary_file.good()) {

      binary_file.read(reinterpret_cast<char *>(&transform),sizeof(Eigen::Matrix4f));
      //}

      binary_file.close();


      std::cout<<"tranform read \n "<<transform<<std::endl;

  }


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
  //our visualizer
  pcl::visualization::PCLVisualizer *p;
  //its left and right viewports
  int vp_1, vp_2;




bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}


boost::mutex io_mutex;

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
 

  p = new pcl::visualization::PCLVisualizer (argc, argv, " Registration example");
   p->setBackgroundColor (0, 0, 0);
  //Eigen::Affine3f pose1;
  //p->addCoordinateSystem( 1, pose1 ); 

//  p->createViewPort (0.0, 0, 0.01, 1.0, vp_1);
  //p->createViewPort (0.01, 0, 1.0, 1.0, vp_2);

 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
 // viewer->setBackgroundColor (0, 0, 0);

  // Generate all the kinect grabbers
  K2G_generator kinect_generator(freenectprocessor, mirroring, argv);
  std::generate(kinects.begin(), kinects.end(), kinect_generator);

  // Initialize clouds and viewer viewpoints
  PointCloud::Ptr result (new PointCloud), source, target,targetToSrc;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;


  //std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  //loadData (argc, argv, data);



  if (is_file_exist("output.transform") ) 
   readTransform("output.transform", GlobalTransform);
 else return 0;
std::cout <<"GlobalTransform \n" << GlobalTransform<<std::endl;

  for(size_t i = 1; i < kinect2_count; ++i)
  {
  	
  //  boost::mutex::scoped_lock scoped_lock(io_mutex);

    source = kinects[i-1]->getCloud();
    target = kinects[i]->getCloud();
    //pcl::io::savePCDFile ("source.pcd", *source, true);
   // pcl::io::savePCDFile ("target.pcd", *target, true);

    //  std::vector<int> indices;
     // pcl::removeNaNFromPointCloud(*source, *source, indices);
    //  indices.clear();
   //   pcl::removeNaNFromPointCloud(*target, *target, indices);

   //   showCloudsLeft(source, target);

     //PointCloud::Ptr temp (new PointCloud);
  //   PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
     //pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
  //  pcl::transformPointCloud (*source, *source, GlobalTransform);

    //update the global transform
   // GlobalTransform = GlobalTransform * pairTransform;
 // std::cout <<"GlobalTransform \n" << GlobalTransform<<std::endl;
 // std::cout <<"pairTransform \n" << pairTransform<<std::endl;

    //save aligned pair, transformed into the first cloud's frame
   // std::stringstream ss;
   // ss << i << ".pcd";
   // pcl::io::savePCDFile (ss.str (), *result, true);

   
 
  	 source->sensor_orientation_.w() = 0.0;         
  	 source->sensor_orientation_.x() = 1.0;    
  	 source->sensor_orientation_.y() = 0.0;        
  	 source->sensor_orientation_.z() = 0.0;  

    target->sensor_orientation_.w() = 0.0;
     target->sensor_orientation_.x() = 1.0;
     target->sensor_orientation_.y() = 0.0;
     target->sensor_orientation_.z() = 0.0; 

    /*source->sensor_orientation_.w() = 0.7071068;
     source->sensor_orientation_.x() = 0.0;
    source->sensor_orientation_.y() = 0.7071068;
    source->sensor_orientation_.z() = 0.0;


     target->sensor_orientation_.w() = 0.7071068;
     target->sensor_orientation_.x() = 0.0;
     target->sensor_orientation_.y() = 0.7071068;
     target->sensor_orientation_.z() = 0.0;
*/
  	 p->addPointCloud<pcl::PointXYZRGB>(source, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(source), "source");

  	 p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source");

   p->addPointCloud<pcl::PointXYZRGB>(target, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(target), "target");

    p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target");

  }


  



  // Add keyboards callbacks
  
  PlySaver ps(clouds, false, false, kinects);
  p->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
  std::cout << "starting cycle" << std::endl;

  std::chrono::high_resolution_clock::time_point tnow, tpost;


  // Start visualization cycle
  while(!p->wasStopped()){

    p->spinOnce (10);

    tnow = std::chrono::high_resolution_clock::now();

    //for(size_t i = 0; i < kinect2_count; ++i)
    //	clouds[i] = kinects[i]->updateCloud(clouds[i]);
     source =kinects[0]->updateCloud(source);
      target =kinects[1]->updateCloud(target);//->makeShared();;

      pcl::transformPointCloud (*source, *source,GlobalTransform );

    tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;

    //for(size_t i = 0; i < kinect2_count; ++i)
    	p->updatePointCloud<pcl::PointXYZRGB>(source, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (source), "source");
  
     p->updatePointCloud<pcl::PointXYZRGB>(target, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (target), "target");

  }




  /*while (true) {

    tnow = std::chrono::high_resolution_clock::now();
    source =kinects[0]->updateCloud(source);
      target =kinects[1]->updateCloud(target);//->makeShared();;
    tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;

  }*/

  // Close all kinect grabbers
 for(auto & k : kinects)  	k->shutDown();

  return 0;
}
