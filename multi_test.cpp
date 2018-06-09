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
#include <pcl/io/png_io.h>
int maxIr, minIr;


void findMinMax(const cv::Mat &ir)
  {
    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *it = ir.ptr<uint16_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++it)
      {
        minIr = std::min(minIr, (int) * it);
        maxIr = std::max(maxIr, (int) * it);
      }
    }
}
void convertIr(const cv::Mat &ir, cv::Mat &grey)
  {
    maxIr=0xFFFF;
    minIr=0;
    cv::Ptr<cv::CLAHE> clahe;
    clahe = cv::createCLAHE(1.5, cv::Size(32, 32));
    const float factor = 255.0f / (maxIr - minIr);
    grey.create(ir.rows, ir.cols, CV_8U);

  //  #pragma omp parallel for
    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *itI = ir.ptr<uint16_t>(r);
      uint8_t *itO = grey.ptr<uint8_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++itI, ++itO)
      {
        *itO = std::min(std::max(*itI - minIr, 0) * factor, 255.0f);
      }
    }
    clahe->apply(grey, grey);
  }

void readTransform(const std::string &file,Eigen::Matrix4f &transform ){


fstream binary_file(file.c_str(),ios::binary|ios::in);
//while (binary_file.good()) {

binary_file.read(reinterpret_cast<char *>(&transform),sizeof(Eigen::Matrix4f));
//}

binary_file.close();


std::cout<<"tranform read \n "<<transform<<std::endl;

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
      cv::Mat color,depth, bigmat, bigmat_scaled, bigmat_grey;
      for(size_t i = 0; i < s->kinects_.size(); ++i){
      	writer.write ("cloud_"+ std::to_string(i) + "_" + now + ".ply", *(s->clouds_[i]), s->binary_, s->use_camera_);
        s->kinects_[i]->get(color, depth, bigmat, false);
      	//cv::imwrite("color_" + std::to_string(i) + "_" + now + ".jpg", color);
        // cv::imwrite("bigmat_" + std::to_string(i) + "_" + now + ".exr", bigmat);
         // cv::imwrite("depth_" + std::to_string(i) + "_" + now + ".exr", depth);
         
         //cv::resize(bigmat, bigmat_scaled, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
         //findMinMax(bigmat);
         //convertIr(bigmat, bigmat_grey);
         cv::imwrite(s->kinects_[i]->getSerial()+"_"+std::to_string(i)+".png", color);
       //  cv::imwrite("ir_" + std::to_string(i) + ".png", bigmat_grey);
       ///cv::imwrite("depth_" + std::to_string(i) + ".exr", bigmat);
       //  cv::imwrite("depth_"+std::to_string(i) + ".exr", depth);
      
          depth.convertTo(depth, CV_16UC1);
          cv::imwrite(s->kinects_[i]->getSerial()+"_"+std::to_string(i)+ "_depth.png", depth);
      //   cv::imwrite(s->kinects_[i]->getSerial()+"_"+std::to_string(i)+ ".exr", depth);
       // cv::FileStorage file("bigmat_" + std::to_string(i) + "_" + now + ".xml", cv::FileStorage::WRITE);
         

         //file << bigmat;
        //std::vector<int> compression_params;
        //compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
       // compression_params.push_back(9);
     //   cv::imwrite("color_" + std::to_string(i) + "_" + now + ".png", color,compression_params);
      //   pcl::io::saveRgbPNGFile ("color_" + std::to_string(i) + "_" + now + ".png", color,512,424);
         
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
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  // Generate all the kinect grabbers
  K2G_generator kinect_generator(freenectprocessor, mirroring, argv);
  std::generate(kinects.begin(), kinects.end(), kinect_generator);

Eigen::Matrix4f transform;
Eigen::Vector4f vector4f;
vector4f(0)=0;
vector4f(1)=0;
vector4f(2)=0;
vector4f(3)=1;

readTransform("output.transforms",transform);
 
 Eigen::Matrix4f transform2= Eigen::Matrix4f::Identity ();
transform2(0,0)=0;
transform2(0,1)=-1;
transform2(1,0)=1;
transform2(1,1)=0;
transform2(0,3)=1;

 std::cout<<"transform  \n"<<transform2<<std::endl;

 std::cout<<"transform inverse \n"<<transform2.inverse()<<std::endl;





  // Initialize clouds and viewer viewpoints
  for(size_t i = 0; i < kinect2_count; ++i)
  {
  	clouds[i] = kinects[i]->getCloud();

  //	clouds[i]->sensor_orientation_.w() = 0.0;
  //	clouds[i]->sensor_orientation_.x() = 1.0;
  //	clouds[i]->sensor_orientation_.y() = 0.0;
  //	clouds[i]->sensor_orientation_.z() = 0.0; 

  	viewer->addPointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(clouds[i]), "sample cloud_" + i);
  	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud_" + i);
  }

  // Add keyboards callbacks
  PlySaver ps(clouds, true, false, kinects);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
  std::cout << "starting cycle" << std::endl;

  std::chrono::high_resolution_clock::time_point tnow, tpost, tpost_prev;
  tpost_prev=std::chrono::high_resolution_clock::now();
  std::vector<pcl::visualization::Camera> cam; 
  // Start vqisualization cycle
  while(!viewer->wasStopped()){

    viewer->spinOnce ();
   
   // cout<< viewer->getViewerPose().linear();
   // viewer->getCameras(cam); 

    // cout << "Cam: " << endl 
    //         << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl 
    //          << " - view: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl 
    //          << " - focal: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl;



    tnow = std::chrono::high_resolution_clock::now();

    for(size_t i = 0; i < kinect2_count; ++i) {
    	clouds[i] = kinects[i]->updateCloud(clouds[i]);
   //   std::cout << "<<< time stamp "<< i <<" : " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count()  << std::endl;

    }
       
    //   cout<<" Point Z "<<clouds[0]->points[2340].z<<endl;
    tpost = std::chrono::high_resolution_clock::now();
   // std::cout << "<<< delta: " << std::chrono::duration_cast<std::chrono::milliseconds>(tpost.time_since_epoch()).count()-std::chrono::duration_cast<std::chrono::milliseconds>(tpost_prev.time_since_epoch()).count()<<std::endl;
   
    tpost_prev=tpost;
   // std::cout << "<<< delta  : " << std::chrono::duration_cast<std::chrono::milliseconds>(tpost.time_since_epoch()).count()-std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()).count();
   // std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;
     
    for(size_t i = 0; i < kinect2_count; ++i)
  	viewer->updatePointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (clouds[i]), "sample cloud_" + i);
  }

  // Close all kinect grabbers
  for(auto & k : kinects)
  	k->shutDown();

  return 0;
}

