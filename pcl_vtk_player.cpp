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
#include <pcl/io/vtk_lib_io.h>

#ifdef WITH_SERIALIZATION
#include "serialization.h"
#endif
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <iostream>
#include <iterator>
#include <string>

using namespace boost::filesystem;



int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
 
  Processor freenectprocessor = CUDA;
 // boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

 // pcl::PolygonMesh mesh;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0,201,204);

 // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  path p(argc>1? argv[1] : ".");

 /*   if(is_directory(p)) {
        std::cout << p << " is a directory containing:\n";

        for(auto& entry : boost::make_iterator_range(directory_iterator(p), {}))
            std::cout << entry << "\n";
   }*/

 /* std::vector<directory_entry> v;

 if(is_directory(p)) {

   copy(directory_iterator(p), directory_iterator(), back_inserter(v));
        std::cout << p << " is a directory containing:\n";
        int i=0;
    std::vector<directory_entry>::const_iterator it = v.begin();  
 while(!viewer->wasStopped()) {
     viewer->spinOnce ();
      //  for ( std::vector<directory_entry>::const_iterator it = v.begin(); it != v.end();  ++ it )
      //  {
            std::cout<< (*it).path().string()<<endl;
             
              std::string file=(*it).path().string();

            boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
              if ( it == v.end() ||  pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud) == -1) //* load the file
                {
                //  PCL_ERROR ("Couldn't read pcd file  \n");
                  continue;
                }

                  cloud->sensor_orientation_.w() = 0.0;
                    cloud->sensor_orientation_.x() = 1.0;
                    cloud->sensor_orientation_.y() = 0.0;
                    cloud->sensor_orientation_.z() = 0.0;

              if (i==0) {
                    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb,"sample cloud");
               viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
               //  viewer->addCoordinateSystem (1.0);
               //  viewer->initCameraParameters ();

               }
                else   {

                  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud,rgb,"sample cloud"); 
                }
              //  viewer->removePointCloud("sample cloud");
              //  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");    

              i++;
              ++ it;
        }    

  }
*/

std::chrono::high_resolution_clock::time_point tnow, tpost;

boost::shared_ptr<pcl::PolygonMesh> model_polygon_mesh(new pcl::PolygonMesh); 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>); 


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
        while(!viewer->wasStopped()) {
      //  for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
      //  {

              cout << "   " << (*it) << '\n';
              std::string file = (*it) .string();

            //    viewer->spinOnce ();
      //  for ( std::vector<directory_entry>::const_iterator it = v.begin(); it != v.end();  ++ it )
      //  {
            
              tnow = std::chrono::high_resolution_clock::now();

             // boost::this_thread::sleep (boost::posix_time::milliseconds (250));  
              //if ( it == v.end() ||    pcl::io::loadPolygonFilePLY (file, *model_polygon_mesh)==-1) //* load the file

              if ( it == v.end() ||    pcl::io::loadPolygonFile(file, *model_polygon_mesh)==-1) //* load the file
                {
                  PCL_ERROR ("Couldn't read pcd file  \n");
                  continue;
                }

 /*                 cloud->sensor_orientation_.w() = 0.0;
                    cloud->sensor_orientation_.x() = 1.0;
                    cloud->sensor_orientation_.y() = 0.0;
                    cloud->sensor_orientation_.z() = 0.0;
*/



             
              pcl::fromPCLPointCloud2 (model_polygon_mesh->cloud, *cloud); 

               viewer->addPolygonMesh(*model_polygon_mesh,file); 
                if (i==0) viewer->addPointCloud(cloud,"Frame");
                else viewer->updatePointCloud(cloud,"Frame"); 
; 
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Frame"); 
                 viewer->spinOnce (100);

              viewer->removePolygonMesh(file); 

             // pcl::fromROSMsg(model_polygon_mesh->cloud, *cloud); 
              
              /*if (i==0) {
                //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb,"sample cloud");
                //viewer->addPolygonMesh(*model_polygon_mesh,"polygon");
                viewer->addPolygonMesh<pcl::PointXYZRGB> (cloud, model_polygon_mesh->polygons,"polygon",0); 
               // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
               //  viewer->addCoordinateSystem (1.0);
                 viewer->initCameraParameters ();

               }
                else   {

                 /// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
                  //viewer->updatePointCloud<pcl::PointXYZRGB> (cloud,rgb,"sample cloud"); 
                //  viewer->updatePolygonMesh(*model_polygon_mesh, "polygon");
                    viewer->updatePolygonMesh<pcl::PointXYZRGB> (cloud, model_polygon_mesh->polygons,"polygon");
                }
*/
                //if (!viewer->updatePolygonMesh<pcl::PointXYZRGB> (cloud, model_polygon_mesh->polygons, "polygon")) 
                  //     viewer->addPolygonMesh<pcl::PointXYZRGB> (cloud, model_polygon_mesh->polygons, "polygon", 0); 


              //  viewer->removePointCloud("sample cloud");
              //  viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");    
             tpost = std::chrono::high_resolution_clock::now();
              std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<float>>(tpost - tnow).count() * 1000 << std::endl;



              i++;
              ++ it;
         

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




/*
  if(argc > 1){
      freenectprocessor = static_cast<Processor>(atoi(argv[1]));
  }



   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

    
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();

  k2g.printParameters();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  PlySaver ps(cloud, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  cv::Mat color, depth;

  while(!viewer->wasStopped()){

    viewer->spinOnce ();
    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    k2g.get(color, depth, cloud);
    // Showing only color since depth is float and needs conversion
    cv::imshow("color", color);
    int c = cv::waitKey(1);
    
    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");    	

  }

  k2g.shutDown();*/
  return 0;
}

