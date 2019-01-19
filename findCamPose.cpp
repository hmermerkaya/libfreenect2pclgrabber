#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/vtk_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tuple>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/algorithm/string.hpp>


bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

void readTransform(const std::string &file,Eigen::Matrix4f &transform ){



  fstream binary_file(file.c_str(),ios::binary|ios::in);
  //while (binary_file.good()) {

  binary_file.read(reinterpret_cast<char *>(&transform),sizeof(Eigen::Matrix4f));
  //}

  binary_file.close();


  std::cout<<"tranform read \n "<<transform<<std::endl;

}

void readTransformFromText(const std::string &file, Eigen::Matrix4f &transform ){



  //fstream infile(file.c_str(), ios::in );
  //while (binary_file.good()) {
   
   std::ifstream infile(file.c_str());
    std::stringstream buffer;

    buffer << infile.rdbuf();
    float temp;
    std::vector<float> matrixElements;
    while (buffer >> temp) {
    matrixElements.push_back(temp);
  //  std::cout<<"temp: "<<temp<<"\n";

    }

     transform= Eigen::Map<Eigen::Matrix<float,4,4,Eigen::RowMajor> >(matrixElements.data());



  /*while (std::getline(infile, line)){
    
    float temp;
    std::istringstream iss(line);
    while (iss >> temp) {
     transform<<temp;
    }
     

  }*/
  //}

  infile.close();


 // std::cout<<"tranform read \n "<<transform<<std::endl;

}

void readTransformsBinary(const std::string &file,  std::vector<Eigen::Matrix4f> &transforms) {

        size_t sz ;
        fstream FILE(file,ios::binary|ios::in);
        //while (binary_file.good()) {
        FILE.read(reinterpret_cast<char*>(&sz), sizeof(size_t));
       // std::cout<<"sz "<<sz<<std::endl;
        transforms.resize(sz);
        FILE.read(reinterpret_cast<char*>(&transforms[0]), sz * sizeof(transforms[0]));
        FILE.close();

}



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
int
main (int argc, char** argv)
{

std::string filename;

if (pcl::console::parse_argument (argc, argv, "-filename", filename) != -1)
	{
	PCL_INFO ("Input filenames given as %s. Batch process mode on.\n",filename.c_str ());
}

Eigen::Matrix4f firstcamTransform= Eigen::Matrix4f::Identity ();
 
firstcamTransform(0,3)=1.5;
firstcamTransform(1,3)=1.5;
firstcamTransform(2,3)=-0.3;

Eigen::Matrix4f calibComb;

readTransformFromText(filename+".calib", calibComb);

Eigen::Matrix4f res=firstcamTransform.inverse()*calibComb;
std::cout<<res<<std::endl;


Eigen::Matrix4f campose;

readTransformFromText(filename+".pose", campose);

Eigen::Matrix4f campose2firstCam;
campose2firstCam=firstcamTransform*campose;

Eigen::Matrix4f calibCampose2firtsCam;
calibCampose2firtsCam=campose2firstCam*res;


std::ofstream fs(filename+".final");
fs<<"TVector\n";
fs<<calibCampose2firtsCam(0,3)<<	"\n";
fs<<calibCampose2firtsCam(1,3)<<	"\n";
fs<<calibCampose2firtsCam(2,3)<<	"\n";
fs<<"\n";
fs<<"RMatrix\n";
fs<<calibCampose2firtsCam(0,0)<<" "<<calibCampose2firtsCam(0,1)<<" "<<calibCampose2firtsCam(0,2)<<"\n"
<<calibCampose2firtsCam(1,0)<<" "<<calibCampose2firtsCam(1,1)<<" "<<calibCampose2firtsCam(1,2)<<"\n"
<<calibCampose2firtsCam(2,0)<<" "<<calibCampose2firtsCam(2,1)<<" "<<calibCampose2firtsCam(2,2)<<"\n";
fs<<"\n";

fs.close();


  return (0);
}
