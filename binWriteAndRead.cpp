#include <iostream>
#include <fstream>
/*
void
saveTransform (const std::string &file, const Eigen::Matrix4d &transform)
{
  ofstream ofs;
  ofs.open (file.c_str (), ios::trunc | ios::binary);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      ofs.write (reinterpret_cast<const char*>(&transform (i, j)), sizeof (double));  
  ofs.close ();
}*/


using namespace std;

int main() {

double transform[2][2]={0.77,7.8,8.9,1.25};
std::ofstream ofs;
  ofs.open ("test.dat", ios::trunc | ios::binary);
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 2; ++j)
      ofs.write (reinterpret_cast<const char*>(&transform[i][j]), sizeof (double));  
  ofs.close ();

double transformBack[2][2];

fstream binary_file("test.dat",ios::binary|ios::in);
//while (binary_file.good()) {

binary_file.read(reinterpret_cast<char *>(&transformBack),sizeof(double)*4);
//}

binary_file.close();


std::cout<<"tranfomBack "<<transformBack[1][1]<<std::endl;
}