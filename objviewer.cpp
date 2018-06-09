#include <pcl/conversions.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
using namespace pcl;

typedef PointCloud<PointXYZ> PC; 

int main(int argc, char **argv) {
  if(argc != 2) {
    cout << "Usage: ./show_obj_file <obj_filename>";
    return -1; 
  }

// pcl::TextureMesh mesh1;
// pcl::io::loadPolygonFileOBJ (argv[1], mesh1);
 pcl::TextureMesh mesh2;
 OBJReader obj;
 obj.read(argv[1], mesh2);
 int sizet=mesh2.tex_coordinates[0].size();
 int sizett=mesh2.tex_polygons[0].size();
 std::cout<<"size "<<sizet<<" "<<sizett<<std::endl;
 // pcl::io::loadOBJFile (argv[1], mesh2);
// mesh1.tex_materials = mesh2.tex_materials;
 pcl::visualization::PCLVisualizer visu("Test");
 visu.addTextureMesh (mesh2,"texture");
 visu.spin ();


  return 0;
}
