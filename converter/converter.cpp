#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_io.h> //need this for savevtkfile

//for reconstructing mesh and saving to vtk
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>





#include <iostream>
#include <pcl/point_types.h>

#include<vtkPolyDataReader.h>
#include<vtkSmartPointer.h>
#include<vtkPolyData.h>
#include<vtkPoints.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


///read in a vtk file, export as a pcl pointcloud and a generate a mesh to write as a vtk file

int main (int argc, char** argv)
{
std::string inputfile=argv[1];
std::string outfile=argv[2];




pcl::PolygonMesh mesh;

vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();

cout<<inputfile;
reader->SetFileName (inputfile.c_str());
reader->Update();
poly_data = reader->GetOutput();

print_info("read it");

int nr_points = poly_data->GetNumberOfPoints();
cout<<nr_points;

print_info("test");


print_info("convertin");

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
cloud_temp->points.resize(nr_points);
double point_xyz[3];
int i =0;
for (i = 0; i < poly_data->GetNumberOfPoints(); i++)
{
  poly_data->GetPoint(i, &point_xyz[0]);
  cloud_temp->points[i].x = (float)(point_xyz[0]);
  cloud_temp->points[i].y = (float)(point_xyz[1]);
  cloud_temp->points[i].z = (float)(point_xyz[2]);
}
cloud_temp->width = cloud_temp->points.size();
cloud_temp->height = 1;
cloud_temp->is_dense = true;

cout<<cloud_temp->points.size();


print_info ("done");


pcl::PCDWriter w;
w.writeASCII<pcl::PointXYZ> (outfile.c_str(), *cloud_temp);





// Load input file into a PointCloud<T> with an appropriate type
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
cloud=cloud_temp;

// Normal estimation*
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud (cloud);
n.setInputCloud (cloud);
n.setSearchMethod (tree);
n.setKSearch (20);
n.compute (*normals);
//* normals should not contain the point normals + surface curvatures

// Concatenate the XYZ and normal fields*
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
//* cloud_with_normals = cloud + normals

// Create search tree*
pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
tree2->setInputCloud (cloud_with_normals);

// Initialize objects
pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
pcl::PolygonMesh triangles;

// Set the maximum distance between connected points (maximum edge length)
gp3.setSearchRadius (0.025);

// Set typical values for the parameters
gp3.setMu (2.5);
gp3.setMaximumNearestNeighbors (100);
gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
gp3.setMinimumAngle(M_PI/18); // 10 degrees
gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
gp3.setNormalConsistency(false);

// Get result
gp3.setInputCloud (cloud_with_normals);
gp3.setSearchMethod (tree2);
gp3.reconstruct (triangles);


pcl::io::saveVTKFile ("cloud.vtk", triangles);




  return (0);
}
