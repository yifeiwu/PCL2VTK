#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>


#include <iostream>
#include <pcl/point_types.h>

#include<vtkPolyDataReader.h>
#include<vtkSmartPointer.h>
#include<vtkPoints.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;






int
main (int argc, char** argv)
{
pcl::PolygonMesh mesh;

  vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
  reader->SetFileName (argv[0]);
  reader->Update();
  poly_data = reader->GetOutput ();
  
  print_info("read it");
  
  int nr_points = poly_data->GetNumberOfPoints();

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














print_info ("done");
std::string s = "test.pcd";

pcl::PCDWriter w;
w.writeASCII<pcl::PointXYZ> (s, *cloud_temp);
//pcl::io::savePCDFileASCII(argv[1], *cloud_temp);



  return (0);
}
