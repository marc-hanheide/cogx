//#include "stable_planes.hpp"
//#include <sampling.hpp>
//#include <Eigen/StdVector>
#include <alignment_ros_wrapper/stable_planes_alignment.h>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/passthrough.h"

int
main (int argc, char ** argv)
{
  //ALIGN one 3D MESH with a partial view
  vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
  readerQuery->SetFileName (argv[1]);
  vtkSmartPointer < vtkPolyData > polydata1 = readerQuery->GetOutput ();
  polydata1->Update ();

  //load pcd file
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_file(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(argv[2], *pcd_file);

  //cut pcd_1...
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcd_file);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.2, 10);
  pass.filter (*pcd_file);

  //scale pcd_file
  double scale_factor=0.1;
  for(size_t i=0; i < pcd_file->points.size(); i++) {
    pcd_file->points[i].getVector4fMap() = pcd_file->points[i].getVector4fMap() * scale_factor;
  }

  StablePlanesAlignment spa(5000);
  spa.setName2(argv[1]);

  Eigen::Matrix4f transform;

  Eigen::Vector4f CoM;
  pcl::compute3DCentroid (*pcd_file, CoM);

  Plane plane_1;
  plane_1.center[0] = CoM[0];
  plane_1.center[1] = 0;
  plane_1.center[2] = CoM[2];

  plane_1.normal[0] = 0;
  plane_1.normal[1] = -1;
  plane_1.normal[2] = 0;

  StablePlanesAlignment spa(15000, 0, false);
  spa.alignPartialViewWithMesh(*pcd_file, plane_1, polydata1, transform);

  return 0;
}
