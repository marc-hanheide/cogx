/*
 * main.cpp
 *
 *  Created on: Jul 18, 2011
 *      Author: aitor
 */

#include <stable_planes.hpp>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextSource.h>
#include <vtkVectorText.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>


int main(int argc, char *argv[])
{

  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (argv[1]);

  vtkPolyDataMapper * mapper = vtkPolyDataMapper::New();
  mapper->SetInputConnection(reader->GetOutputPort());
  mapper->Update();

  std::vector<Plane> stable_planes_1;

  stablePlanes sp;
  sp.setThreshold (0.01);
  sp.setVisualize (false);

  //check if file exists
  sp.setFilenameForCache (getName (argv[1]));
  bool cache_exist = sp.getStablePlanesFromCache (stable_planes_1,getName (argv[1]));

  if (!cache_exist)
  {
    //std::cout << "polydata 1 is not null" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ> ());
    getVerticesAsPointCloud (mapper->GetInput(), *cloud_out);
    sp.setInputCloud (cloud_out);
    sp.compute (stable_planes_1);
  }
  else
  {
    std::cout << "Stable planes for polydata 1 load from CACHE" << std::endl;
  }

}
