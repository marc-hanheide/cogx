//#include "stable_planes.hpp"
//#include <sampling.hpp>
//#include <Eigen/StdVector>
#include <alignment/stable_planes_alignment.h>
#include <omp.h>

int
main (int argc, char ** argv)
{
  //ALIGN TWO 3D MESHES
  vtkSmartPointer < vtkPLYReader > readerQuery = vtkSmartPointer<vtkPLYReader>::New ();
  readerQuery->SetFileName (argv[1]);
  vtkSmartPointer < vtkPolyData > polydata1 = readerQuery->GetOutput ();
  polydata1->Update ();

  vtkSmartPointer < vtkPLYReader > readerTarget = vtkSmartPointer<vtkPLYReader>::New ();
  readerTarget->SetFileName (argv[2]);
  vtkSmartPointer < vtkPolyData > polydata2 = readerTarget->GetOutput ();
  polydata2->Update ();

  bool parallel=false;
  if(argc > 3) {
    if (atoi(argv[3]) == 1) {
      parallel = true;
    }
  }

  bool gpu=false;
    if(argc > 5) {
      if (atoi(argv[5]) == 1) {
        gpu = true;
      }
    }

  std::string name_1 = std::string(argv[1]);
  std::string name_2 = std::string(argv[2]);

  StablePlanesAlignment spa(15000,atoi(argv[4]), parallel, gpu);
  Eigen::Matrix4f transform;
  spa.setName1(name_1);
  spa.setName2(name_2);
  spa.alignMeshes(polydata1, polydata2,transform);

  return 0;
}
