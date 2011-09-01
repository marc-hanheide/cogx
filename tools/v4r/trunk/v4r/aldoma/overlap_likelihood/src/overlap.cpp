/*
 * overlap.cpp
 *
 *  Created on: Jun 6, 2011
 *      Author: aitor
 */

#include "overlap.h"
#include "pcl/octree/octree.h"

#include "vtkPLYReader.h"
#include "vtkTransform.h"
#include "vtkTransformFilter.h"
#include "vtkPolyDataMapper.h"
#include "vtkDoubleArray.h"
#include "vtkPointData.h"
#include "vtkIVWriter.h"
#include "vtkPolyDataWriter.h"
#include "vtkPolyDataNormals.h"
#include "pcl/visualization/pcl_visualizer.h"

OverlapLikelihood::OverlapLikelihood (std::string path_for_vtk_files, float MAX_DIST)
{
  MAX_DIST_ = MAX_DIST;
  path_for_vtk_files_ = path_for_vtk_files;
}

OverlapLikelihood::~OverlapLikelihood ()
{

}

void
OverlapLikelihood::getVerticesAsPointCloud (vtkSmartPointer<vtkPolyData> & poly,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr & vertices)
{
  vtkPoints *points = poly->GetPoints ();
  vertices->points.resize (points->GetNumberOfPoints ());
  vertices->width = vertices->points.size ();
  vertices->height = 1;
  vertices->is_dense = false;

  for (int i = 0; i < points->GetNumberOfPoints (); i++)
  {
    double p[3];
    points->GetPoint (i, p);
    vertices->points[i].x = p[0];
    vertices->points[i].y = p[1];
    vertices->points[i].z = p[2];
  }
}

//Input (aligned mesh or path to ply + pose

void
OverlapLikelihood::setModel (vtkSmartPointer<vtkPolyData> & poly)
{
  poly_input_ = poly;
}

void
OverlapLikelihood::setModel (std::string path_to_ply, Eigen::Matrix4f & transform, float scale)
{
  vtkSmartPointer < vtkPLYReader > reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName (path_to_ply.c_str ());

  vtkSmartPointer < vtkTransform > trans = vtkSmartPointer<vtkTransform>::New ();
  trans->PostMultiply ();
  trans->Scale (scale, scale, scale);

  vtkSmartPointer < vtkTransform > poseTransform = vtkSmartPointer<vtkTransform>::New ();

  vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
  for (size_t kk = 0; kk < 4; kk++)
  {
    for (size_t k = 0; k < 4; k++)
    {
      mat->SetElement (kk, k, transform (kk, k));
    }
  }

  poseTransform->SetMatrix(mat);
  trans->Concatenate (poseTransform);
  trans->Modified ();

  //create transformation filter
  vtkSmartPointer < vtkTransformFilter > trans_filter = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter->SetTransform (trans);
  trans_filter->SetInputConnection (reader->GetOutputPort ());

  vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputConnection (trans_filter->GetOutputPort ());
  mapper->Update ();

  //generate polydata and save it to class
  poly_input_ = mapper->GetInput ();
}

void
OverlapLikelihood::setPartialPointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & input)
{
  input_ = input;
}

//Computation
void
OverlapLikelihood::compute ()
{

  poly_output_ = vtkSmartPointer<vtkPolyData>::New();
  poly_output_->DeepCopy (poly_input_);

  vtkSmartPointer < vtkDoubleArray > weights = vtkSmartPointer<vtkDoubleArray>::New ();

  // 0) Build octtree with input_ points
  float resolution = 128.0f;

  pcl::octree::OctreePointCloud < pcl::PointXYZ > octree (resolution);

  octree.setInputCloud (input_);
  octree.addPointsFromInputCloud ();

  /*pcl::visualization::PCLVisualizer vis ("likelihoodData");
  vis.addModelFromPolyData(poly_output_);
  vis.addPointCloud(input_);
  vis.spin ();*/

  // 1) Get points/vertices of the mesh
  vtkPoints *points = poly_output_->GetPoints ();
  weights->SetNumberOfValues(points->GetNumberOfPoints ());

  for (int i = 0; i < points->GetNumberOfPoints (); i++)
  {
    double p[3];
    points->GetPoint (i, p);

    // 2) Search for each vertex the closest neighbor in the octree
    std::vector<int> pointIdxNNSearch;
    std::vector<float> pointNNSquaredDistance;

    pcl::PointXYZ searchPoint;

    searchPoint.x = p[0];
    searchPoint.y = p[1];
    searchPoint.z = p[2];

    if (octree.nearestKSearch (searchPoint, 1, pointIdxNNSearch, pointNNSquaredDistance) > 0)
    {
      //compute weight
      double w;
      //w = 1.0 / (pointNNSquaredDistance[0] + 1.0);
      w = 1.0 / ( (sqrt(pointNNSquaredDistance[0]) / MAX_DIST_) + 1.0);
      weights->SetValue(i,w);
    }
  }

  poly_output_->GetPointData()->SetScalars(weights);
}

//Output
void
OverlapLikelihood::getOverlapLikelihoodPolydata (vtkSmartPointer<vtkPolyData> & poly_likelihood)
{
  poly_likelihood = poly_output_;
}

void
OverlapLikelihood::generateIVModel (std::string & path_to_iv)
{
  if(poly_output_ != NULL) {

    vtkSmartPointer<vtkPolyDataNormals> normal_computation = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_computation->ConsistencyOn();
    normal_computation->ComputeCellNormalsOn();
    normal_computation->ComputePointNormalsOn();
    normal_computation->AutoOrientNormalsOn();
    normal_computation->FlipNormalsOff();
    normal_computation->NonManifoldTraversalOn();
    normal_computation->SetInput(poly_output_);
    normal_computation->Update();

    vtkSmartPointer<vtkIVWriter> writer = vtkSmartPointer<vtkIVWriter>::New();
    writer->SetFileName("/tmp/tmp_iv.iv");
    writer->SetInput(normal_computation->GetOutput());
    writer->Update();
    writer->Write();

    std::ifstream in;    // Create an input file stream.
    in.open("/tmp/tmp_iv.iv");
    if ( ! in ) {
       cout << "Error: Can't open the file named data.txt.\n";
       exit(1);
    }

    std::ofstream out;
    out.open(path_to_iv.c_str());

    std::string str;
    getline(in,str);  // Get the frist line from the file, if any.
    int i=0;
    while ( in ) {  // Continue if the line was sucessfully read.

        out << str << endl;

        if (i == 3) {
          out << "ShapeHints {" << endl;
          out << "vertexOrdering CLOCKWISE" << endl;
          out << "}" << endl;
        }

        getline(in,str);   // Try to get another line.
        i++;
    }

    //save also VTK file
    vtkSmartPointer<vtkPolyDataWriter> vtk_writer = vtkSmartPointer<vtkPolyDataWriter>::New();

    std::stringstream path_stream;
    path_stream << path_for_vtk_files_ << "/test.vtk";

    std::string path = path_stream.str();

    vtk_writer->SetFileName(path.c_str());
    vtk_writer->SetInput(normal_computation->GetOutput());
    vtk_writer->Update();
    vtk_writer->Write();

  }
}
