/*
 * overlap.h
 *
 *  Created on: Jun 6, 2011
 *      Author: aitor
 */

#ifndef OVERLAP_H_
#define OVERLAP_H_

#include "pcl/common/common.h"
#include "vtkSmartPointer.h"
#include "vtkPolyData.h"
//#include "vtkSmartPointer.h"

class OverlapLikelihood {
  private:
    vtkSmartPointer<vtkPolyData> poly_input_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    vtkSmartPointer<vtkPolyData> poly_output_;
    float MAX_DIST_;
    std::string path_for_vtk_files_;
    void getVerticesAsPointCloud(vtkSmartPointer<vtkPolyData> & poly, pcl::PointCloud<pcl::PointXYZ>::Ptr & vertices);

  public:
    OverlapLikelihood(std::string path_for_vtk_files, float MAX_DIST=0.01);
    ~OverlapLikelihood();

    //Input
    void setModel(vtkSmartPointer<vtkPolyData> & poly);
    void setModel(std::string path_to_ply, Eigen::Matrix4f & transform, float scale = 1.0);
    void setPartialPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & input);

    //Computation
    void compute();

    //Output
    void getOverlapLikelihoodPolydata(vtkSmartPointer<vtkPolyData> & poly_likelihood);
    void generateIVModel(std::string & path_to_iv);
};

#endif /* OVERLAP_H_ */
