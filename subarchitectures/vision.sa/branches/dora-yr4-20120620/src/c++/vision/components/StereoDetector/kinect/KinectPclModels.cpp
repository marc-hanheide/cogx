/**
 * @file KinectPclModels.cpp
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D models from Kinect data with pcl-library.
 */

#include <vector>
#include "KinectCore.h"
#include "KinectPclModels.h"
#include "Patch3D.h"
#include "PclSphere3D.h"
#include "PclCylinder3D.h"

namespace Z
{

/**
 * @brief Constructor of KinectPclCylinders
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
KinectPclModels::KinectPclModels(KinectCore *kc, VisionCore *vc) 
                : KinectBase(kc, vc)
{
  numModels = 0;
  numPatches = 0;
  numSpheres = 0;
  numCylinders = 0;
}


/**
 * @brief Clear all arrays.
 */
void KinectPclModels::ClearResults()
{
  numModels = 0;
  numPatches = 0;
  numSpheres = 0;
  numCylinders = 0;
}


/**
 * @brief Calculate 3D lines from kinect data.
 */
void KinectPclModels::Process()
{
printf("KinectPclModels::Process: start!\n");
static struct timespec start, last, current;
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// last = current;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pclA::CopyPointCloud(kcore->GetPclCloud(), pcl_cloud);

  double minZ = 0.3;
  double maxZ = 1.5;
  pclA::ModelFitter::Parameter param(false, 0.005, true, minZ, maxZ);
  model_fitter = new pclA::ModelFitter(param);
  model_fitter->addModelType(pcl::SACMODEL_PLANE);
//   model_fitter->addModelType(pcl::SACMODEL_NORMAL_PLANE);
//  model_fitter->addModelType(pcl::SACMODEL_CYLINDER);
//  model_fitter->addModelType(pcl::SACMODEL_SPHERE);
  pcl::PointCloud<pcl::Normal>::Ptr no = kcore->GetPclNormals();
  model_fitter->setNormals(no);
    
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  KinectPclModels: Runtime for pre-processing: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  model_fitter->useDominantPlane(true);
  model_fitter->setInputCloud(pcl_cloud);
  model_fitter->compute();
  
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  KinectPclModels: Runtime for processing: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  std::vector<int> pcl_model_types;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< std::vector<int> > pcl_model_cloud_indexes;        /// TODO Old version => delete
  std::vector<pcl::PointIndices::Ptr> pcl_model_cloud_indices;  /// TODO New version!!!
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  model_fitter->GetResults(pcl_model_types, pcl_model_clouds, pcl_model_cloud_indexes, model_coefficients); /// TODO Old version => delete
  model_fitter->getResults(pcl_model_types, model_coefficients, pcl_model_cloud_indices);                   /// TODO New version!!!
            
//   printf("############## Planes.size: %u\n", pcl_model_cloud_indices.size());
//   for(unsigned i=0; i<pcl_model_cloud_indices.size(); i++)
//     for(unsigned j=0; j<pcl_model_cloud_indices[i]->indices.size(); j++)
// printf("KPM: %u-%u => idx: %u\n", i, j, pcl_model_cloud_indices[i]->indices[j]);
  
  std::vector< std::vector<double> > distances;
  std::vector<double> square_errors;
  model_fitter->getError(distances, square_errors);

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  KinectPclModels: Runtime for processing: %4.3f\n", timespec_diff(&current, &last));
last = current;  
  
//   std::vector< std::vector<unsigned> > neighbors;
//   planes = new pclA::Planes();
//   planes->setZLimit(0.005);
//   planes->setInputCloud(pcl_cloud);
//   planes->setPlanes(pcl_model_types, model_coefficients, pcl_model_cloud_indices);
//   planes->computeNeighbors();
//   planes->getNeighbors(neighbors);
 
//   printf("Results of neighbors: \n");
//   for(unsigned i=0; i<neighbors.size(); i++)
//     for(unsigned j=0; j<neighbors[i].size(); j++)
//       printf("#%u neighbors: %u-%u\n", i, i, neighbors[i][j]);
  
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("  KinectPclModels: Runtime for planes neighbor-processing: %4.3f\n", timespec_diff(&current, &last));
last = current;

  /// ++++++++++++++++++++++++++++++++++++++++ postprocessing ++++++++++++++++++++++++++++++++++++++++ ///

  // Create now the different models
  std::vector< std::vector<cv::Vec4f> > cv_vec_models;
  pclA::ConvertPCLClouds2CvVecs(pcl_model_clouds, cv_vec_models);
  for(unsigned i=0; i<cv_vec_models.size(); i++)
  {
// printf("KinectPclModels::Process: Size of pcl_model_cloud: %u\n", pcl_model_clouds[i]->points.size());
    /// get 2d mask hull indexes and 3d mask hull points 
    std::vector<int> mask_hull_idxs;
    std::vector<cv::Vec4f> mask_hull_points;
    cv::Mat_<cv::Vec3b> patch_mask = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
    cv::Mat_<cv::Vec3b> patch_edges = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
    pclA::ConvertIndexes2Mask(pcl_model_cloud_indexes[i], patch_mask);    /// TODO Write new function for pcl_model_cloud_indices (oder brauch ich das wegen planes eh nicht mehr?)
    pclA::ConvertMask2Edges(patch_mask, patch_edges);
    pclA::ConvertEdges2Indexes(patch_edges, mask_hull_idxs);
    for(unsigned j=0; j<mask_hull_idxs.size(); j++)
    {
      pcl::PointXYZRGB pt = kcore->GetPclCloud()->points[mask_hull_idxs[j]];
      cv::Vec4f p;
      p[0] = pt.x;
      p[1] = pt.y;
      p[2] = pt.z;
      p[3] = pt.rgb;
      mask_hull_points.push_back(p);
    }
// cv::imshow("Patch-Mask", patch_mask);
    
    if(pcl_model_types[i] == pcl::SACMODEL_PLANE || pcl_model_types[i] == pcl::SACMODEL_NORMAL_PLANE)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_hull;
      pclA::GetConvexHull(pcl_model_clouds[i], model_coefficients[i], pcl_hull);

      // convert pcl point clouds to vectors of Vec4f
      std::vector<cv::Vec4f> cv_vec_hull;
      pclA::ConvertPCLCloud2CvVec(pcl_hull, cv_vec_hull, true);
      
      /// TODO TODO TODO Convex hull area!!!
//       std::vector<cv::Vec3f> pcl_convex_hull;
//       pclA::ConvertPCLCloud2CvVec(pcl_hull, pcl_convex_hull);
//       double area = pclA::GetConvexHullArea(pcl_convex_hull);
      
      /// TODO Calculate square error !!!
//       double square_error = 0.;
//       pclA::CalcSquareErrorToPlane(pcl_model_clouds[i], model_coefficients[i], square_error);

// printf("PclModels %u: pts: %u - Area: %4.3f => %4.3f && square_error: %4.3f\n", pl, cv_vec_models[i].size(), area*1000, cv_vec_models[i].size()/(area*1000), square_error);
// if(cv_vec_models[i].size()/(area*1000) < 20 && cv_vec_models[i].size() < 200)
// printf("                    => Maybe wrong?\n");

      cv::Vec3f plane_normal;
      if(model_coefficients[i]->values[3] < 0.) {
        plane_normal[0] = -model_coefficients[i]->values[0];
        plane_normal[1] = -model_coefficients[i]->values[1];
        plane_normal[2] = -model_coefficients[i]->values[2];
      }
      else {
        plane_normal[0] = model_coefficients[i]->values[0];
        plane_normal[1] = model_coefficients[i]->values[1];
        plane_normal[2] = model_coefficients[i]->values[2];
      }
      
      Z::Patch3D *p3d = new Z::Patch3D(cv_vec_models[i], pcl_model_cloud_indexes[i], cv_vec_hull, mask_hull_points, mask_hull_idxs, plane_normal);    /// TODO Übergabe von pcl_model_cloud_indices statt indexes
      kcore->NewGestalt3D(p3d);
      numPatches++;
      numModels++;
    }
    else if(pcl_model_types[i] == pcl::SACMODEL_SPHERE)
    {
      Z::PclSphere3D *pl3d = new Z::PclSphere3D(cv_vec_models[i], pcl_model_cloud_indexes[i], mask_hull_idxs);                                        /// TODO Übergabe von pcl_model_cloud_indices statt indexes
      kcore->NewGestalt3D(pl3d);
      numSpheres++;
      numModels++;
    }
    else if(pcl_model_types[i] == pcl::SACMODEL_CYLINDER)
    {
      Z::PclCylinder3D *pl3d = new Z::PclCylinder3D(cv_vec_models[i], pcl_model_cloud_indexes[i], mask_hull_idxs);                                    /// TODO Übergabe von pcl_model_cloud_indices statt indexes
      kcore->NewGestalt3D(pl3d);
      numCylinders++;
      numModels++;
    } 
  }
  
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  Runtime for KinectPclModels: %4.3f\n", timespec_diff(&current, &last));
// last = current;
printf("KinectPclModels::Process: end!\n");

}


}








