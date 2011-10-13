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
// static struct timespec start, last, current;
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// last = current;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pclA::CopyPointCloud(kcore->GetPclCloud(), pcl_cloud);

  model_fitter = new pclA::ModelFitter();
  model_fitter->AddModelType(pcl::SACMODEL_PLANE);
//   model_fitter->AddModelType(pcl::SACMODEL_CYLINDER);
//   model_fitter->AddModelType(pcl::SACMODEL_SPHERE);
  pcl::PointCloud<pcl::Normal>::Ptr no = kcore->GetPclNormals();
  model_fitter->SetNormals(no);
    
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  ### KinectPclModels: Runtime for pre-processing: %4.3f\n", timespec_diff(&current, &last));
// last = current;

//   model_fitter->Process(pcl_cloud);
  model_fitter->ProcessWithoutDP(pcl_cloud);
  
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  ### KinectPclModels: Runtime for processing: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  std::vector<int> pcl_model_types;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< std::vector<int> > pcl_model_cloud_indexes;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  model_fitter->GetResults(pcl_model_types, pcl_model_clouds, pcl_model_cloud_indexes, model_coefficients);

// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  ### KinectPclModels: Runtime for post-processing: %4.3f\n", timespec_diff(&current, &last));
// last = current;

  /// ++++++++++++++++++++++++++++++++++++++++ postprocessing ++++++++++++++++++++++++++++++++++++++++ ///

  int pl = 0;  // remove later
  
  // Create now the different models
  std::vector< std::vector<cv::Vec4f> > cv_vec_models;
  pclA::ConvertPCLClouds2CvVecs(pcl_model_clouds, cv_vec_models);
  for(unsigned i=0; i<cv_vec_models.size(); i++)
  {
    /// convert 
    std::vector<int> indexes;
    cv::Mat_<cv::Vec3b> patch_mask = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
    cv::Mat_<cv::Vec3b> patch_edges = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
    pclA::ConvertIndexes2Mask(pcl_model_cloud_indexes[i], patch_mask);
    pclA::ConvertMask2Edges(patch_mask, patch_edges);
    pclA::ConvertEdges2Indexes(patch_edges, indexes);
//     cv::imshow("Patch-Mask", patch_edges);
    
    if(pcl_model_types[i] == pcl::SACMODEL_PLANE)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_hull;
      pclA::GetConvexHull(pcl_model_clouds[i], model_coefficients[i], pcl_hull);

      // convert pcl point clouds to vectors of Vec4f
      std::vector<cv::Vec4f> cv_vec_hull;
      pclA::ConvertPCLCloud2CvVec(pcl_hull, cv_vec_hull, true);
      
      /// TODO TODO TODO Convex hull area!!!
      std::vector<cv::Vec3f> pcl_convex_hull;
      pclA::ConvertPCLCloud2CvVec(pcl_hull, pcl_convex_hull);
      double area = pclA::GetConvexHullArea(pcl_convex_hull);
      
      /// TODO Calculate square error !!!
      double square_error = 0.;
      pclA::CalcSquareErrorToPlane(pcl_model_clouds[i], model_coefficients[i], square_error);

// printf("PclModels %u: pts: %u - Area: %4.3f => %4.3f && square_error: %4.3f\n", pl, cv_vec_models[i].size(), area*1000, cv_vec_models[i].size()/(area*1000), square_error);
// if(cv_vec_models[i].size()/(area*1000) < 20 && cv_vec_models[i].size() < 200)
// printf("                    => Maybe wrong?\n");

      
      Z::Patch3D *p3d = new Z::Patch3D(cv_vec_models[i], indexes, cv_vec_hull);
      kcore->NewGestalt3D(p3d);
      numPatches++;
      numModels++;
      pl++;
    }
    else if(pcl_model_types[i] == pcl::SACMODEL_SPHERE)
    {
      Z::PclSphere3D *pl3d = new Z::PclSphere3D(cv_vec_models[i], indexes);
      kcore->NewGestalt3D(pl3d);
      numSpheres++;
      numModels++;
    }
    else if(pcl_model_types[i] == pcl::SACMODEL_CYLINDER)
    {
      Z::PclCylinder3D *pl3d = new Z::PclCylinder3D(cv_vec_models[i], indexes);
      kcore->NewGestalt3D(pl3d);
      numCylinders++;
      numModels++;
    } 
  }
  
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("  Runtime for KinectPclModels: %4.3f\n", timespec_diff(&current, &last));
// last = current;

}


}








