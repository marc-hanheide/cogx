/**
 * @file KinectPclCylinders.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D cylinders from Kinect data with pcl-library.
 */

#include <vector>
#include "KinectCore.h"
#include "KinectPclCylinders.h"
#include "PclCylinder3D.h"

namespace Z
{

/**
 * @brief Constructor of KinectPclCylinders
 * @param vc Vision core of calculated LEFT and RIGHT stereo image
 */
KinectPclCylinders::KinectPclCylinders(KinectCore *kc, VisionCore *vc) 
                  : KinectBase(kc, vc)
{
  numCylinders = 0;
}


/**
 * @brief Draw matched Lines.
 * @param side Left or right image from stereo rig.
 * @param single Draw single feature
 * @param id ID of single feature
 * @param detail Degree of detail
 */
// void KinectPclLines::DrawMatched(int side, bool single, int id, int detail)
// {
//   if(single)
//   {
//     if(id < 0 || id >= lineMatches)
//     {
//       printf("StereoLines::DrawMatched: warning: id out of range!\n");
//       return;
//     }
//     DrawSingleMatched(side, id, detail);
//   }
//   else
//     for(int i=0; i< lineMatches; i++)
//       DrawSingleMatched(side, i, detail);
// }

/**
 * @brief Draw single matched line.
 * @param side Left or right image from stereo rig.
 * @param id ID of single feature
 * @param detail Degree of detail
 */
// void KinectPclLines::DrawSingleMatched(int side, int id, int detail)
// {
//   lines[side][id].Draw(detail);
// }


/**
 * @brief Clear all arrays.
 */
void KinectPclCylinders::ClearResults()
{
  numCylinders = 0;
}


/**
 * @brief Calculate 3D lines from kinect data.
 */
void KinectPclCylinders::Process()
{
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pclA::ConvertCvMat2PCLCloud(kcore->points, pcl_cloud);

    // preprocess point cloud
  bool useVoxelGrid = false;
  double vg_size = 0.005;                  // 0.005 - 0.01
  pclA::PreProcessPointCloud(pcl_cloud, useVoxelGrid, vg_size);

  bool sac_optimal_distance = false;
  double sac_optimal_weight_factor = 1.5; // with/out voxelgrid: 1.5 / 3
  double sac_distance = 0.005;            // with/out voxelgrid: 0.005 / 0.01
  int sac_max_iterations = 250;
  int sac_min_inliers = 25;
  double ec_cluster_tolerance = 0.015;    // 15mm
  int ec_min_cluster_size = 25;
  int ec_max_cluster_size = 1000000;

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  pcl::PointCloud<pcl::Normal>::Ptr normals;// (new pcl::PointCloud<pcl::Normal>);

// printf("KinectPclLines::FitLine start\n");
//   pclA::FitLine(pcl_cloud.makeShared(), pcl_line_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, sac_min_inliers);
// printf("KinectPclLines::FitLine ended\n");
  
printf("KinectPclCylinders::FitMultipleModelRecursive start\n");
//   pclA::FitCylindersRecursive(pcl_cloud.makeShared(), pcl_cylinder_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
//                               sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
  
  std::vector<int> sac_models, resulting_models;
  sac_models.push_back(pcl::SACMODEL_PLANE);
  sac_models.push_back(pcl::SACMODEL_SPHERE);
  
  pclA::FitMultipleModelRecursive(pcl_cloud.makeShared(), 
                                  normals,
                                  pcl_model_clouds, 
                                  model_coefficients, 
                                  sac_models, 
                                  resulting_models, 
                                  sac_optimal_distance, 
                                  sac_optimal_weight_factor, 
                                  sac_distance, 
                                  sac_max_iterations, 
                                  sac_min_inliers, 
                                  ec_cluster_tolerance, 
                                  ec_min_cluster_size, 
                                  ec_max_cluster_size);
printf("KinectPclCylinders::FitMultipleModelRecursive end\n");


  std::vector< std::vector<cv::Vec4f> > cv_vec_cylinders;
  pclA::ConvertPCLClouds2CvVecs(pcl_model_clouds, cv_vec_cylinders);

  for(unsigned i=0; i<cv_vec_cylinders.size(); i++)
  {
    Z::PclCylinder3D *pl3d = new Z::PclCylinder3D(cv_vec_cylinders[i]);
    kcore->NewGestalt3D(pl3d);
    numCylinders++;
  }
}


}








