/**
 * @file KinectPatches.cpp
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Kalculate 3D segments from Kinect data.
 */

#include "KinectCore.h"
#include "KinectPatches.h"
//#include "VisionUtils.h"

#include "PCLCommonHeaders.h"
#include "Patch3D.h"

namespace Z
{


/**
 * @brief Constructor of class KinectPatches
 * @param kc Kinect core
 * @param vc Vision core
 * @param iplI Ipl-Image
 * @param p Point matrix
 */
KinectPatches::KinectPatches(KinectCore *kc, VisionCore *vc, IplImage *iplI, cv::Mat_<cv::Vec4f> &p) 
                             : KinectBase(kc, vc, iplI, p)
{
  numPatches = 0;
}

/**
 * @brief Clear all arrays.
 */
void KinectPatches::ClearResults()
{
  numPatches = 0;
}


/**
 * @brief Calculate 3D patches from kinect data.
 */
void KinectPatches::Process()
{
//   struct timespec start, current;
//   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pclU::Cv2PCLCloud(points, cloud);

  // preprocess point cloud
  bool useVoxelGrid = true;
  double vg_size = 0.008;                 // 0.01 - 0.005
  pclF::PreProcessPointCloud(cloud, useVoxelGrid, vg_size);

  bool sac_optimal_distance = true;
  double sac_optimal_weight_factor = 1.5;
  double sac_distance = 0.005;            // 5mm
  int sac_max_iterations = 100;
  int sac_min_inliers = 25;
  double ec_cluster_tolerance = 0.015;    // 15mm
  int ec_min_cluster_size = 25;
  int ec_max_cluster_size = 1000000;

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  pclF::FitPlanesRecursive(cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
                           sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
                           
//   pclF::FitPlanesRecursiveWithNormals(cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
//                            sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);

  // calculate convex hulls
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_hulls;
  pclF::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_hulls);

  // convert pcl point clouds to cv matrices
  std::vector< std::vector<cv::Vec4f> > cv_vec_planes;
  std::vector< std::vector<cv::Vec4f> > cv_vec_hulls;
  pclU::PCLClouds2CvVecs(pcl_plane_clouds, cv_vec_planes);
  pclU::PCLClouds2CvVecs(pcl_hulls, cv_vec_hulls, true);
        
//   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
//   printf("Runtime for processing the point cloud: %4.3f\n", timespec_diff(&current, &start));
//   start=current;

  // create Patch3D´s
  for(unsigned i=0; i<cv_vec_planes.size(); i++)
  {
    Z::Patch3D *p3d = new Z::Patch3D(cv_vec_planes[i], cv_vec_hulls[i]);
    kcore->NewGestalt3D(p3d);
    numPatches++;
  }
}


}








