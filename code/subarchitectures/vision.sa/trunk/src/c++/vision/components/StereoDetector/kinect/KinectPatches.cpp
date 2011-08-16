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
// printf("KinectPatches::Process: start!\n");
//   struct timespec start, current;
//   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  pclA::ConvertCvMat2PCLCloud(points, pcl_cloud);

// printf("KinectPatches::Process: pcl_cloud.points: %u\n", pcl_cloud.points.size());
  
  // preprocess point cloud
  bool useVoxelGrid = false;
  double vg_size = 0.005;                  // 0.005 - 0.01
  pclA::PreProcessPointCloud(pcl_cloud, useVoxelGrid, vg_size);

  bool sac_optimal_distance = true;
  double sac_optimal_weight_factor = 1.5; // with/out voxelgrid: 1.5 / 3
  double sac_distance = 0.005;            // with/out voxelgrid: 0.005 / 0.01
  int sac_max_iterations = 250;
  int sac_min_inliers = 25;
  double ec_cluster_tolerance = 0.015;    // 15mm
  int ec_min_cluster_size = 25;
  int ec_max_cluster_size = 1000000;

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;

// printf("KinectPatches::Process: Fit Planes Recursive: start: Nr of points: %u\n", pcl_cloud.points.size());
  pclA::FitPlanesRecursive(pcl_cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
                           sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);

// printf("KinectPatches::ProcessFit Planes Recursive: end: nr_patches: %u!\n", pcl_plane_clouds.size());

//   pclF::FitPlanesRecursiveWithNormals(cloud.makeShared(), pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
//                            sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);

  // calculate convex hulls
// printf("KinectPatches: Calcualte convex hulls: start\n");
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_hulls;
  pclA::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_hulls);
// printf("KinectPatches: Calcualte convex hulls: end\n");

  // convert pcl point clouds to cv matrices
  std::vector< std::vector<cv::Vec4f> > cv_vec_planes;
  std::vector< std::vector<cv::Vec4f> > cv_vec_hulls;
  pclA::ConvertPCLClouds2CvVecs(pcl_plane_clouds, cv_vec_planes);
  pclA::ConvertPCLClouds2CvVecs(pcl_hulls, cv_vec_hulls, true);
  
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
// printf("KinectPatches::Process: end!\n");
}


}








