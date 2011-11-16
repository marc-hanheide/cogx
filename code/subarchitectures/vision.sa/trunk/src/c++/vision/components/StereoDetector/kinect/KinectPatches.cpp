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
KinectPatches::KinectPatches(KinectCore *kc, VisionCore *vc) : KinectBase(kc, vc)
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
printf("KinectPatches::Process: Antiquated: Use KinectPclModels instead.\n");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
  pclA::ConvertCvMat2PCLCloud(kcore->GetPointCloud(), pcl_cloud);

  // preprocess point cloud
  bool useVoxelGrid = false;                // still possible to use it???
  double vg_size = 0.005;                  // 0.005 - 0.01
  std::vector<int> pcl_model_cloud_indexes; // TODO be careful => not initialized!
  bool useZFilter = false;                // still possible to use it???
  double minZ = 0.3;
  double maxZ = 1.3;
  pclA::PreProcessPointCloud(pcl_cloud, pcl_model_cloud_indexes, 
                             useVoxelGrid, vg_size,
                             useZFilter, minZ, maxZ);

//   bool sac_optimal_distance = true;
//   double sac_optimal_weight_factor = 1.5; // with/out voxelgrid: 1.5 / 3
//   double sac_distance = 0.005;            // with/out voxelgrid: 0.005 / 0.01
//   int sac_max_iterations = 250;           // 250
//   int sac_min_inliers = 15;
//   double ec_cluster_tolerance = 0.025;    // 15mm
//   int ec_min_cluster_size = 15;
//   int ec_max_cluster_size = 1000000;

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_plane_clouds;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;

// printf("KinectPatches::Process: Fit Planes Recursive: start: Nr of points: %u\n", pcl_cloud.points.size());

struct timespec start, current;
clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);

  pclA::FitModelRecursive(pcl_cloud, pcl_plane_clouds, model_coefficients, pcl::SACMODEL_PLANE/*, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
                          sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size*/);

clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
printf("Runtime for processing kinect PATCHES: FitModelRecursive %4.3f\n", timespec_diff(&current, &start));
start=current;

//   pclA::FitModelRecursiveWithNormals(pcl_cloud.makeShared(), pcl_plane_clouds, model_coefficients, pcl::SACMODEL_PLANE/*, sac_optimal_distance, sac_optimal_weight_factor, sac_distance, sac_max_iterations, 
//                           sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size*/);
// 
// clock_gettime(CLOCK_THREAD_CPUTIME_ID, &current);
// printf("Runtime for processing kinect PATCHES: FitModelRecursiveWithNormals %4.3f\n", timespec_diff(&current, &start));
// start=current;
                          

  // Calculate convex hulls
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_hulls;
  pclA::GetConvexHulls(pcl_plane_clouds, model_coefficients, pcl_hulls);

  // convert pcl point clouds to vectors of Vec4f
  std::vector< std::vector<cv::Vec4f> > cv_vec_planes;
  std::vector< std::vector<cv::Vec4f> > cv_vec_hulls;
  pclA::ConvertPCLClouds2CvVecs(pcl_plane_clouds, cv_vec_planes);
  pclA::ConvertPCLClouds2CvVecs(pcl_hulls, cv_vec_hulls, true);
  
  // create Patch3D´s
  for(unsigned i=0; i<cv_vec_planes.size(); i++)
  {
    std::vector<int> mask_hull_idxs;          // TODO TODO TODO We do not have mask_hull_idxs here!!! => see KinectPclModels
    std::vector<cv::Vec4f> mask_hull_points;  // TODO TODO TODO We do not have mask_hull_points here
    std::vector<int> indices;                 // TODO TODO TODO We do not have point-indices => see KinectPclModels

//     cv::Mat_<cv::Vec3b> patch_mask = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
//     cv::Mat_<cv::Vec3b> patch_edges = cv::Mat_<cv::Vec3b>::zeros(kcore->GetPointCloudHeight(), kcore->GetPointCloudWidth());
//     pclA::ConvertIndexes2Mask(pcl_model_cloud_indexes[i], patch_mask);
//     pclA::ConvertMask2Edges(patch_mask, patch_edges);
//     pclA::ConvertEdges2Indexes(patch_edges, indexes);

    cv::Vec3f plane_normal;
    if(model_coefficients[i]->values[3] < 0.)
    {
      printf("KinectPatches::Process: Warning: model_coefficients of plane with negative distance. Inverted.\n");
      plane_normal[0] = -model_coefficients[i]->values[0];
      plane_normal[1] = -model_coefficients[i]->values[1];
      plane_normal[2] = -model_coefficients[i]->values[2];
    }
    else
    {
      plane_normal[0] = model_coefficients[i]->values[0];
      plane_normal[1] = model_coefficients[i]->values[1];
      plane_normal[2] = model_coefficients[i]->values[2];
    }

    Z::Patch3D *p3d = new Z::Patch3D(cv_vec_planes[i], indices, cv_vec_hulls[i], mask_hull_points, mask_hull_idxs, plane_normal);
    kcore->NewGestalt3D(p3d);
    numPatches++;
  }
// printf("KinectPatches::Process: end!\n");
}


}








