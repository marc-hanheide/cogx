/**
 * @file PCLFunctions.h
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#ifndef PCLF_PCLFUNCTIONS_H
#define PCLF_PCLFUNCTIONS_H

#include "PCLCommonHeaders.h"

namespace pclF
{
  
/**
 * @brief Remove zero points from point cloud and use a VoxelGrid to reduce
 * point cloud complexity.
 * @param pcl_cloud Point cloud in PCL format
 * @param useVoxelGrid True, if voxel grid should be used.
 * @param vc_size Voxel grid size.
 * @return Returns true for success.
 */ 
bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
                          bool useVoxelGrid = false,
                          double vg_size = 0.01);

/**
 * @brief Fit planes into a point cloud, using SAC-segmentation and euclidean clustering.
 * First SAC removes the most supported plane, then the euclidean cluster algorithm
 * splits the rest of the cloud into clusters and calls for each cluster again this
 * function (recursive). Segmented planes are stored in clustered clouds in openCV 
 * matrix style.
 * @param pcl_cloud Point cloud in PCL format.
 * @param pcl_plane_clouds Vector of point clouds with resulting planes.
 * @param sac_optimal_distance Calculate optimal distance threshold for plane fitting.
 * @param sac_optimal_weight_factor Weight factor for optimal threshold processing.
 * @param sac_distance Inlier distance for SAC segmentation.
 * @param sac_max_iterations Maximum iterations for SAC.
 * @param sac_min_inliers Minimum inliers for a plane.
 * @param ec_cluster_tolerance Minimum distance for euclidean clustering.
 * @param ec_min_cluster_size Minimum cluster size.
 * @param ec_max_cluster_size Maximum cluster size.
 * @return Returns true for success.
 */
bool FitPlanesRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                        std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                        bool sac_optimal_distance = false,
                        double sac_optimal_weight_factor = 1.5,
                        double sac_distance = 0.008,
                        int sac_max_iterations = 100,
                        int sac_min_inliers = 25,
                        double ec_cluster_tolerance = 0.015, 
                        int ec_min_cluster_size = 25,
                        int ec_max_cluster_size = 1000000);
                        
/**
 * @brief See FitPlanesRecursive().
 * This function uses SAC-segmentation with normals.
 */
bool FitPlanesRecursiveWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                        std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                        bool sac_optimal_distance = false,
                        double sac_optimal_weight_factor = 1.5,
                        double sac_distance = 0.008,
                        int sac_max_iterations = 100,
                        int sac_min_inliers = 25,
                        double ec_cluster_tolerance = 0.015, 
                        int ec_min_cluster_size = 25,
                        int ec_max_cluster_size = 1000000);
         

/**
 * @brief Process a single SAC segmentation: Fit a plane into the cloud, using SAC_RANSAC.
 * Returns the pcl_cloud without the segmented plane points. The resulting plane will be
 * stored in clustered_clouds in openCV matrix style.
 * @param pcl_cloud Point cloud in pcl format to segment.
 * @param pcl_plane_clouds Vector of point clouds with resulting planes.
 * @param sac_optimal_distance Calculate optimal SAC distance threshold
 * @param sac_optimal_weight_factor Weight factor for optimal SAC distance trheshold calculation.
 * @param sac_distance Distance threshold for the plane
 * @param maxIterations Maximum iterations for the RANSAC.
 * @param minInliers Minimum inliers for a successful plane.
 * @return Returns true for success and if enough points for another segmentation are left.
 */  
bool SingleSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                           bool sac_optimal_distance = true,
                           double sac_optimal_weight_factor = 1.5,
                           double sac_distance = 0.008, 
                           int maxIterations = 100,
                           int minInliers = 25);
                           
/**
 * @brief See SingleSACSegmentation().
 * This function uses point normals to segment the point cloud.
 */
bool SingleSACSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                      std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                      std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                      bool calc_optimal_sac_distance,
                                      double sac_optimal_weight_factor,
                                      double sac_distance, 
                                      int maxIterations,
                                      int minInliers);
                           

/**
 * @brief Cluster a point cloud with a euclidean threshold.
 * @param pcl_cloud Point cloud in pcl format to cluster.
 * @param cluster_tolerance Euclidean threshold for clustering points.
 * @param min_cluster_size Minimum cluster size for a object.
 * @return Returns true for success
 */  
bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         double cluster_tolerance, 
                         double min_cluster_size,
                         double max_cluster_size);                           

/**
 * @brief Calculate the convex hull of a point cloud. First project pcl_clouds to the planes,
 * then calculate the convex hull.
 * @param pcl_clouds Planes as projected points.
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_convex_hulls Convex hulls
 */                        
bool GetConvexHulls(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                    std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_convex_hulls);
                           
                           
                         
                         
                         
                         
                         

               

// bool FitPlanesIncremental(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
//                           std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
//                           double cluster_tolerance, 
//                           int min_cluster_size,
//                           int max_cluster_size);
//   
// /**
//  * @brief TODO
//  */
// bool FitPlanesAfterDominantPlaneExtraction(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
//                                            std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
//                                            double cluster_tolerance = 0.01, 
//                                            int min_cluster_size = 50,
//                                            int max_cluster_size = 1000000);
// 
// /** TODO TODO TODO Write explanataion
//  * @brief FitPlanes into a point cloud. Use first euclidean clustering and second SACSegmentation. 
//  * @param cloud Point cloud in pcl format to cluster into planes.
//  * @param pcl_clustered_clouds Resulting plane clouds in pcl style.
//  * @param clustered_clouds Resulting plane clusters in openCv style.
//  * @param cluster_tolerance Euclidean threshold for clustering points and SACSegmentation. (default: 2cm)
//  * @param min_cluster_size Minimum cluster size for a plane. (default: 100 points)
//  * @return Returns true for success
//  */ 
// bool FitPlanesWithNormals(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
//                           std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
//                           double cluster_tolerance, 
//                           int min_cluster_size,
//                           int max_cluster_size);
//                                            
// /**
//  * @brief FitPlanes into a point cloud. Use first euclidean clustering and second SACSegmentation. 
//  * @param cloud Point cloud in pcl format to cluster into planes.
//  * @param pcl_clustered_clouds Resulting plane clouds in pcl style.
//  * @param clustered_clouds Resulting plane clusters in openCv style.
//  * @param cluster_tolerance Euclidean threshold for clustering points and SACSegmentation. (default: 2cm)
//  * @param min_cluster_size Minimum cluster size for a plane. (default: 100 points)
//  * @return Returns true for success
//  */  
// bool FitPlanes(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
//                std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
//                double cluster_tolerance = 0.01, 
//                int min_cluster_size = 50,
//                int max_cluster_size = 1000000);
// 
// 
// /** TODO TODO TODO Write explanataion
//  * @brief Cluster a point cloud with a euclidean threshold.
//  * @param cloud Point cloud in pcl format to cluster.
//  * @param cluster_tolerance Euclidean threshold for clustering points.
//  * @param min_cluster_size Minimum cluster size for a object.
//  * @return Returns true for success
//  */  
// bool SingleEuclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
//                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cluster_cloud,
//                                double cluster_tolerance = 0.015, // 15mm
//                                double min_cluster_size = 50,
//                                double max_cluster_size = 1000000);
// 
//                          
// 
// 
//   
// 
//                            
// /**
//  * @brief Calculate reverse SAC-Segmentation with pcl.
//  * @param cloud PCL point cloud
//  * @param distanceTH Distance threshold for the SACSegmentation of pcl.
//  * @param cvClouds Resulting point clouds of the segmentation.
//  * @return Returns true for success.
//  */
// bool ReverseSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
//                             std::vector< cv::Mat_<cv::Vec4f> > &cvClouds,
//                             double distanceTH);
// 
//  
// /**
//  * @brief Calculate reverse SAC-Segmentation with pcl.
//  * @param points Point cloud in cast-framework style
//  * @param distanceTH Distance threshold for the SACSegmentation of pcl.
//  * @param cvClouds Resulting point clouds of the segmentation.
//  * @return Returns true for success.
//  * TODO TODO Do not give surface points to this functions, give a cv::Mat, 
//  * so that we do not need cast things in here
//  */
// // bool ReverseSACSegmentationFromNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
// //                                        std::vector< cv::Mat_<cv::Vec4f> > &cvClouds,
// //                                        double distanceTH);
// //   
  
}

#endif