/**
 * @file PCLFunctions.h
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#ifndef PCLA_PCLFUNCTIONS_H
#define PCLA_PCLFUNCTIONS_H

#include <vector>
#include "v4r/PCLAddOns/utils/PCLCommonHeaders.h"

namespace pclA
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
 * stored in pcl_plane_clouds in openCV matrix style.
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
 * @param pcl_cluster_clouds The resulting clustered clouds in pcl-format.
 * @param cluster_tolerance Euclidean threshold for clustering points.
 * @param min_cluster_size Minimum cluster size for a object.
 * @param max_cluster_size Maximum cluster size for a object.
 * @return Returns true for success
 */  
bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         double cluster_tolerance, 
                         double min_cluster_size,
                         double max_cluster_size);                           


/**
 * @brief Cluster a point cloud with a euclidean threshold. Indices, marking the popout points
 * are given.
 * @param pcl_cloud Point cloud in pcl format to cluster.
 * @param popout Indices, marking the popout
 * @param pcl_cluster_clouds The resulting clustered clouds in pcl-format.
 * @param cluster_tolerance Euclidean threshold for clustering points.
 * @param min_cluster_size Minimum cluster size for a object.
 * @param max_cluster_size Maximum cluster size for a object.
 * @return Returns true for success
 */ 
void EuclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, 
                           const pcl::PointIndices popout,
                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                           double cluster_tolerance = 0.015,
                           double min_cluster_size = 10,
                           double max_cluster_size = 1000000);
                         
/**
 * @brief Calculate the convex hull of a point cloud. First project pcl_cloud to the plane,
 * then calculate the convex hull.
 * @param pcl_cloud The point cloud - Returned projected.
 * @param model_coefficient Model coefficients of the plane.
 * @param pcl_convex_hull Convex hull
 */                        
bool GetConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                   pcl::ModelCoefficients::Ptr model_coefficient,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_convex_hull);
 
/**
 * @brief Calculate the convex hull of a point cloud. First project pcl_clouds to the planes,
 * then calculate the convex hull.
 * @param pcl_clouds The point clouds - Returned projected!
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_convex_hulls Convex hulls
 */                        
bool GetConvexHulls(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                    std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_convex_hulls);
 
                    
/**
 * @brief Project a point cloud to the model (SAC plane).
 * @param pcl_clouds Point clouds in pcl format.
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_clouds_projected Projected point clouds
 */    
bool GetProjectedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                        pcl::ModelCoefficients::Ptr model_coefficients,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud_projected);
                           
/**
 * @brief Project point clouds to models (SAC plane).
 * @param pcl_clouds Point clouds in pcl format.
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_clouds_projected Projected point clouds
 */                            
bool GetProjectedPoints(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds_projected);                         
                         
                         
                         
/**
 * @brief Space Of Interest (SOI) Segmentation: We assume a dominant plane (table) on which we
 * can identify seperated objects. We prune segmented clouds outside of the dominant plane.
 * Returns the ...??? TODO
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
bool SOISegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                     std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                     std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                     bool sac_optimal_distance,
                     double sac_optimal_weight_threshold,
                     double sac_distance,
                     int sac_max_iterations,
                     int sac_min_inliers,
                     double ec_cluster_tolerance, 
                     int ec_min_cluster_size,
                     int ec_max_cluster_size);
               

/**
 * @brief Space Of Interest (SOI) Segmentation: We assume a dominant plane (table) on which we
 * @param pcl_cloud Point cloud in PCL format.
 * @param pcl_plane_clouds Vector of point clouds with resulting planes.
 * @param sac_optimal_distance Calculate optimal distance threshold for plane fitting.
 * @param                                                 
 */                       
void Mask2SOI(const cv::Mat_<ushort> &labels, 
              const std::vector<unsigned> &sizeClusters, 
              const cv::Mat_<uchar> &_mask,
              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &sois,
              std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &soiHulls);

  
              
/**
 * @brief Get the maximum distance from one point of a point cloud to a given plane.
 * @param pcl_cloud Point cloud
 * @param model_coefficients Model coefficients of the plane
 * @param max_distance Maximum distance of one of the points
 */
void MaxDistanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        const pcl::ModelCoefficients::Ptr &model_coefficients,
                        double &max_distance);
                        
                        
 /**
 * @brief Create for a given cloud with model_coefficients of a table and a reference
 * distance a SOI.
 * @param pcl_cloud Point cloud
 * @param model_coefficients Model coefficients of the plane
 * @param distance Distance between convex hull on the table and point on top of SOI.
 * @param soi Space of interest hull points.
 */                       
void CreateSOI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
               const pcl::ModelCoefficients::Ptr &model_coefficients,
               const double distance, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &soi);
                        
}

#endif
