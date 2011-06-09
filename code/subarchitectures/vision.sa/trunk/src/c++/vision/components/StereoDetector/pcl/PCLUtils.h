/**
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Utils for calculations with PCL, openCV and cast prototypes.
 */


#ifndef PCLU_PCLUTILS_H
#define PCLU_PCLUTILS_H

#include "PCLCommonHeaders.h"

namespace pclU
{
  
/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char Blue;   // Blue channel
    unsigned char Green;  // Green channel
    unsigned char Red;    // Red channel
    unsigned char Alpha;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


/**
 * @brief Convert openCV vector points to pcl point cloud.
 * @param cv_cloud Point cloud in openCV vector format
 * @param pcl_cloud PCL style point cloud
 */
void Cv2PCLCloud(std::vector<cv::Vec4f> cv_cloud,
                 pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud);
                 
/**
 * @brief Convert openCV matrix points to pcl point cloud.
 * @param cv_cloud Point cloud in openCV matrix format
 * @param pcl_cloud PCL style point cloud
 */
void Cv2PCLCloud(cv::Mat_<cv::Vec4f> cv_cloud,
                 pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud);
                 
/**
 * TODO Antiquated?
 * @brief Convert CogX points to pcl color point cloud.
 * @param points Point cloud in cogX framework format
 * @param cloud PCL style point cloud
 */
void Points2PCLColCloud(const std::vector<PointCloud::SurfacePoint> points, 
                        pcl::PointCloud<pcl::PointXYZRGB> &cloud);


/**
 * @brief Convert CogX points to pcl point cloud.
 * @param points Point cloud in cogX framework format
 * @param cloud PCL style point cloud
 */
void Points2PCLCloud(const std::vector<PointCloud::SurfacePoint> points, 
                     pcl::PointCloud<pcl::PointXYZRGB> &cloud);


/**
 * @brief Convert a point cloud from pcl-format to opencv vector format.
 * @param cloud Point cloud in pcl-format.
 * @param cvCloud Point cloud as openCV vector.
 * @param random_color Convert cloud with random color.
 */
void PCLCloud2CvVec(pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud, 
                      std::vector<cv::Vec4f> &cvCloud,
                      bool random_colors = false);
                      
/**
 * @brief Convert point clouds from pcl-format to opencv vector format.
 * @param pcl_clouds Point clouds in pcl-format.
 * @param cv_clouds Point clouds as openCV vector.
 * @param random_color Convert cloud with random color.
 */              
void PCLClouds2CvVecs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clouds,
                      std::vector< std::vector<cv::Vec4f> > &cv_clouds,
                      bool random_colors = false);
                      
/**
 * @brief Convert a point cloud from pcl-format to opencv matrix format.
 * @param cloud Points cloud in pcl-format.
 * @param cvCloud Point cloud in openCV format.
 * @param random_color Convert cloud with random color.
 */
void PCLCloud2CvMat(pcl::PointCloud<pcl::PointXYZRGB> cloud, 
                      cv::Mat_<cv::Vec4f> &cvCloud,
                      bool random_colors = false);

/**
 * // TODO Random color!!!
 * @brief Convert point clouds from pcl-format to opencv matrix format.
 * @param pcl_clouds Points clouds in pcl-format.
 * @param cv_clouds Point clouds in openCV format.
 */
void PCLClouds2CvMats(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clouds,
                        std::vector< cv::Mat_<cv::Vec4f> > &cv_clouds,
                        bool random_colors = false);

/**
 * @brief Print the points of the cloud to the console.
 * @param cloud Point cloud
 */
void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);


/**
 * @brief Remove points with z=0 from the pcl point cloud.
 * @param cloud PCL point cloud
 */
// void RemoveZeros(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
void RemoveNormalZeros(pcl::PointCloud<pcl::Normal> &cloud);

/**
 * @brief Get the inlier cloud of a point cloud after segmentation.
 * @param cloud Full PCL point cloud
 * @param inliers Inlier indices
 * @param inlierCloud Resulting inlier cloud
 */
void GetInlierCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierCloud);
void GetInlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZRGB> &inlierCloud);
  

/**
 * @brief Get the outlier cloud of a point cloud after segmentation.
 * @param cloud Full PCL point cloud
 * @param inliers Inlier indices
 * @param outlierCloud Resulting outlier cloud
 */
void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                     pcl::PointIndices inliers, 
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlierCloud);
void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers);                        /// => Returns cloud as outlier cloud!
void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers, 
                     pcl::PointCloud<pcl::PointXYZ> &outlierCloud);
void GetOutlierNormalCloud(pcl::PointCloud<pcl::Normal> &cloud, 
                           pcl::PointIndices inliers, 
                           pcl::PointCloud<pcl::Normal> &outlierCloud);


/**
 * @brief Calculate an optimal SAC distance threshold for fitting planes, 
 * dependent on distance, for the Kinect sensor
 * @param pcl_cloud Point cloud
 * @param sac_distance Optimal SAC threshold
 * @param weight_factor A higher weight factor bred to consider the distance
 * to the point cloud more and more.
 */
void CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                       double &sac_distance, 
                                       double weight_factor = 1.5);


/**
 * @brief Estimate average distance to point cloud.
 * @param pcl_cloud Point cloud
 * @param distance Average distance to point cloud
 */
void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, double &distance);

}

#endif