/**
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Utils for calculations with PCL and openCV prototypes.
 */


#ifndef PCLA_PCLUTILS_H
#define PCLA_PCLUTILS_H

#include "v4r/PCLAddOns/utils/PCLCommonHeaders.h"

namespace pclA
{
  
/**
 * @brief RGBValue of point clouds, accessable as float or long value.
 */
typedef union
{
  struct
  {
    unsigned char b;  // Blue channel
    unsigned char g;  // Green channel
    unsigned char r;  // Red channel
    unsigned char a;  // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


/**
 * @brief Convert openCV vector points to pcl point cloud.
 * @param cv_cloud Point cloud in openCV vector format
 * @param pcl_cloud PCL style point cloud
 */
void ConvertCvVec2PCLCloud(const std::vector<cv::Vec4f> &cv_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud);
                 
/**
 * @brief Convert openCV matrix points to pcl point cloud.
 * @param cv_cloud Point cloud in openCV matrix format
 * @param pcl_cloud PCL style point cloud
 */
void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud);
                 

/**
 * @brief Convert a point cloud from pcl-format to opencv vector format.
 * @param cloud Point cloud in pcl-format.
 * @param cvCloud Point cloud as openCV vector.
 * @param random_color Convert cloud with random color.
 */
void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, 
                           std::vector<cv::Vec4f> &cvCloud,
                           bool random_colors = false);
                      
/**
 * @brief Convert point clouds from pcl-format to opencv vector format.
 * @param pcl_clouds Point clouds in pcl-format.
 * @param cv_clouds Point clouds as openCV vector.
 * @param random_color Convert cloud with random color.
 */              
void ConvertPCLClouds2CvVecs(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                             std::vector< std::vector<cv::Vec4f> > &cv_clouds,
                             bool random_colors = false);
                      
/**
 * @brief Convert a point cloud from pcl-format to opencv matrix format.
 * @param cloud Points cloud in pcl-format.
 * @param cvCloud Point cloud in openCV format.
 * @param random_color Convert cloud with random color.
 */
void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           bool random_colors = false);
void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           float z_min, float z_max);

/**
 * // TODO Random color!!!
 * @brief Convert point clouds from pcl-format to opencv matrix format.
 * @param pcl_clouds Points clouds in pcl-format.
 * @param cv_clouds Point clouds in openCV format.
 */
void ConvertPCLClouds2CvMats(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                             std::vector< cv::Mat_<cv::Vec4f> > &cv_clouds,
                             bool random_colors = false);

                               
/**
 * @brief Convert point cloud to image. Only for dense point clouds with correct header.
 * @param cloud Point cloud to be transformed.
 * @param image Image as openCV matrix with 3b-vectors
 */
void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<cv::Vec3b> &image);

/**
 * @brief Creates mask from point cloud.
 * @param cloud Point cloud to get mask from.
 * @param mask Image indicating whether a point in the cloud is available (255) or not (0)
 */
void ConvertPCLCloud2Mask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<uchar> &mask);

/**
 * @brief Convert point cloud to normal map. Only for dense point clouds with correct header.
 * @param cloud Point cloud to be transformed.
 * @param normals Normal map as openCV matrix with 4f-vectors (4th element is curvature)
 */
void ConvertPCLCloud2Normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           cv::Mat_<cv::Vec4f> &normals, float radius_search);
                             
/**
 * @brief Copy one point cloud to another without loosing the array structure.
 * @param src Source point cloud to copy.
 * @param dst Destination point cloud.
 */             
void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src,
                    pcl::PointCloud<pcl::PointXYZRGB> &dst);
           

/** TODO Experimental => Without loosing the array structure => setting other points to NAN
 * @brief Copy one point cloud to another without loosing the array structure
 * and using point indices
 * @param src Source point cloud to copy.
 * @param indices Point indices
 * @param dst Destination point cloud.
 */             
void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src,
                    const pcl::PointIndices indices,
                    pcl::PointCloud<pcl::PointXYZRGB> &dst);
                           
                           
/**
 * @brief Print the points of the cloud to the console.
 * @param cloud Point cloud
 */
void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);


/**
 * @brief Remove points with z=0 from the pcl point cloud.
 * @param cloud PCL point cloud
 */
void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB> &cloud);
void RemoveNormalZeros(pcl::PointCloud<pcl::Normal> &cloud);


/**
 * @brief Get the outlier cloud of a point cloud after segmentation.
 * @param cloud Full PCL point cloud
 * @param inliers Inlier indices
 * @param outlierCloud Resulting outlier cloud
 */
void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers);
void GetOutlierCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers, 
                     pcl::PointCloud<pcl::PointXYZ> &outlierCloud);
void GetOutlierNormalCloud(const pcl::PointCloud<pcl::Normal> &cloud, 
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
 * @brief Estimate average distance to point cloud. Choose a number of points to estimate
 * the average distance.
 * @param pcl_cloud Point cloud
 * @param distance Average distance to point cloud
 * @param nrOfPoints Used number of points to calculate the mean distance.
 */
void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                               double &distance, 
                               int nrOfPoints = 5);
                               



}

#endif
