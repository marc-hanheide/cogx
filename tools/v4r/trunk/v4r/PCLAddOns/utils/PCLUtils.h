/**
 * @file PCLUtils.h
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
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);
                 
/**
 * @brief Convert openCV matrix points to pcl point cloud.
 * @param cv_cloud Point cloud in openCV matrix format
 * @param pcl_cloud PCL style point cloud
 */
void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);                 

/**
 * @brief Convert openCV matrix normals to pcl point cloud normals.
 * @param cv_normals Point cloud of normals in openCV matrix format
 * @param normals PCL style point cloud normals
 */
void ConvertCvMat2PCLNormals(const cv::Mat_<cv::Vec4f> &cv_normals,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals);                 

/**
 * @brief Convert a point cloud from pcl-format to opencv vector format.
 * @param cloud Point cloud in pcl-format.
 * @param cvCloud Point cloud as openCV vector.
 */
void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           std::vector<cv::Vec3f> &cvCloud);
                      
/**
 * @brief Convert a point cloud from pcl-format to opencv vector format.
 * @param cloud Point cloud in pcl-format.
 * @param cvCloud Point cloud as openCV vector.
 * @param random_color Convert cloud with random color.
 */
void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
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
 * Second function uses a additional z-filter with min and max values.
 * @param pcl_cloud Points cloud in pcl-format.
 * @param cvCloud Point cloud in openCV format.
 * @param random_color Convert cloud with random color.
 * @param z_min Minium z-value
 * @param z_max Maximum z-value
 */
void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           bool random_colors = false);
void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
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
void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                           cv::Mat_<cv::Vec3b> &image);
void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                           cv::Mat_<cv::Vec3b> &image);

/**
 * @brief Convert a cv::Mat (float, uchar) image to a grey-level rgb-image.
 * @param mat_image Float-image
 * @param image Image as openCV matrix with 3b-vectors
 * @param invert_y_coordinate Invert the y-coordinate (OpenGL-style)
 */
void ConvertCvMat2Image(const cv::Mat_<float> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate);

void ConvertCvMat2Image(const cv::Mat_<uchar> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate);

void ConvertCvMat2Image(const cv::Mat_<cv::Vec3f> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate);

                        
/**
 * @brief Convert a index vector to a mask image.
 * @param indexes Vector with indexes
 * @param mask Mask image
 * @param width Patch mask width (if not initialized)
 * @param height Patch mask height (if not initialized)
 */            
void ConvertIndexes2Mask(std::vector<int> &indexes,
                         cv::Mat_<cv::Vec3b> &mask,
                         int width = 640, int height = 480);

/**
 * @brief Extract from a binary mask the edges.
 * @param mask Mask image
 * @param edge Edge image
 */ 
void ConvertMask2Edges(const cv::Mat_<cv::Vec3b> &mask,
                       cv::Mat_<cv::Vec3b> &edge);
/**
 * @brief Convert edge image to index vector
 * @param edge Edge image
 * @param indexes Index vector
 */ 
void ConvertEdges2Indexes(const cv::Mat_<cv::Vec3b> &edge,
                          std::vector<int> &indexes);

/**
 * @brief Convert a cv::Mat (float, uchar) image to a grey-level rgb-image.
 * @param normals Normals
 */
void ShowNormalImage(const pcl::PointCloud<pcl::Normal>::Ptr &normals);


/**
 * @brief Creates mask from point cloud.
 * @param cloud Point cloud to get mask from.
 * @param mask Image indicating whether a point in the cloud is available (255) or not (0)
 * @param treat_zeros_as_nan Remove zero-values
 * @param treat_floatmax_as_nan Remove FLT_MAX values
 * @param use_z_filter Remove points out of z-value range
 * @param zMin Minimum z-value
 * @param zMax Maximum z-value
 */
void ConvertPCLCloud2Mask(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                          cv::Mat_<uchar> &mask, 
                          bool treat_zeros_as_nan=true, 
                          bool treat_floatmax_as_nan=true,
                          bool use_z_filter = false,
                          double zMin = 0.0,
                          double zMax = 10.0);

/**
 * @brief Copy one point cloud to another WITHOUT loosing the array structure.
 * @param src Source point cloud to copy.
 * @param dst Destination point cloud.
 */             
void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src,
                    pcl::PointCloud<pcl::PointXYZRGB> &dst);
           
void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst);
           

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
void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);


/**
 * @brief Remove points with z=0 and all nan's from the pcl point cloud.
 * SAC and VoxelGrid is not able to handle these points.
 * @param cloud PCL point cloud
 */
void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);
void RemoveZerosIndexed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                        std::vector<int> &indexes);
void RemoveNormalZeros(pcl::PointCloud<pcl::Normal>::Ptr &cloud);


/**
 * @brief Filter point cloud by z-value (distance)
 * @param pcl_cloud Full PCL point cloud
 * @param minZ Minimum z-value
 * @param maxZ Maximum z-value
 */
void FilterZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
             double minZ, double maxZ);


/**
 * @brief Calculate an optimal SAC distance threshold for fitting planes, 
 * dependent on distance, for the Kinect sensor
 * @param pcl_cloud Point cloud
 * @param sac_distance Optimal SAC threshold
 * @param weight_factor A higher weight factor bred to consider the distance
 * to the point cloud more and more.
 */
void CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                                       double &sac_distance, 
                                       double weight_factor = 1.5);


/**
 * @brief Estimate average distance to point cloud. Choose a number of points to estimate
 * the average distance.
 * @param pcl_cloud Point cloud
 * @param distance Average distance to point cloud
 * @param nrOfPoints Used number of points to calculate the mean distance.
 */
void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                               double &distance, 
                               int nrOfPoints = 5);

/**
 * @brief Rearange the point cloud: Remove FLT_MAX values and insert instead again nan's
 * @param pcl_cloud Point cloud to rearange
 */
void SubstituteNanValues(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);


/**
 * @brief Dilation 
 * @param pcl_cloud_src Source point cloud (ordered)
 * @param pcl_cloud_dst Destination point cloud.
 * @param fx-fy-cx-cy Camera parameters (standard-values for Kinect with 640x480)
 * @param valid_nghbr Number of required valid neighbors (of 4)
 */
void Dilation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst, 
              double fx = 525., double fy = 525., double cx = 320., double cy = 240., 
              int valid_nghbr = 2);

}

#endif
