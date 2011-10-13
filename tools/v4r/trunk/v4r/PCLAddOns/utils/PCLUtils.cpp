/**
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Utils for calculations with PCL and openCV prototypes.
 */

#include "PCLUtils.h"
#include "cmath"
 
namespace pclA
{
  
  
void ConvertCvVec2PCLCloud(const std::vector<cv::Vec4f> &cv_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud)
{
  pcl_cloud.width = cv_cloud.size();
  pcl_cloud.height = 1;
  pcl_cloud.points.resize(cv_cloud.size());                  
  for(unsigned idx = 0; idx < cv_cloud.size(); idx++)
  {
    pcl_cloud.points[idx].x = (float) cv_cloud[idx][0];
    pcl_cloud.points[idx].y = (float) cv_cloud[idx][1];
    pcl_cloud.points[idx].z = (float) cv_cloud[idx][2];
    pcl_cloud.points[idx].rgb = (float) cv_cloud[idx][3];
  }
}                 
  
void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud,
                           pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud)
{
  pcl_cloud.width = cv_cloud.cols;
  pcl_cloud.height = cv_cloud.rows;
  pcl_cloud.points.resize(cv_cloud.cols*cv_cloud.rows);
  
  for(unsigned row = 0; row < cv_cloud.rows; row++)
  {
    for(unsigned col = 0; col < cv_cloud.cols; col++)
    {
      pcl_cloud.points[row*cv_cloud.cols+col].x = (float) cv_cloud(row, col)[0];
      pcl_cloud.points[row*cv_cloud.cols+col].y = (float) cv_cloud(row, col)[1];
      pcl_cloud.points[row*cv_cloud.cols+col].z = (float) cv_cloud(row, col)[2];
      pcl_cloud.points[row*cv_cloud.cols+col].rgb = (float) cv_cloud(row, col)[3];
    }
  }
}

void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, 
                           std::vector<cv::Vec4f> &cvCloud,
                           bool random_colors)
{
  unsigned pcWidth = pcl_cloud.width;
  unsigned pcHeight = pcl_cloud.height;
  unsigned position = 0;
  
  pclA::RGBValue color;
  if(random_colors)
  {
    color.r = rand()%255;
    color.g = rand()%255;
    color.b = rand()%255;
    color.a = 0;
  }
  for(unsigned row = 0; row < pcHeight; row++)
  {
    for(unsigned col = 0; col < pcWidth; col++)
    {
      cv::Vec4f p;
      position = row*pcWidth + col;
      p[0] = pcl_cloud.points[position].x;
      p[1] = pcl_cloud.points[position].y;
      p[2] = pcl_cloud.points[position].z;
      if(random_colors)
        p[3] = color.float_value;
      else
        p[3] = pcl_cloud.points[position].rgb;
      cvCloud.push_back(p);
    }
  } 
}

void ConvertPCLClouds2CvVecs(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                             std::vector< std::vector<cv::Vec4f> > &cv_clouds,
                             bool random_colors)
{
  std::vector<cv::Vec4f> cv_cloud;
  for(unsigned idx=0; idx< pcl_clouds.size(); idx++)
  {
    ConvertPCLCloud2CvVec(*pcl_clouds[idx], cv_cloud, random_colors); 
    cv_clouds.push_back(cv_cloud);
    cv_cloud.clear();
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                           cv::Mat_<cv::Vec4f> &cvCloud, 
                           bool random_colors)
{
  unsigned pcWidth = cloud.width;
  unsigned pcHeight = cloud.height;
  unsigned position = 0;
  
  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth);    // rows = height / cols = width
  
  RGBValue color;
  if(random_colors)
  {
    color.r = rand()%255;
    color.g = rand()%255;
    color.b = rand()%255;
    color.a = 0;
  }
  
  for(unsigned row = 0; row < pcHeight; row++)
  {
    for(unsigned col = 0; col < pcWidth; col++)
    {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f>(row, col);
      position = row*pcWidth + col;
      p[0] = cloud.points[position].x;
      p[1] = cloud.points[position].y;
      p[2] = cloud.points[position].z;
      if(random_colors)
        p[3] = color.float_value;
      else
        p[3] = cloud.points[position].rgb;
    }
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           float z_min, float z_max)
{
  unsigned pcWidth = cloud.width;
  unsigned pcHeight = cloud.height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth);    // rows = height / cols = width

  for(unsigned row = 0; row < pcHeight; row++)
  {
    for(unsigned col = 0; col < pcWidth; col++)
    {

      cv::Vec4f &p = cvCloud.at<cv::Vec4f>(row, col);
      position = row*pcWidth + col;
      const pcl::PointXYZRGB &pt = cloud.points[position];
      if(pt.z<=z_max && pt.z>=z_min){
		  p[0] = pt.x;
		  p[1] = pt.y;
		  p[2] = pt.z;
		  p[3] = pt.rgb;
      }
    }
  }
}

void ConvertPCLClouds2CvMats(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                             std::vector< cv::Mat_<cv::Vec4f> > &cv_clouds,
                             bool random_colors)
{
  cv::Mat_<cv::Vec4f> cv_cloud;
  for(unsigned idx=0; idx< pcl_clouds.size(); idx++)
  {
    ConvertPCLCloud2CvMat(*pcl_clouds[idx], cv_cloud, random_colors); 
    cv_clouds.push_back(cv_cloud);
  }
}

void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<cv::Vec3b> &image)
{
  pclA::RGBValue col;
  image = cv::Mat_<cv::Vec3b>(cloud.height, cloud.width);

  for (int v = 0; v < (int)cloud.height; ++v)
  {
    for (int u = 0; u < (int)cloud.width; ++u)
    {
      const pcl::PointXYZRGB &pt = cloud(u, v);

      cv::Vec3b &ptCol = image(v,u);
      if (pt.rgb!=pt.rgb)
      {
        ptCol[0] = 0; ptCol[1] = 0; ptCol[2] = 0;
      }
      else
      {
        col.float_value = pt.rgb;
        ptCol[0] = col.r;
        ptCol[1] = col.g;
        ptCol[2] = col.b;
      }
    }
  }
}

void ConvertPCLCloud2Mask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                           cv::Mat_<uchar> &mask)
{
  mask = cv::Mat_<uchar>(cloud.height, cloud.width);

  for (int v = 0; v < (int)cloud.height; ++v)
  {
    for (int u = 0; u < (int)cloud.width; ++u)
    {
      const pcl::PointXYZRGB &pt = cloud(u, v);
      uchar &m = mask(v,u);

	  if(isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
		  m = 0;
	  else
		  m = 255;
    }
  }
}

void ConvertPCLCloud2Normals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           cv::Mat_<cv::Vec4f> &normals, float radius_search)
{
	normals = cv::Mat_<cv::Vec4f>(cloud->height, cloud->width);

  pcl::PointCloud<pcl::Normal> cloud_normals;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius_search);

  ne.compute(cloud_normals);

  for (int v = 0; v < (int)cloud_normals.height; ++v)
  {
    for (int u = 0; u < (int)cloud_normals.width; ++u)
    {
      const pcl::Normal &n = cloud_normals(u, v);

      cv::Vec4f &ptNorm = normals(v,u);

      if (n.normal!=n.normal)
      {
        ptNorm[0]=0.0f; ptNorm[1]=0.0f; ptNorm[2]=0.0f; ptNorm[3]=0.0f;
      }
      else
      {
    	  isnan(n.normal[0]) ? ptNorm[0]=0.0f : ptNorm[0]=n.normal[0];
    	  isnan(n.normal[1]) ? ptNorm[1]=0.0f : ptNorm[1]=n.normal[1];
    	  isnan(n.normal[2]) ? ptNorm[2]=0.0f : ptNorm[2]=n.normal[2];
    	  isnan(n.curvature) ? ptNorm[3]=0.0f : ptNorm[3]=n.curvature;
      }
    }
  }
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src, 
                                 pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  dst.header   = src.header;
  dst.width    = src.width;
  dst.height   = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src,
                    const pcl::PointIndices indices,
                    pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  printf("PCLUtils::CopyPointCloud: Experimental function!\n");
  dst.header   = src.header;
  dst.width    = src.width;
  dst.height   = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;                   

  for(unsigned i=0; i<src.points.size(); i++)
  {
    dst.points[i].x = NAN;
    dst.points[i].y = NAN;
    dst.points[i].z = NAN;
    dst.points[i].rgb = 0;
  }
  
  for(unsigned i=0; i<indices.indices.size(); i++)
    dst.points[i] = src.points[i];                   
  printf("PCLUtils::CopyPointCloud: Experimental function => ended!\n");
}

void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  printf("PCL Cloud:\n");
  for(unsigned idx=0; idx<cloud.points.size(); idx++)
  {
    RGBValue color;
    color.float_value = cloud.points[idx].rgb;
    printf(" point[%u]: %4.4f / %4.4f / %4.4f with rgb: %u / %u / %u\n", 
           idx, cloud.points[idx].x, cloud.points[idx].y, cloud.points[idx].z,
           color.r, color.g, color.b);
  }
}


void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> newCloud;
  for(unsigned idx=0; idx < cloud.points.size(); idx++)
  {
    if(cloud.points[idx].z != 0.)
      newCloud.points.push_back(cloud.points[idx]);
  }
  cloud = newCloud;
}


void RemoveNormalZeros(pcl::PointCloud<pcl::Normal> &cloud)
{
  pcl::PointCloud<pcl::Normal> newCloud;
  for(unsigned idx=0; idx < cloud.points.size(); idx++)
  {
    if(cloud.points[idx].normal_z != 0.)
      newCloud.points.push_back(cloud.points[idx]);
  }
  cloud = newCloud;
}

void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers)
{
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    cloud.points[inliers.indices[i]].z = 0.;
  RemoveZeros(cloud);
}

void GetOutlierCloud(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                     pcl::PointIndices inliers, 
                     pcl::PointCloud<pcl::PointXYZRGB> &outlierCloud)
{
  pcl::copyPointCloud(cloud, outlierCloud);
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    outlierCloud.points[inliers.indices[i]].z = 0.;
  RemoveZeros(outlierCloud);
}

void GetOutlierNormalCloud(const pcl::PointCloud<pcl::Normal> &cloud, 
                           pcl::PointIndices inliers, 
                           pcl::PointCloud<pcl::Normal> &outlierCloud)
{
  pcl::copyPointCloud(cloud, outlierCloud);
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    outlierCloud.points[inliers.indices[i]].normal_z = 0.;
  RemoveNormalZeros(outlierCloud);
}


void CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                       double &sac_distance, 
                                       double weight_factor)
{
  double mean_distance;
  pclA::GetMeanPointCloudDistance(pcl_cloud, mean_distance);
  sac_distance *= mean_distance;
  sac_distance *= weight_factor;
}


void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                               double &distance, 
                               int nrOfPoints)
{
  distance = 0.;
  int p[nrOfPoints];
  for(unsigned i=0; i<nrOfPoints; i++)
    p[i] = (int) (pcl_cloud->points.size()-1)*i/(nrOfPoints-1);

  for(unsigned i=0; i<nrOfPoints; i++)
    distance += sqrt(pow((double)pcl_cloud->points[p[i]].x, 2) + pow((double)pcl_cloud->points[p[i]].y, 2) + pow((double)pcl_cloud->points[p[i]].z, 2));
  distance /= nrOfPoints;
}

}

