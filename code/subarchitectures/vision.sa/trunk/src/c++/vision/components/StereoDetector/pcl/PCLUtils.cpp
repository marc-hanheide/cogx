/**
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Utils for calculations with PCL, openCV and cast prototypes.
 */

#include "PCLUtils.h"
 
namespace pclU
{
  
  
void Cv2PCLCloud(std::vector<cv::Vec4f> cv_cloud,
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
  
void Cv2PCLCloud(cv::Mat_<cv::Vec4f> cv_cloud,
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

void Points2PCLColCloud(const std::vector<PointCloud::SurfacePoint> points, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
  unsigned pcWidth = sqrt(points.size()*4/3);
  unsigned pcHeight = pcWidth *3/4;
  unsigned position = 0;
  
  cloud.points.resize(pcWidth*pcHeight);
  
  RGBValue color;
  color.Alpha = 1.;
  
  for(unsigned row = 0; row < pcHeight; row++)
  {
    for(unsigned col = 0; col < pcWidth; col++)
    {
      position = row*pcWidth + col;

      color.Red = points[position].c.r;
      color.Green = points[position].c.g;
      color.Blue = points[position].c.b;

      cloud.width = pcWidth;
      cloud.height = pcHeight;
      cloud.points[row*pcWidth+col].x = (float) points[position].p.x;
      cloud.points[row*pcWidth+col].y = (float) points[position].p.y;
      cloud.points[row*pcWidth+col].z = (float) points[position].p.z;
      cloud.points[row*pcWidth+col].rgb = color.float_value;
    }
  }
}

void Points2PCLCloud(const std::vector<PointCloud::SurfacePoint> points, 
                     pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  unsigned pcWidth = sqrt(points.size()*4/3);
  unsigned pcHeight = pcWidth *3/4;
  unsigned position = 0;
  
  cloud.width = pcWidth;
  cloud.height = pcHeight;
  cloud.points.resize(pcWidth*pcHeight);
  
  for(unsigned row = 0; row < pcHeight; row++)
  {
    for(unsigned col = 0; col < pcWidth; col++)
    {
      position = row*pcWidth + col;

      cloud.points[row*pcWidth+col].x = (float) points[position].p.x;
      cloud.points[row*pcWidth+col].y = (float) points[position].p.y;
      cloud.points[row*pcWidth+col].z = (float) points[position].p.z;
    }
  }
}


void PCLCloud2CvVec(pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud, 
                    std::vector<cv::Vec4f> &cvCloud,
                    bool random_colors)
{
  unsigned pcWidth = pcl_cloud.width;
  unsigned pcHeight = pcl_cloud.height;
  unsigned position = 0;
  
  RGBValue color;
  if(random_colors)
  {
    color.Red = rand()%255;
    color.Green = rand()%255;
    color.Blue = rand()%255;
    color.Alpha = 0;
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

void PCLClouds2CvVecs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clouds,
                      std::vector< std::vector<cv::Vec4f> > &cv_clouds,
                      bool random_colors)
{
  std::vector<cv::Vec4f> cv_cloud;
  for(unsigned idx=0; idx< pcl_clouds.size(); idx++)
  {
    PCLCloud2CvVec(*pcl_clouds[idx], cv_cloud, random_colors); 
    cv_clouds.push_back(cv_cloud);
    cv_cloud.clear();
  }
}

void PCLCloud2CvMat(pcl::PointCloud<pcl::PointXYZRGB> cloud, 
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
    color.Red = rand()%255;
    color.Green = rand()%255;
    color.Blue = rand()%255;
    color.Alpha = 0;
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

void PCLClouds2CvMats(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clouds,
                        std::vector< cv::Mat_<cv::Vec4f> > &cv_clouds,
                        bool random_colors)
{
  cv::Mat_<cv::Vec4f> cv_cloud;
  for(unsigned idx=0; idx< pcl_clouds.size(); idx++)
  {
    PCLCloud2CvMat(*pcl_clouds[idx], cv_cloud, random_colors); 
    cv_clouds.push_back(cv_cloud);
  }
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
           color.Red, color.Green, color.Blue);
  }
}


// void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
// {
//   printf("Remove zeros: cloud: %u\n", cloud->points.size());
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//   for(unsigned idx=0; idx < cloud->points.size(); idx++)
//   {
//     if(cloud->points[idx].z != 0.)
//       newCloud->points.push_back(cloud->points[idx]);
// //     else printf("Zero found! ");
//   }
//   cloud = newCloud;
//   printf("Remove zeros ended: cloud: %u\n", cloud->points.size());
// }

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

void GetInlierCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices inliers,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierCloud)
{
printf("PCLUtils::GetInlierCloud: Not yet implemented!\n");
//   inlierCloud->points.resize(inliers.indices.size());
//   inlierCloud->width = inliers.indices.size();
//   inlierCloud->height = 1;
//   for (size_t i = 0; i < inliers.indices.size(); ++i)
//   {
//     inlierCloud->points[i].x = cloud->points[inliers.indices[i]].x;
//     inlierCloud->points[i].y = cloud->points[inliers.indices[i]].y;
//     inlierCloud->points[i].z = cloud->points[inliers.indices[i]].z;
//   }
}

void GetInlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZRGB> &inlierCloud)
{
  inlierCloud.points.resize(inliers.indices.size());
  inlierCloud.width = inliers.indices.size();
  inlierCloud.height = 1;
  for (size_t i = 0; i < inliers.indices.size(); ++i)
  {
    inlierCloud.points[i].x = cloud.points[inliers.indices[i]].x;
    inlierCloud.points[i].y = cloud.points[inliers.indices[i]].y;
    inlierCloud.points[i].z = cloud.points[inliers.indices[i]].z;
    inlierCloud.points[i].rgb = cloud.points[inliers.indices[i]].rgb;
  }
}
 
 
void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlierCloud)
{
  printf("PCLUtils::GetOutlierCloud: Not yet implemented!\n");
//   for (size_t i = 0; i < inliers.indices.size(); ++i)
//     cloud->points[inliers.indices[i]].z = 0.;
// 
//   outlierCloud->width = cloud->points.size() - inliers.indices.size();
//   outlierCloud->height = 1;
//   outlierCloud->points.resize(outlierCloud->width);
//   
// printf("    GetOutlierCloud: cloud: %u\n", cloud->points.size());  
//   RemoveZeros(*cloud);
// printf("    GetOutlierCloud: removed zeros: cloud: %u\n", cloud->points.size());  
// printf("    GetOutlierCloud: outlierCloud->width: %u\n", outlierCloud->width);  
//   
//   outlierCloud = cloud;
}

void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointIndices inliers)
{
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    cloud.points[inliers.indices[i]].z = 0.;

  RemoveZeros(cloud);
}

void GetOutlierCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::PointXYZRGB> &outlierCloud)
{
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    cloud.points[inliers.indices[i]].z = 0.;

  RemoveZeros(cloud);
  outlierCloud = cloud;
}

void GetOutlierNormalCloud(pcl::PointCloud<pcl::Normal> &cloud, pcl::PointIndices inliers, pcl::PointCloud<pcl::Normal> &outlierCloud)
{
  for (size_t i = 0; i < inliers.indices.size(); ++i)
    cloud.points[inliers.indices[i]].normal_z = 0.;

  RemoveNormalZeros(cloud);
  outlierCloud = cloud;
}


void CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, double &sac_distance, double weight_factor)
{
  double mean_distance;
  pclU::GetMeanPointCloudDistance(pcl_cloud, mean_distance);
  sac_distance *= mean_distance;
  sac_distance *= weight_factor;
}


void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, double &distance)
{
  distance = 0.;
  int nrOfPoints = 5;
  
  int p[nrOfPoints];
  for(unsigned i=0; i<nrOfPoints; i++)
    p[i] = (int) (pcl_cloud->points.size()-1)*i/(nrOfPoints-1);

  for(unsigned i=0; i<nrOfPoints; i++)
    distance += sqrt(pow((double)pcl_cloud->points[p[i]].x, 2) + pow((double)pcl_cloud->points[p[i]].y, 2) + pow((double)pcl_cloud->points[p[i]].z, 2));

  distance /= nrOfPoints;
}

}

