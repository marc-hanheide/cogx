/**
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Utils for calculations with PCL and openCV prototypes.
 */

#include "PCLUtils.h"
#include "cmath"

#include "highgui.h"
#include "cv.h"

namespace pclA {

void ConvertCvVec2PCLCloud(const std::vector<cv::Vec4f> &cv_cloud, 
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  if( pcl_cloud.get() == 0 )
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_cloud->width = cv_cloud.size();
  pcl_cloud->height = 1;
  pcl_cloud->points.resize(cv_cloud.size());
  for( unsigned idx = 0; idx < cv_cloud.size(); idx++ )
  {
    pcl_cloud->points[idx].x = (float) cv_cloud[idx][0];
    pcl_cloud->points[idx].y = (float) cv_cloud[idx][1];
    pcl_cloud->points[idx].z = (float) cv_cloud[idx][2];
    pcl_cloud->points[idx].rgb = (float) cv_cloud[idx][3];
  }
}

void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud, 
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  int pos = 0;
  if( pcl_cloud.get() == 0 )
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_cloud->width = cv_cloud.cols;
  pcl_cloud->height = cv_cloud.rows;
  pcl_cloud->points.resize(cv_cloud.cols * cv_cloud.rows);

  for( unsigned row = 0; row < cv_cloud.rows; row++ )
  {
    for( unsigned col = 0; col < cv_cloud.cols; col++ )
    {
      pos = row*cv_cloud.cols + col;
      pcl_cloud->points[pos].x = cv_cloud(row, col)[0];
      pcl_cloud->points[pos].y = cv_cloud(row, col)[1];
      pcl_cloud->points[pos].z = cv_cloud(row, col)[2];
      pcl_cloud->points[pos].rgb = cv_cloud(row, col)[3];
    }
  }
}

void ConvertCvMat2PCLNormals(const cv::Mat_<cv::Vec4f> &cv_normals,
                             pcl::PointCloud<pcl::Normal>::Ptr &normals)    
{
  int pos = 0;
  if(normals.get() == 0 )
    normals.reset(new pcl::PointCloud<pcl::Normal>);

  normals->width = cv_normals.cols;
  normals->height = cv_normals.rows;
  normals->points.resize(cv_normals.cols * cv_normals.rows);

  for( unsigned row = 0; row < cv_normals.rows; row++ )
  {
    for( unsigned col = 0; col < cv_normals.cols; col++ )
    {
      pos = row*cv_normals.cols + col;
      normals->points[pos].normal_x = cv_normals(row, col)[0];
      normals->points[pos].normal_y = cv_normals(row, col)[1];
      normals->points[pos].normal_z = cv_normals(row, col)[2];
      normals->points[pos].curvature = cv_normals(row, col)[3];  /// curvature?
    }
  }
}
                             
void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           std::vector<cv::Vec4f> &cvCloud,
                           bool random_colors)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  pclA::RGBValue color;
  if( random_colors )
  {
    color.r = rand() % 255;
    color.g = rand() % 255;
    color.b = rand() % 255;
    color.a = 0;
  }
  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      cv::Vec4f p;
      position = row * pcWidth + col;
      p[0] = pcl_cloud->points[position].x;
      p[1] = pcl_cloud->points[position].y;
      p[2] = pcl_cloud->points[position].z;
      if( random_colors )
        p[3] = color.float_value;
      else
        p[3] = pcl_cloud->points[position].rgb;
      cvCloud.push_back(p);
    }
  }
}

void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, std::vector<cv::Vec3f> &cvCloud)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      cv::Vec3f p;
      position = row * pcWidth + col;
      p[0] = pcl_cloud->points[position].x;
      p[1] = pcl_cloud->points[position].y;
      p[2] = pcl_cloud->points[position].z;

      cvCloud.push_back(p);
    }
  }
}

void ConvertPCLClouds2CvVecs(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clouds,
    std::vector<std::vector<cv::Vec4f> > &cv_clouds, bool random_colors)
{
  std::vector<cv::Vec4f> cv_cloud;
  for( unsigned idx = 0; idx < pcl_clouds.size(); idx++ )
  {
    ConvertPCLCloud2CvVec(pcl_clouds[idx], cv_cloud, random_colors);
    cv_clouds.push_back(cv_cloud);
    cv_cloud.clear();
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat_<cv::Vec4f> &cvCloud,
    bool random_colors)
{
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  RGBValue color;
  if( random_colors )
  {
    color.r = rand() % 255;
    color.g = rand() % 255;
    color.b = rand() % 255;
    color.a = 0;
  }

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].x;
      p[1] = cloud->points[position].y;
      p[2] = cloud->points[position].z;
      if( random_colors )
        p[3] = color.float_value;
      else
        p[3] = cloud->points[position].rgb;
    }
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, cv::Mat_<cv::Vec4f> &cvCloud,
    float z_min, float z_max)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {

      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];
      if( pt.z <= z_max && pt.z >= z_min )
      {
        p[0] = pt.x;
        p[1] = pt.y;
        p[2] = pt.z;
        p[3] = pt.rgb;
      } else
      {
        p = cv::Vec4f(NAN, NAN, NAN, NAN);
      }
    }
  }
}

void ConvertPCLClouds2CvMats(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clouds,
    std::vector<cv::Mat_<cv::Vec4f> > &cv_clouds, bool random_colors)
{
  cv::Mat_<cv::Vec4f> cv_cloud;
  for( unsigned idx = 0; idx < pcl_clouds.size(); idx++ )
  {
    ConvertPCLCloud2CvMat(pcl_clouds[idx], cv_cloud, random_colors);
    cv_clouds.push_back(cv_cloud);
  }
}

void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  pclA::RGBValue color;

  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      if( pt.rgb != pt.rgb )
      {
        cvp[0] = 0;
        cvp[1] = 0;
        cvp[2] = 0;
      } else
      {
        color.float_value = pt.rgb;
        cvp[0] = color.r;
        cvp[1] = color.g;
        cvp[2] = color.b;
      }
    }
  }
}

void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud, 
                           cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  pclA::RGBValue color;

  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      if( pt.rgb != pt.rgb )
      {
        cvp[0] = 0;
        cvp[1] = 0;
        cvp[2] = 0;
      } else
      {
        color.float_value = pt.rgb;
        cvp[0] = color.r;
        cvp[1] = color.g;
        cvp[2] = color.b;
      }
    }
  }
}

void ConvertCvMat2Image(const cv::Mat_<float> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate)
{
  int w = 0;
  for( int v = 0; v < (int) mat_image.rows; ++v )
  {
    for( int u = 0; u < (int) mat_image.cols; ++u )
    {
      if(invert_y_coordinate) w = mat_image.rows-v-1;
      else w = v;
      const float &value = mat_image(w,u);
      cv::Vec3b &pt = image(w, u);
      
      pt[0] = (uchar) (value*255.);
      pt[1] = (uchar) (value*255.);
      pt[2] = (uchar) (value*255.);
    }      
  }
}

void ConvertCvMat2Image(const cv::Mat_<uchar> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate)
{
  int w = 0;
  for( int v = 0; v < (int) mat_image.rows; ++v )
  {
    for( int u = 0; u < (int) mat_image.cols; ++u )
    {
      if(invert_y_coordinate) w = mat_image.rows-v-1;
      else w = v;
      const uchar &value = mat_image(w,u);
      cv::Vec3b &pt = image(w, u);
      
      pt[0] = (uchar) (value);
      pt[1] = (uchar) (value);
      pt[2] = (uchar) (value);
    }      
  }
}

void ConvertCvMat2Image(const cv::Mat_<cv::Vec3f> &mat_image, 
                        cv::Mat_<cv::Vec3b> &image,
                        bool invert_y_coordinate)
{
printf("VisionUtils::ConvertCVMat2Image: Time to implement!\n");
//   int w = 0;
//   for( int v = 0; v < (int) mat_image.rows; ++v )
//   {
//     for( int u = 0; u < (int) mat_image.cols; ++u )
//     {
//       if(invert_y_coordinate) w = mat_image.rows-v-1;
//       else w = v;
//       const cv::Vec3f &value = mat_image(w,u);
//       cv::Vec3b &pt = image(w, u);
//       
//       pt[0] = (uchar) (value);
//       pt[1] = (uchar) (value);
//       pt[2] = (uchar) (value);
//     }      
//   }
}

void ConvertIndexes2Mask(std::vector<int> &indexes,
                          cv::Mat_<cv::Vec3b> &patch_mask,
                          int width, int height)
{
  if(patch_mask.empty())
    patch_mask = cv::Mat_<cv::Vec3b>::zeros(height, width);

  cv::Mat_<cv::Vec3b> patch_edge = cv::Mat_<cv::Vec3b>::zeros(patch_mask.rows, patch_mask.cols);

  for(size_t i=0; i<indexes.size(); i++)
  {
    int row = indexes[i] / patch_mask.cols;
    int col = indexes[i] % patch_mask.cols;
    cv::Vec3b &pt = patch_mask(row, col);
    pt[0] = 255;
    pt[1] = 255;
    pt[2] = 255;
  }
}

void ConvertMask2Edges(const cv::Mat_<cv::Vec3b> &mask,
                       cv::Mat_<cv::Vec3b> &edge)
{
  if(mask.empty()) {
    printf("PCLUtils::ConvertMask2Edges: Error: mask is empty!\n"); return; }
  if(edge.empty())
    edge = cv::Mat_<cv::Vec3b>::zeros(mask.rows, mask.cols);

  for( int v = 1; v < (int) mask.cols-1; ++v )
  {
    for( int u = 1; u < (int) mask.rows-1; ++u )
    {
      cv::Vec3b &pt_e = edge(u, v);
      if(mask(u, v)[0] != 0)
      {
        if(mask(u+1, v)[0] == 0 || mask(u-1, v)[0] == 0 ||
           mask(u, v+1)[0] == 0 || mask(u, v-1)[0] == 0)
        {
          pt_e[0] = 255;
          pt_e[1] = 255;
          pt_e[2] = 255;
        }
      }
    }
  }
}

void ConvertEdges2Indexes(const cv::Mat_<cv::Vec3b> &edge,
                          std::vector<int> &indexes)
{
  if(edge.empty()) {
    printf("PCLUtils::ConvertMask2Edges: Error: mask is empty!\n"); return; }
  if(indexes.size() != 0) indexes.resize(0);

  for( int v = 0; v < (int) edge.cols; ++v )
  {
    for( int u = 0; u < (int) edge.rows; ++u )
    {
      if(edge(u,v)[0] != 0)
        indexes.push_back(u*edge.cols + v);
    }
  }
}

void ShowNormalImage(const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  unsigned pcWidth = normals->width;
  unsigned pcHeight = normals->height;
  int pos = 0;

  cv::Mat_<cv::Vec3b> x_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);
  cv::Mat_<cv::Vec3b> y_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);
  cv::Mat_<cv::Vec3b> z_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  float x, y, z;
  int i_x, i_y, i_z;
  for( int v = 0; v < (int) pcWidth; ++v )
  {
    for( int u = 0; u < (int) pcHeight; ++u )
    {
      pos = u*pcWidth + v;
      const pcl::Normal &normal = normals->points[pos];
      cv::Vec3b &x_pt = x_image(u, v);
      cv::Vec3b &y_pt = y_image(u, v);
      cv::Vec3b &z_pt = z_image(u, v);
      
      x = (normals->points[pos].normal_x + 1)*255./2.;
      y = (normals->points[pos].normal_y + 1)*255./2.;
      z = (normals->points[pos].normal_z + 1)*255./2.;
      
      i_x = (int) x;
      i_y = (int) y;
      i_z = (int) z;
      
// printf("values: %4.3f - %4.3f - %4.3f\n", (normals->points[pos].normal_x + 1)/2., (normals->points[pos].normal_y + 1)/2., (normals->points[pos].normal_z + 1)/2.);
      x_pt[0] = (uchar) i_x;
      x_pt[1] = 0;
      x_pt[2] = 0;

      y_pt[0] = 0;
      y_pt[1] = (uchar) i_y;
      y_pt[2] = 0;

      z_pt[0] = 0;
      z_pt[1] = 0;
      z_pt[2] = (uchar) i_z;
      
//printf("values: %4.3f - %4.3f - %4.3f\n", normals->points[pos].normal_x, normals->points[pos].normal_y, normals->points[pos].normal_z);
// printf("values: %4.3f - %4.3f - %4.3f\n", x, y, z);
// printf("values: %u - %u - %u\n", i_x, i_y, i_z);
// printf("values: %u - %u - %u\n", x_pt[0], y_pt[1], z_pt[2]);
    }      
  }
  
  cv::imshow("X-Normals", x_image);
  cv::imshow("Y-Normals", y_image);
  cv::imshow("Z-Normals", z_image);
}


void ConvertPCLCloud2Mask(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                          cv::Mat_<uchar> &mask,
                          bool treat_zeros_as_nan, 
                          bool treat_floatmax_as_nan,
                          bool use_z_filter,
                          double zMin,
                          double zMax)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  mask = cv::Mat_<uchar>(pcHeight, pcWidth);

  for( unsigned row = 0; row < pcHeight; row++ )
  {
    for( unsigned col = 0; col < pcWidth; col++ )
    {
      uchar &m = mask(row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      if( pt.z < zMin || pt.z > zMax)
        m = 0;
      else if( pt.x != pt.x || pt.y != pt.y || pt.z != pt.z )
        m = 0;
      else if( treat_zeros_as_nan && pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0 )
        m = 0;
      else if( treat_floatmax_as_nan && pt.x == FLT_MAX && pt.y == FLT_MAX && pt.z == FLT_MAX )
        m = 0;
      else
        m = 255;
    }
  }
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src, 
                    pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  dst.header = src.header;
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst)
{
  if(dst.get() == 0)
    dst.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
  dst->header = src->header;
  dst->width = src->width;
  dst->height = src->height;
  dst->is_dense = src->is_dense;
  dst->points = src->points;
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src, const pcl::PointIndices indices,
    pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  printf("PCLUtils::CopyPointCloud: Experimental function!\n");
  dst.header = src.header;
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;

  for( unsigned i = 0; i < src.points.size(); i++ )
  {
    dst.points[i].x = NAN;
    dst.points[i].y = NAN;
    dst.points[i].z = NAN;
    dst.points[i].rgb = 0;
  }

  for( unsigned i = 0; i < indices.indices.size(); i++ )
    dst.points[i] = src.points[i];
  printf("PCLUtils::CopyPointCloud: Experimental function => ended!\n");
}

void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  printf("PCL Cloud:\n");
  for( unsigned idx = 0; idx < pcl_cloud->points.size(); idx++ )
  {
    RGBValue color;
    color.float_value = pcl_cloud->points[idx].rgb;
    printf(" point[%u]: %4.4f / %4.4f / %4.4f with rgb: %u / %u / %u\n", idx, pcl_cloud->points[idx].x,
        pcl_cloud->points[idx].y, pcl_cloud->points[idx].z, color.r, color.g, color.b);
  }
}

void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for( unsigned idx = 0; idx < pcl_cloud->points.size(); idx++ )
  {
    if( pcl_cloud->points[idx].z != 0. && (pcl_cloud->points[idx].x == pcl_cloud->points[idx].x) )
      newCloud->points.push_back(pcl_cloud->points[idx]);
  }
  pcl_cloud = newCloud;
}

void RemoveZerosIndexed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        std::vector<int> &indexes)
{
  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLUtils::RemoveZerosIndexed: Warning: Size of point cloud and indexes differ: re-indexing.\n");
    indexes.resize(0);
    for(unsigned i=0; i<pcl_cloud->points.size(); i++)
      indexes.push_back(i);
  }
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> new_indexes;
  for( unsigned idx = 0; idx < pcl_cloud->points.size(); idx++ )
  {
    if( pcl_cloud->points[idx].z != 0. && (pcl_cloud->points[idx].x == pcl_cloud->points[idx].x) )
    {
      newCloud->points.push_back(pcl_cloud->points[idx]);
      new_indexes.push_back(indexes[idx]);
    }
  }
  pcl_cloud = newCloud;
  indexes = new_indexes;
}

void RemoveNormalZeros(pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr newCloud(new pcl::PointCloud<pcl::Normal>);
  for( unsigned idx = 0; idx < normals->points.size(); idx++ )
  {
    if( normals->points[idx].normal_z != 0. )
      newCloud->points.push_back(normals->points[idx]);
  }
  normals = newCloud;
}

void FilterZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double minZ, double maxZ)
{
printf("FilterZ: Error: Not yet implemented!!!\n");
//  pcl::PassThrough < pcl::PointXYZRGB > zFilter;
//  zFilter.setFilterFieldName("z");
//  zFilter.setFilterLimits(minZ, maxZ);
//  zFilter.setKeepOrganized(true);
//  zFilter.setInputCloud(pcl_cloud);
//  zFilter.filter(*pcl_cloud);
}

void CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                                       double &sac_distance, double weight_factor)
{
  double mean_distance;
  pclA::GetMeanPointCloudDistance(pcl_cloud, mean_distance);
  sac_distance *= mean_distance;
  sac_distance *= weight_factor;
}

void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                               double &distance, int nrOfPoints)
{
  distance = 0.;
  int p[nrOfPoints];
  for( unsigned i = 0; i < nrOfPoints; i++ )
    p[i] = (int) (pcl_cloud->points.size() - 1) * i / (nrOfPoints - 1);

  for( unsigned i = 0; i < nrOfPoints; i++ )
    distance += sqrt(
        pow((double) pcl_cloud->points[p[i]].x, 2) + pow((double) pcl_cloud->points[p[i]].y, 2) + pow(
            (double) pcl_cloud->points[p[i]].z, 2));
  distance /= nrOfPoints;
}

void SubstituteNanValues(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  for(unsigned idx = 0; idx < pcl_cloud->points.size(); idx++)
  {
    if(pcl_cloud->points[idx].x == FLT_MAX)
    {
      pcl_cloud->points[idx].x = NAN;
      pcl_cloud->points[idx].y = NAN;
      pcl_cloud->points[idx].z = NAN;
    }
  }
}

void Dilation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst, 
              double fx, double fy, double cx, double cy, int valid_nghbr)
{
  if(src->width < 2 || src->height < 2)
    printf("PCLUtils::Dilation: Warning: Maybe unordered source point cloud.\n");
  
  dst.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  dst->header = src->header;
  dst->width = src->width;
  dst->height = src->height;
  dst->is_dense = src->is_dense;
  dst->points = src->points;
  
  int pos = 0;
  for(int v=1; v<src->height-1; v++)
  {
    for(int u=1; u<src->width-1; u++)
    {
      pos = v*src->width + u;
      if(src->points[pos].z != src->points[pos].z)
      {
        // Check the four neighbors
        double mean = 0;
        int nr_valid = 0;
        pcl::PointXYZRGB values[4];
        bool isValid[4] = {false, false, false, false};
        if(src->points[pos-src->width].z == src->points[pos-src->width].z && src->points[pos-src->width].z != 0.) {
          values[0] = src->points[pos-src->width];
          mean += values[0].z;
          nr_valid++;
          isValid[0] = true;
          values[0].rgb = src->points[pos-src->width].rgb;
        }
        if(src->points[pos+src->width].z == src->points[pos+src->width].z && src->points[pos+src->width].z != 0.) {
          values[1] = src->points[pos+src->width];
          mean += values[1].z;
          nr_valid++;
          isValid[1] = true;
          values[0].rgb = src->points[pos+src->width].rgb;
        }
        if(src->points[pos-1].z == src->points[pos-1].z && src->points[pos-1].z != 0.) {
          values[2] = src->points[pos-1];
          mean += values[2].z;
          nr_valid++;
          isValid[2] = true;
          values[0].rgb = src->points[pos-1].rgb;
        }
        if(src->points[pos+1].z == src->points[pos+1].z && src->points[pos+1].z != 0.) {
          values[3] = src->points[pos+1];
          mean += values[3].z;
          nr_valid++;
          isValid[3] = true;
          values[0].rgb = src->points[pos+1].rgb;
        }
        if(nr_valid >= valid_nghbr)
        {
          mean /= nr_valid;
          int nr_pos = 0;
          int nr_neg = 0;
          int is_pos[4] = {0,0,0,0};
          if(isValid[0]) {
            if(values[0].z > mean){ nr_pos++; is_pos[0] = 1;} 
            else {nr_neg++; is_pos[0] = -1;}
          }
          if(isValid[1]) {
            if(values[1].z > mean && isValid[1]){ nr_pos++; is_pos[1] = 1;} 
            else {nr_neg++; is_pos[1] = -1;}
          }
          if(isValid[2]) {
            if(values[2].z > mean && isValid[2]){ nr_pos++; is_pos[2] = 1;} 
            else {nr_neg++; is_pos[2] = -1;}
          }
          if(isValid[3]) {
            if(values[3].z > mean && isValid[3]){ nr_pos++; is_pos[3] = 1;} 
            else {nr_neg++; is_pos[3] = -1;}
          }
          
          bool done = false;
          if(nr_pos >= nr_neg){
            for(unsigned i=0; i<4; i++)
              if(is_pos[i] == 1 && !done)
              {
                double camRay_x, camRay_y;
                camRay_x = (float)(u-cx)/fx;
                camRay_y = (float)(v-cy)/fy;
                values[i].x = camRay_x * values[i].z;
                values[i].y = camRay_y * values[i].z;
                done = true;
                dst->points[pos] = values[i];
              }
          }
          else
            for(unsigned i=0; i<4; i++)
              if(is_pos[i] == -1 && !done)
              {
                double camRay_x, camRay_y;
                camRay_x = (float)(u-cx)/fx;
                camRay_y = (float)(v-cy)/fy;
                values[i].x = camRay_x * values[i].z;
                values[i].y = camRay_y * values[i].z;
                done = true;
                dst->points[pos] = values[i];
              }
        }
      }
    }
  }
}

} // namespace pcla

