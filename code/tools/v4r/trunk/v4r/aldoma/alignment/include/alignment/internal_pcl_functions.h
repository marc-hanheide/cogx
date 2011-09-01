#ifndef INTERNAL_PCL_FUNCTIONS_H_
#define INTERNAL_PCL_FUNCTIONS_H_

#include <pcl/common/common.h>
#include <limits>

namespace internal_pcl
{

  inline float
  getMaxHeight (pcl::PointCloud<pcl::PointXYZ> & cluster)
  {
    float MAX_Y = std::numeric_limits<float>::min ();
    for (size_t i = 0; i < cluster.points.size (); i++)
    {
      //std::cout << cluster.points[i].y << std::endl;
      if (cluster.points[i].y > MAX_Y)
      {
        MAX_Y = cluster.points[i].y;
      }
    }

    return MAX_Y;
  }

  inline float
  getMinMaxDist (pcl::PointCloud<pcl::PointXYZ> & pcd_i, pcl::PointCloud<pcl::PointXYZ> & pcd_j)
  {
    //Compute distance for neighborhood search
    Eigen::Vector4f com_i_t, com_j_t;
    pcl::compute3DCentroid (pcd_i, com_i_t);
    pcl::compute3DCentroid (pcd_j, com_j_t);

    Eigen::Vector4f max_i_pt, max_j_pt;
    pcl::getMaxDistance (pcd_i, com_i_t, max_i_pt);
    pcl::getMaxDistance (pcd_j, com_j_t, max_j_pt);

    float max_dist1, max_dist2;
    max_dist1 = (max_i_pt - com_i_t).norm ();
    max_dist2 = (max_j_pt - com_j_t).norm ();

    float min_dist = max_dist1;
    if (max_dist2 < max_dist1)
      min_dist = max_dist2;

    return min_dist;
  }

  inline float
  getScaleFactorOnPlane (const pcl::PointCloud<pcl::PointXYZ> & pcd_i, const pcl::PointCloud<pcl::PointXYZ> & pcd_j)
  {

    float max_dist1, max_dist2;
    max_dist1 = max_dist2 = -1;

    for (size_t i = 0; i < pcd_i.points.size (); i++)
    {
      float dist = pcd_i.points[i].x * pcd_i.points[i].x + pcd_i.points[i].z * pcd_i.points[i].z;
      if (dist > max_dist1) {
        max_dist1 = dist;
      }
    }

    for (size_t i = 0; i < pcd_j.points.size (); i++)
    {
      float dist = pcd_j.points[i].x * pcd_j.points[i].x + pcd_j.points[i].z * pcd_j.points[i].z;
      if (dist > max_dist2) {
        max_dist2 = dist;
      }
    }

    float max_dist = max_dist1;
    if (max_dist2 > max_dist)
      max_dist = max_dist2;

    max_dist = sqrt(max_dist); //square distance, should not be bigger than one meter!

    return (0.75 / max_dist); //scale factor
  }

  inline double
  timing (struct timeval & start, struct timeval & end)
  {
    double ms = (end.tv_sec - start.tv_sec) * 1000.0;
    double micros = (end.tv_usec - start.tv_usec) / 1000.0;
    ms += micros;
    return ms;
  }

}

#endif
