#include <pcl/registration/icp.h>
#include <pcl/segmentation/segment_differences.h>
#include "alignment/internal_pcl_functions.h"

class FitnessScore {
private:
  int FS_ID_;
  bool partial_views_;

  //TODO: This should not change the point clouds...
  double numberOfInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_i, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_j) {

    float max_dist = internal_pcl::getMinMaxDist (*pcd_i, *pcd_j);
    //voxel grid data here and use the voxel grided versions
    float voxel_size = max_dist / 50;

    if (partial_views_)
      voxel_size = 0.005;

    pcl::VoxelGrid < pcl::PointXYZ > grid_;
    grid_.setInputCloud (pcd_i);
    grid_.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid_.filter (*pcd_i);

    pcl::VoxelGrid < pcl::PointXYZ > grid_2;
    grid_2.setInputCloud (pcd_j);
    grid_2.setLeafSize (voxel_size, voxel_size, voxel_size);
    grid_2.filter (*pcd_j);

    pcl::SegmentDifferences < pcl::PointXYZ > seg;
    typedef pcl::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
    KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > (true);
    //seg.setDistanceThreshold (max_dist * 0.05 * max_dist * 0.05); //TODO: Check this parameters!!
    seg.setDistanceThreshold (voxel_size); //TODO: Check this parameters!!
    seg.setSearchMethod (normals_tree);
    seg.setInputCloud (pcd_i);
    seg.setTargetCloud (pcd_j);

    int min_points = (std::min)(pcd_i->points.size(),pcd_j->points.size());

    pcl::PointCloud < pcl::PointXYZ > difference;
    seg.segment (difference);

    double inliers_inversed = 1.0 / double((pcd_i->points.size () - difference.points.size ())); // / (double)transformed_i->points.size ();
    //std::cout << "factor inliers:" << inliers_inversed << "min_points:" << min_points << " diff:" << difference.points.size ()  << std::endl;
    return inliers_inversed;
  }

  double ICPFitnessScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_i, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_j) {
    pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > reg;
    reg.setInputCloud (pcd_i);
    reg.setInputTarget (pcd_j);
    //reg.setMaxCorrespondenceDistance (max_dist * 0.05);
    return reg.getFitnessScore();
  }

public:

  FitnessScore(int FS_ID=0) {
    FS_ID_ = FS_ID;
    partial_views_ = false;
  }

  ~FitnessScore() {

  }

  void setPartialViewAlignment(bool partial_views) {
    partial_views_ = partial_views;
  }

  double computeFitnessScore(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_i, pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_j) {
    switch(FS_ID_) {
      case 1:
        //#ICP fitness score
        return ICPFitnessScore(pcd_i,pcd_j);
      case 0:
      default:
        //#number of inliers
        return numberOfInliers(pcd_i,pcd_j);
    }
  }
};
