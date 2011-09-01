/*
 * cvfh_utils.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef SHOT_ESTIMATOR_H_
#define SHOT_ESTIMATOR_H_

#include <estimators/normal_estimator.h>
#include <pcl/features/shot.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class SHOTEstimator : public NormalEstimator <PointInT, PointOutT, FeatureT> {

public:
  typedef NormalEstimator<PointInT, PointOutT, FeatureT> NormalEstimatorT;
  typedef typename NormalEstimatorT::PointInTPtr PointInTPtr;
  typedef typename NormalEstimatorT::PointOutTPtr PointOutTPtr;

  using NormalEstimatorT::apply_radius_removal_;
  using NormalEstimatorT::apply_voxel_grid_;
  using NormalEstimatorT::leaf_size_;

  SHOTEstimator(bool apply_voxel_grid, bool apply_radius_removal, double leaf_size) {
    apply_voxel_grid_ = apply_voxel_grid;
    apply_radius_removal_ = apply_radius_removal;
    leaf_size_ = leaf_size;
  }

  void estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
          pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids);

  void
  computeSHOT (PointOutTPtr cloud_normals, std::vector<pcl::PointCloud<FeatureT>,
      Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & vfh_signatures, std::vector<Eigen::Vector3f> & centroids_dominant_orientations);

};

#endif /* CVFH_ESTIMATOR_H_ */
