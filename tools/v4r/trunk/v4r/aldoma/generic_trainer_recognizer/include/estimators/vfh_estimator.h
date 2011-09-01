/*
 * cvfh_utils.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef VFH_ESTIMATOR_H_
#define VFH_ESTIMATOR_H_

#include <estimators/normal_estimator.h>
#include <pcl/features/vfh.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
  class VFHEstimator : public NormalEstimator<PointInT, PointOutT, FeatureT>
  {

  public:

    typedef NormalEstimator<PointInT, PointOutT, FeatureT> NormalEstimatorT;
    typedef typename NormalEstimatorT::PointInTPtr PointInTPtr;
    typedef typename NormalEstimatorT::PointOutTPtr PointOutTPtr;

    using NormalEstimatorT::apply_radius_removal_;
    using NormalEstimatorT::apply_voxel_grid_;
    using NormalEstimatorT::leaf_size_;
    using NormalEstimatorT::compute_mesh_resolution_;
    using NormalEstimatorT::leaf_size_factor_;

    VFHEstimator (bool apply_voxel_grid, bool apply_radius_removal, double leaf_size)
    {
      apply_voxel_grid_ = apply_voxel_grid;
      apply_radius_removal_ = apply_radius_removal;
      leaf_size_ = leaf_size;
      compute_mesh_resolution_ = false;
      leaf_size_factor_ = 3;
    }

    void
    estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
        pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids);

  };

#endif /* VFH_ESTIMATOR_H_ */
