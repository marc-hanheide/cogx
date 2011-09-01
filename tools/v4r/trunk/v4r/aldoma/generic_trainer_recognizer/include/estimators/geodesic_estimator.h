/*
 * cvfh_utils.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef GEODESIC_ESTIMATOR_H_
#define GEODESIC_ESTIMATOR_H_

#include <estimators/roll_hist_estimator.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
  class GeodesicEstimator : public RollEstimator<PointInT, PointOutT, FeatureT>
  {

  public:

    typedef RollEstimator<PointInT, PointOutT, FeatureT> RollEstimatorT;
    typedef typename RollEstimatorT::PointInTPtr PointInTPtr;
    typedef typename RollEstimatorT::PointOutTPtr PointOutTPtr;
    typedef typename RollEstimatorT::PointOutTPtr FeatureTPtr;

    using RollEstimatorT::apply_radius_removal_;
    using RollEstimatorT::apply_voxel_grid_;
    using RollEstimatorT::nr_bins_;
    using RollEstimatorT::leaf_size_;
    using RollEstimatorT::distance_diff_;
    using RollEstimatorT::roll_histograms_freq_domain_;

    GeodesicEstimator (bool apply_voxel_grid, bool apply_radius_removal, double leaf_size, int nr_bins)
    {
      apply_voxel_grid_ = apply_voxel_grid;
      apply_radius_removal_ = apply_radius_removal;
      leaf_size_ = leaf_size;
      nr_bins_ = nr_bins;
      distance_diff_ = 0.005;
    }

    void
    estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
        pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids);

  };

#endif /* GEODESIC_ESTIMATOR_H_ */
