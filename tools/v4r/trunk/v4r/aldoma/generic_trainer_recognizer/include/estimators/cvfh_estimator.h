/*
 * cvfh_utils.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef CVFH_ESTIMATOR_H_
#define CVFH_ESTIMATOR_H_

#include <estimators/normal_estimator.h>
#include <pcl/features/cvfh.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class CVFHEstimator : public NormalEstimator <PointInT, PointOutT, FeatureT> {

public:
  typedef NormalEstimator<PointInT, PointOutT, FeatureT> NormalEstimatorT;
  typedef typename NormalEstimatorT::PointInTPtr PointInTPtr;
  typedef typename NormalEstimatorT::PointOutTPtr PointOutTPtr;

  using NormalEstimatorT::apply_radius_removal_;
  using NormalEstimatorT::apply_voxel_grid_;
  using NormalEstimatorT::leaf_size_;
  using NormalEstimatorT::compute_mesh_resolution_;
  using NormalEstimatorT::leaf_size_factor_;

  CVFHEstimator(bool apply_voxel_grid, bool apply_radius_removal, double leaf_size) {
    apply_voxel_grid_ = apply_voxel_grid;
    apply_radius_removal_ = apply_radius_removal;
    leaf_size_ = leaf_size;
    compute_mesh_resolution_ = false;
    leaf_size_factor_ = 3;
    eps_angle_threshold_ = 0.13;
    curvature_threshold_ = 0.03;
    normalize_bins_ = false;
  }

  void estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
          pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids);

  void
  computeCVFH (PointOutTPtr cloud_normals, std::vector<pcl::PointCloud<FeatureT>,
      Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & vfh_signatures, std::vector<Eigen::Vector3f> & centroids_dominant_orientations);

  void setEpsAngleThreshold(float t) {
    eps_angle_threshold_ = t;
  }

  void setCurvatureThreshold(float t) {
    curvature_threshold_ = t;
  }

  void setNormalizeBins(bool norm) {
    normalize_bins_ = norm;
  }

  private:
    float eps_angle_threshold_;
    float curvature_threshold_;
    bool normalize_bins_;
};

#endif /* CVFH_ESTIMATOR_H_ */
