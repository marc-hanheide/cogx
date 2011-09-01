/*
 * roll_hist_estimator.h
 *
 *  Created on: Jun 22, 2011
 *      Author: aitor
 */

#ifndef NORMAL_ESTIMATOR_H_
#define NORMAL_ESTIMATOR_H_

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <estimators/estimator.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class NormalEstimator : public Estimator<PointInT, PointOutT, FeatureT>
{

protected:
  /** \brief Wether the input cloud needs to be voxel grided, default=true **/
  bool apply_voxel_grid_;

  /** \brief Wether if the input cloud needs to be filtered, default=true **/
  bool apply_radius_removal_;

  /** \brief Voxel grid size, default=5mm **/
  double leaf_size_;

  /** \brief Computes the mesh resolution and uses it as the leaf_size_, default=false **/
  bool compute_mesh_resolution_;

  /** \brief leaf_size_ factor for support stuff, default=3 **/
  float leaf_size_factor_;

public:

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;
  typedef typename pcl::KdTree<PointOutT>::Ptr KdTreePtr;

  virtual void
               estimate (
                         PointInTPtr & in,
                         PointOutTPtr & out,
                         std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                         std::vector<Eigen::Vector3f> & centroids)=0;

  bool
  computeNormals (PointInTPtr input, PointOutTPtr cloud_normals);

  void
  setComputeMeshResolution(bool apply) {
    compute_mesh_resolution_ = apply;
  }

  void
  setLeafSizeFactor(float factor) {
    leaf_size_factor_ = factor;
  }

};

#endif /* NORMAL_ESTIMATOR_H_ */
