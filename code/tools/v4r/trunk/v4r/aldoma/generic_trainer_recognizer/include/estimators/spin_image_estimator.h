/*
 * spin_image_estimator.h
 *
 *  Created on: Aug 27, 2011
 *      Author: aitor
 */

#ifndef SPIN_IMAGE_ESTIMATOR_H_
#define SPIN_IMAGE_ESTIMATOR_H_

#include <estimators/estimator.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class SpinImageEstimator : public Estimator<PointInT, PointOutT, FeatureT>
{

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

public:
  SpinImageEstimator ()
  {

  }

  void
       estimate (
                 PointInTPtr & in,
                 PointOutTPtr & out,
                 std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                 std::vector<Eigen::Vector3f> & centroids);

};


#endif /* SPIN_IMAGE_ESTIMATOR_H_ */
