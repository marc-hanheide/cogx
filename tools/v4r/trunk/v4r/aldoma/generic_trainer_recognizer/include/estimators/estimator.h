/*
 * estimator.h
 *
 *  Created on: Jun 20, 2011
 *      Author: aitor
 */

#include <pcl/common/common.h>
#include "generic_trainer_recognizer/specific_point_types.h"

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

template<typename PointInT, typename PointOutT, typename FeatureT>
  class Estimator
  {

    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

  public:
    virtual void
    estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
        pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids)=0;
  };

#endif /* ESTIMATOR_H_ */
