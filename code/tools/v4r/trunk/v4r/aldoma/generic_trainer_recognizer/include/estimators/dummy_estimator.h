/*
 * dummy_estimator.h
 *
 *  Created on: Aug 29, 2011
 *      Author: aitor
 */

#ifndef DUMMY_ESTIMATOR_H_
#define DUMMY_ESTIMATOR_H_

#include <estimators/estimator.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
class DummyEstimator : public Estimator<PointInT, PointOutT, FeatureT>
{

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

public:

  DummyEstimator ()
  {

  }

  void
       estimate (
                 PointInTPtr & in,
                 PointOutTPtr & out,
                 std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                 std::vector<Eigen::Vector3f> & centroids) {

    std::cout << "I am dummy estimator" << std::endl;

  }

};


#endif /* DUMMY_ESTIMATOR_H_ */
