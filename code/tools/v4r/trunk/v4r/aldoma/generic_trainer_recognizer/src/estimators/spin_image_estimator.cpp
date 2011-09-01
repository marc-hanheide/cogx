/*
 * spin_image_estimator.cpp
 *
 *  Created on: Aug 27, 2011
 *      Author: aitor
 */

#include "estimators/spin_image_estimator.h"
#include "pcl/features/spin_image.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  SpinImageEstimator<PointInT, PointOutT, FeatureT>::estimate (
                                                               PointInTPtr & in,
                                                               PointOutTPtr & out,
                                                               std::vector<pcl::PointCloud<FeatureT>,
                                                                   Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                                                               std::vector<Eigen::Vector3f> & centroids)
  {
    typename pcl::KdTree<PointOutT>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > ();
    typedef pcl::Histogram<153> SpinImage;
    typedef typename pcl::SpinImageEstimation<PointOutT, PointOutT, SpinImage> SpinImageEstimation;

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid (*in, centroid4f);
    Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);
    centroids.push_back (centroid3f);

    SpinImageEstimation spin_est;

    PointOutT spin_point;

    Eigen::Vector4f normal (centroid4f);
    normal.normalize ();

    spin_point.getVector4fMap () = centroid4f;
    spin_point.getNormalVector4fMap () = -normal; //direction to camera (assuming vp (0,0,0)

    spin_est.setRotationAxis (spin_point);

    PointOutTPtr surface (new pcl::PointCloud<PointOutT> ());
    pcl::copyPointCloud (*in, *out);

    PointOutTPtr input(new pcl::PointCloud<PointOutT> ());
    input->points.resize(1);
    input->points[0] = spin_point;

    spin_est.setInputWithNormals (input, input);
    spin_est.setSearchSurfaceWithNormals (out, out);
    spin_est.setSearchMethod (tree);

    pcl::PointXYZ zero;
    zero.getVector4fMap () = Eigen::Vector4f::Zero ();
    float radius = std::numeric_limits<float>::min ();
    for (size_t i = 0; i < in->points.size (); i++)
    {
      radius = (std::max) (radius, pcl::euclideanDistance (zero, in->points[i]));
    }

    spin_est.setRadiusSearch (radius);

    pcl::PointCloud < FeatureT > signature;
    signature.points.resize (1);
    signature.width = 1;
    signature.height = 1;

    spin_est.compute (signature);

    signatures.resize (1);

    /*for (int i = 0; i < 153; i++)
     signature.points[0].histogram[i] = descriptors.points[0].histogram[i];*/

    signatures[0] = (signature);

  }

template class SpinImageEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<153> > ;
