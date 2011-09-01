#include "model.h"

template<template<typename,typename> class ViewTT, typename PointT, typename FeatureT>
int
Model<ViewTT, PointT,FeatureT>::addView (pcl::PointCloud<PointT> & pointcloud, std::vector<pcl::PointCloud<FeatureT>,
    Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
         std::vector<Eigen::Vector3f> & centroids, Eigen::Matrix4f & pose, float entropy )
{
  //copy the stuff into view
  boost::shared_ptr<ViewT> v (new View<PointT, FeatureT> ());
  pcl::copyPointCloud (pointcloud, v->pointcloud_);
  v->pose_ = Eigen::Matrix4f (pose);

  for (size_t i = 0; i < signatures.size (); i++)
    v->signatures_.push_back (signatures[i]);

  for (size_t i = 0; i < centroids.size (); i++)
    v->centroids_.push_back (centroids[i]);

  v->entropy_ = entropy;

  views_.push_back (v);

  return views_.size() - 1;
}

template class Model<View, pcl::PointNormal, pcl::Histogram<153> >;
template class Model<View, pcl::PointNormal, pcl::Histogram<640> >;
template class Model<View, pcl::PointNormal, pcl::Histogram<416> >;
template class Model<View, pcl::PointNormal, pcl::Histogram<480> >;
template class Model<View, pcl::PointNormal, pcl::VFHSignature308>;
template class Model<View, pcl::PointXYZ, pcl::VFHSignature308>;
template class Model<View, pcl::PointXYZRGBNormal, pcl::VFHSignature308>;

