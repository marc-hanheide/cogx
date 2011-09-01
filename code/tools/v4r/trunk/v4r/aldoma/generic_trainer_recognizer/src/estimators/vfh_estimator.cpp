#include "estimators/vfh_estimator.h"

typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;

template<typename PointInT, typename PointOutT, typename FeatureT>
void
VFHEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<
    pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
    Eigen::Vector3f> & centroids)
{

  std::cout << "VFHEstimator::estimate()" << std::endl;
  computeNormals (in, out);

  pcl::io::savePCDFile("vfh_cloud.pcd", *out);

  //..compute VFH
  typedef pcl::VFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::VFHSignature308> VFHEstimation;
  pcl::PointCloud < pcl::VFHSignature308 > vfh_signature;

  VFHEstimation vfh;
  pcl::KdTree<pcl::PointNormal>::Ptr vfh_tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointNormal> > ();
  vfh.setSearchMethod (vfh_tree);
  vfh.setInputCloud (out);
  vfh.setInputNormals (out);
  vfh.setNormalizeBins(true);
  vfh.compute (vfh_signature);

  signatures.push_back(vfh_signature);

  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid(*in,centroid4f);
  Eigen::Vector3f centroid3f(centroid4f[0],centroid4f[1],centroid4f[2]);
  centroids.push_back(centroid3f);
}

template class VFHEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
