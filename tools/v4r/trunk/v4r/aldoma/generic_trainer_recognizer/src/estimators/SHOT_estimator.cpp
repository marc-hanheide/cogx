#include "estimators/SHOT_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"
//#include "pcl/point_types.hpp"

template<typename PointInT, typename PointOutT, typename FeatureT>
void
SHOTEstimator<PointInT, PointOutT, FeatureT>::computeSHOT (
    PointOutTPtr cloud_normals,
                                                           std::vector<pcl::PointCloud<FeatureT>,
                                                               Eigen::aligned_allocator<pcl::PointCloud<
                                                               FeatureT> > > & vfh_signatures,
                                                           std::vector<Eigen::Vector3f> & centroids_dominant_orientations)
{
  //compute CVFH
  typedef typename pcl::SHOTEstimation<PointOutT, PointOutT, pcl::SHOT> SHOTEstimation;
  pcl::PointCloud < FeatureT > signatures;

  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid  ( *cloud_normals, centroid4f);
  Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);
  centroids_dominant_orientations.push_back (centroid3f);

  typename pcl::PointCloud < PointOutT >::Ptr input_cloud(new pcl::PointCloud < PointOutT > ());
  input_cloud->points.resize(1);
  input_cloud->width = input_cloud->height = 1;
  input_cloud->points[0].getVector4fMap() = centroid4f;

  SHOTEstimation shot_estimate;
  typename pcl::KdTree<PointOutT>::Ptr cvfh_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > ();
  shot_estimate.setSearchMethod (cvfh_tree);
  shot_estimate.setInputCloud (input_cloud);
  shot_estimate.setInputNormals (input_cloud);
  shot_estimate.compute (signatures);

  std::cout << "compute CVFH:" << signatures.size() << std::endl;

  /*for (size_t i = 0; i < signatures.points.size (); i++)
  {
    pcl::PointCloud < FeatureT > signature;
    signature.points.resize (1);
    vfh_signature.width = vfh_signature.height = 1;
    for (int d = 0; d < 308; ++d)
      vfh_signature.points[0].histogram[d] = cvfh_signatures.points[i].histogram[d];

    vfh_signatures.push_back (vfh_signature);
  }

  cvfh.getCentroidClusters (centroids_dominant_orientations);*/
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
SHOTEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<
    pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
    Eigen::Vector3f> & centroids)
{

  computeNormals (in, out);
  computeSHOT (out, signatures, centroids);

  std::cout << "Number of descriptors:" << signatures.size() << std::endl;
}

template class SHOTEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::SHOT> ;
template class SHOTEstimator<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::SHOT> ;
