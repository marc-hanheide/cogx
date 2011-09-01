#include "estimators/cvfh_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"

template<typename PointInT, typename PointOutT, typename FeatureT>
void
CVFHEstimator<PointInT, PointOutT, FeatureT>::computeCVFH (
    PointOutTPtr cloud_normals,
                                                           std::vector<pcl::PointCloud<FeatureT>,
                                                               Eigen::aligned_allocator<pcl::PointCloud<
                                                               FeatureT> > > & vfh_signatures,
                                                           std::vector<Eigen::Vector3f> & centroids_dominant_orientations)
{
  //compute CVFH
  typedef typename pcl::CVFHEstimation<PointOutT, PointOutT, pcl::VFHSignature308> CVFHEstimation;
  pcl::PointCloud < FeatureT > cvfh_signatures;

  CVFHEstimation cvfh;
  cvfh.setClusterTolerance(leaf_size_ * leaf_size_factor_);
  cvfh.setLeafSize(leaf_size_);
  cvfh.setEPSAngleThreshold(eps_angle_threshold_);
  cvfh.setCurvatureThreshold(curvature_threshold_);
  cvfh.setNormalizeBins(normalize_bins_);

  typename pcl::KdTree<PointOutT>::Ptr cvfh_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > ();
  cvfh.setSearchMethod (cvfh_tree);
  cvfh.setInputCloud (cloud_normals);
  cvfh.setInputNormals (cloud_normals);
  cvfh.compute (cvfh_signatures);


  //pcl::io::savePCDFile("clusters_colored.pcd", *cvfh.clusters_colored_);
  std::cout << "thresholds:" << eps_angle_threshold_ << " " << curvature_threshold_ << "normalize_bins:" << normalize_bins_ << std::endl;
  //std::cout << "compute CVFH:" << cvfh_signatures.size() << std::endl;

  for (size_t i = 0; i < cvfh_signatures.points.size (); i++)
  {
    pcl::PointCloud < FeatureT > vfh_signature;
    vfh_signature.points.resize (1);
    vfh_signature.width = vfh_signature.height = 1;
    for (int d = 0; d < 308; ++d)
      vfh_signature.points[0].histogram[d] = cvfh_signatures.points[i].histogram[d];

    vfh_signatures.push_back (vfh_signature);
  }

  cvfh.getCentroidClusters (centroids_dominant_orientations);
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
CVFHEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<
    pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
    Eigen::Vector3f> & centroids)
{

  computeNormals (in, out);
  computeCVFH (out, signatures, centroids);

  //std::cout << "Number of descriptors:" << signatures.size() << std::endl;
}

template class CVFHEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class CVFHEstimator<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ;
