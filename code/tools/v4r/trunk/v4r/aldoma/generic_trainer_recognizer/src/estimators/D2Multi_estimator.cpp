#include "estimators/D2Multi_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"

typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;

inline void
D2_multi (pcl::PointCloud<pcl::PointXYZ> &pc, int h32[], int h64[], int h128[], int h256[])
{
  unsigned int sample_size = 20000;
  srand ((unsigned)time (0));
  int maxindex = pc.points.size ();
  int index1, index2;
  std::vector<float> dv;
  dv.reserve (sample_size);
  float maxd = 0;
  for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
  {
    index1 = rand () % maxindex;
    index2 = rand () % maxindex;
    dv[nn_idx] = pcl::euclideanDistance (pc.points[index1], pc.points[index2]);
    if (dv[nn_idx] > maxd)
      maxd = dv[nn_idx];
  }

  for (size_t nn_idx = 0; nn_idx < sample_size; ++nn_idx)
  {
    h32[(int)round (dv[nn_idx] / maxd * (32 - 1))]++;
    h64[(int)round (dv[nn_idx] / maxd * (64 - 1))]++;
    h128[(int)round (dv[nn_idx] / maxd * (128 - 1))]++;
    h256[(int)round (dv[nn_idx] / maxd * (256 - 1))]++;
  }

}

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  D2MultiScaleEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in, PointOutTPtr & out, std::vector<
      pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
      Eigen::Vector3f> & centroids)
  {

    std::cout << "D2MultiScaleEstimator::estimate()" << std::endl;
    computeNormals (in, out);

    //..compute
    pcl::PointCloud < FeatureT > signature;

    int h32[32] = {0};
    int h64[64] = {0};
    int h128[128] = {0};
    int h256[256] = {0};

    D2_multi (*in, h32, h64, h128, h256);

    signature.points.resize(1);
    signature.width = 1;
    signature.height = 1;

    int k=0;

    for (int i = 0; i < 32; i++,k++)
      signature.points[0].histogram[k] = h32[i];
    for (int i = 0; i < 64; i++,k++)
      signature.points[0].histogram[k] = (h64[i]);
    for (int i = 0; i < 128; i++,k++)
      signature.points[0].histogram[k] = (h128[i]);
    for (int i = 0; i < 256; i++,k++)
      signature.points[0].histogram[k] = (h256[i]);

    signatures.push_back (signature);

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid (*in, centroid4f);
    Eigen::Vector3f centroid3f (centroid4f[0], centroid4f[1], centroid4f[2]);
    centroids.push_back (centroid3f);

    //and roll histograms for each signature
    std::vector<cv::Mat> roll_histograms_freq_domain;
    roll_histograms_freq_domain.resize (signatures.size ());

    for (size_t idx = 0; idx < signatures.size (); idx++)
    {
      computeRollHistogram (out, centroids[idx], roll_histograms_freq_domain[idx]);
    }

    //save roll_histograms_freq_domain_
    roll_histograms_freq_domain_ = roll_histograms_freq_domain;
  }

template class D2MultiScaleEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ;
