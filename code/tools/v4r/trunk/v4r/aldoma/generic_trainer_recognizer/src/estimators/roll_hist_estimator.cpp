/*
 * roll_hist_estimator.cpp
 *
 *  Created on: Jun 22, 2011
 *      Author: aitor
 */

#include "estimators/roll_hist_estimator.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/common/time.h"

typedef struct
{
  bool
  operator() (std::pair<double, size_t> const& a, std::pair<double, size_t> const& b)
  {
    return a.first > b.first;
  }
} peaks_ordering;

template<typename PointInT, typename PointOutT, typename FeatureT>
bool
RollEstimator<PointInT, PointOutT, FeatureT>::computeNormals (PointInTPtr input,
                                                              PointOutTPtr cloud_normals)
{
  PointInTPtr grid (new pcl::PointCloud<PointInT> ());

  if (apply_voxel_grid_)
  {
    pcl::VoxelGrid<PointInT> grid_;
    grid_.setInputCloud (input);
    grid_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    grid_.filter (*grid);
  }
  else
  {
    std::cout << "Do not apply voxel gridding" << std::endl;
    copyPointCloud (*input, *grid);
  }

  if (grid->points.size () == 0)
  {
    std::cout << "Cloud has no points after voxel grid, wont be able to compute normals!"
        << std::endl;
    return false;
  }

  PointInTPtr gridFiltered (new pcl::PointCloud<PointInT> ());
  if (apply_radius_removal_)
  {
    //in synthetic views the render grazes some parts of the objects that are barely seen from that VP
    //thus creating a very sparse set of points that causes the normals to be very noisy!!
    //remove these points
    pcl::RadiusOutlierRemoval<PointInT> sor;
    sor.setInputCloud (grid);
    sor.setRadiusSearch (leaf_size_ * 3);
    sor.setMinNeighborsInRadius (10);
    sor.filter (*gridFiltered);
  }
  else
  {
    copyPointCloud (*grid, *gridFiltered);
    std::cout << "Do not apply radius removal" << std::endl;
  }

  /*pcl::visualization::PCLVisualizer scene("lolo");
  scene.addPointCloud<PointInT>(gridFiltered);
  scene.spin();*/

  if (gridFiltered->points.size () == 0)
  {
    std::cout << "Cloud has no points after voxel grid and filtering, wont be able to compute normals or descriptor"
        << std::endl;
    return false;
  }

  typedef typename pcl::NormalEstimation<PointInT, PointOutT> NormalEstimator;
  NormalEstimator n3d;

  typedef typename pcl::KdTree<PointInT>::Ptr KdTreeInPtr;
  KdTreeInPtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointInT> > (false);
  normals_tree->setInputCloud (gridFiltered);
  n3d.setRadiusSearch (leaf_size_ * 3);
  n3d.setSearchMethod (normals_tree);
  n3d.setInputCloud (gridFiltered);
  n3d.compute (*cloud_normals);

  std::cout << "cloud_normals size:" << cloud_normals->points.size() << std::endl;

  for (size_t i = 0; i < cloud_normals->points.size (); ++i)
  {
    cloud_normals->points[i].getVector4fMap() = gridFiltered->points[i].getVector4fMap();
  }

  //check nans...
  int j = 0;
  for (size_t i = 0; i < cloud_normals->points.size (); ++i)
  {
    if (!pcl_isfinite (cloud_normals->points[i].normal_x) || !pcl_isfinite (cloud_normals->points[i].normal_y)
        || !pcl_isfinite (cloud_normals->points[i].normal_z))
      continue;

    cloud_normals->points[j] = cloud_normals->points[i];
    j++;
  }

  cloud_normals->points.resize (j);
  cloud_normals->width = j;
  cloud_normals->height = 1;
  return true;
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
RollEstimator<PointInT, PointOutT, FeatureT>::computeRollHistogram (PointOutTPtr input,
                                                                    Eigen::Vector3f & centroid,
                                                                    cv::Mat & frequency_domain)
{
  Eigen::Vector3f plane_normal;
  plane_normal[0] = -centroid[0];
  plane_normal[1] = -centroid[1];
  plane_normal[2] = -centroid[2];
  Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
  plane_normal.normalize ();
  Eigen::Vector3f axis = plane_normal.cross (z_vector);
  double rotation = -asin (axis.norm ());
  axis.normalize ();

  unsigned int nbins = nr_bins_;
  int bin_angle = 360 / nbins;

  Eigen::Affine3f transformPC (Eigen::AngleAxisf (rotation, axis));

  pcl::PointCloud < PointOutT > grid;
  pcl::transformPointCloudWithNormals (*input, grid, transformPC);

  cv::Mat hist = cv::Mat_<float>::zeros (nbins, 1);

  double sum_w = 0, w = 0;
  int bin = 0;
  for (size_t i = 0; i < grid.points.size (); ++i)
  {
    bin = (int)(((atan2 (grid.points[i].normal_y, grid.points[i].normal_x) + M_PI) * 180 / M_PI) / bin_angle) % nbins;
    w = sqrt (grid.points[i].normal_y * grid.points[i].normal_y + grid.points[i].normal_x * grid.points[i].normal_x);
    sum_w += w;
    hist.at<float> (bin) += w;
  }

  for (size_t i = 0; i < nbins; ++i)
  {
    hist.at<float> (i) /= sum_w;
  }

  cv::dft (hist, frequency_domain);
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
RollEstimator<PointInT, PointOutT, FeatureT>::computeTransformToZAxes (Eigen::Vector4f & centroid,
                                                                       Eigen::Affine3f & transform)
{
  Eigen::Vector3f plane_normal;
  plane_normal[0] = -centroid[0];
  plane_normal[1] = -centroid[1];
  plane_normal[2] = -centroid[2];
  Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
  plane_normal.normalize ();
  Eigen::Vector3f axis = plane_normal.cross (z_vector);
  double rotation = -asin (axis.norm ());
  axis.normalize ();
  transform = Eigen::Affine3f (Eigen::AngleAxisf (rotation, axis));
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
RollEstimator<PointInT, PointOutT, FeatureT>::computeRollTransform (Eigen::Vector4f & centroidInput,
                                                                    Eigen::Vector4f & centroidResult,
                                                                    double roll_angle, Eigen::Affine3f & final_trans)
{
  Eigen::Affine3f transformInputToZ;
  computeTransformToZAxes (centroidInput, transformInputToZ);

  transformInputToZ = transformInputToZ.inverse ();

  Eigen::Affine3f transformRoll (Eigen::AngleAxisf (-(roll_angle * M_PI / 180), Eigen::Vector3f::UnitZ ()));

  Eigen::Affine3f transformDBResultToZ;
  computeTransformToZAxes (centroidResult, transformDBResultToZ);

  final_trans = transformInputToZ * transformRoll * transformDBResultToZ;
}

template<typename PointInT, typename PointOutT, typename FeatureT>
int
RollEstimator<PointInT, PointOutT, FeatureT>::computeRollAngle (const cv::Mat & hist_fft_const,
                                                                const cv::Mat & hist_fft_input,
                                                                Eigen::Vector4f centroid_view,
                                                                Eigen::Vector4f centroid_input,
                                                                const pcl::PointCloud<PointOutT> & view,
                                                                const pcl::PointCloud<PointOutT> & input)
{

  cv::Mat hist_fft (hist_fft_const);

  for (size_t i = 2; i < (size_t)nr_bins_; i += 2)
  {
    hist_fft.at<float> (i) = -hist_fft.at<float> (i);
  }

  int nr_bins_after_padding = 180;
  int peak_distance = 4;
  cv::Mat multAB = cv::Mat_<float>::zeros (nr_bins_after_padding, 1);

  float a, b, c, d;

  multAB.at<float> (0) = hist_fft.at<float> (0) * hist_fft_input.at<float> (0);

  size_t cutoff = 5;
  cutoff = nr_bins_ - 1;
  for (size_t i = 1; i < cutoff; i += 2)
  {
    a = hist_fft.at<float> (i);
    b = hist_fft.at<float> (i + 1);
    c = hist_fft_input.at<float> (i);
    d = hist_fft_input.at<float> (i + 1);
    multAB.at<float> (i) = a * c - b * d;
    multAB.at<float> (i + 1) = b * c + a * d;
    float tmp = sqrt (multAB.at<float> (i) * multAB.at<float> (i) + multAB.at<float> (i + 1) * multAB.at<float> (i
        + 1));

    multAB.at<float> (i) /= tmp;
    multAB.at<float> (i + 1) /= tmp;
  }

  multAB.at<float> (nr_bins_ - 1) = hist_fft.at<float> (nr_bins_ - 1) * hist_fft_input.at<float> (nr_bins_ - 1);
  cv::Mat invAB;
  cv::idft (multAB, invAB, cv::DFT_SCALE);

  std::vector<std::pair<double, int> > scored_peaks (nr_bins_after_padding);

  for (size_t i = 0; i < (size_t)nr_bins_after_padding; i++)
  {
    scored_peaks[i] = std::make_pair<double, size_t> (invAB.at<float> (i), (int)i);
  }

  std::sort (scored_peaks.begin (), scored_peaks.end (), peaks_ordering ());
  std::vector<int> peaks;
  std::vector<float> peaks_values;

  // we look at the upper 20% quantile
  for (size_t i = 0; i < (size_t)(0.2 * nr_bins_after_padding); i++)
  {
    if (scored_peaks[i].first >= scored_peaks[0].first * 0)
    {
      bool insert = true;

      for (size_t j = 0; j < peaks.size (); j++)
      { //check inserted peaks, first pick always inserted
        if (fabs (peaks[j] - scored_peaks[i].second) <= peak_distance || fabs (peaks[j] - (scored_peaks[i].second
            - nr_bins_after_padding)) <= peak_distance)
        {
          insert = false;
          break;
        }
      }

      if (insert)
      {
        peaks.push_back (scored_peaks[i].second);
        peaks_values.push_back (scored_peaks[i].first);
      }
    }
  }

  if (peaks.size () == 1)
  {
    return peaks[0] * (360 / nr_bins_after_padding);
  }

  int max_idx = 0;
  int NUM_PEAKS_CHECKED = 0;
  PointOutTPtr input_ptr (new pcl::PointCloud<PointOutT> (input));
  double max_score = std::numeric_limits<double>::min ();

  {
    for (size_t i = 0; i < peaks.size (); i++)
    {
      double current_angle = peaks[i] * (360 / nr_bins_after_padding);
      Eigen::Affine3f final_trans;
      computeRollTransform (centroid_input, centroid_view, current_angle, final_trans);

      PointOutTPtr gridTransformed (new pcl::PointCloud<PointOutT> ());
      pcl::transformPointCloud (view, *gridTransformed, final_trans);

      //translate result point cloud so centroids match...
      Eigen::Vector3f centr (centroid_view[0], centroid_view[1], centroid_view[2]);
      centr = final_trans * centr;
      Eigen::Vector4f diff = Eigen::Vector4f::Zero ();
      diff[0] = -centroid_input[0] + centr[0];
      diff[1] = -centroid_input[1] + centr[1];
      diff[2] = -centroid_input[2] + centr[2];

      PointOutTPtr cloud_xyz_demean (new pcl::PointCloud<PointOutT> ());
      pcl::demeanPointCloud (*gridTransformed, diff, *cloud_xyz_demean);

      distance_diff_ = 0.0125;
      double fscore;

      pcl::SegmentDifferences < PointOutT > seg;
      KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > (true);
      seg.setDistanceThreshold (distance_diff_ * distance_diff_);
      seg.setSearchMethod (normals_tree);
      seg.setInputCloud (input_ptr);
      seg.setTargetCloud (cloud_xyz_demean);

      pcl::PointCloud < PointOutT > difference;
      seg.segment (difference);

      fscore = input.points.size () - difference.points.size (); //number of inliers

      if (fscore > max_score)
      {
        max_score = fscore;
        max_idx = peaks[i];
      }

      NUM_PEAKS_CHECKED++;
    }

  } //end time scope...

  return max_idx * (360 / nr_bins_after_padding);
}

template<typename PointInT, typename PointOutT, typename FeatureT>
void
RollEstimator<PointInT, PointOutT, FeatureT>::estimate (PointInTPtr & in,
               PointOutTPtr & out,
               std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
               std::vector<Eigen::Vector3f> & centroids) {

  //estimate the feature using the underlying estimator
  estimator_->estimate(in, out, signatures, centroids);

  std::cout << "points in after computing:" << in->points.size() << std::endl;

  //estimate the roll histogram computing normals if necessary
  computeNormals (in, out);

  std::vector<cv::Mat> roll_histograms_freq_domain;
  roll_histograms_freq_domain.resize (signatures.size ());

  std::cout << "Number of descriptors:" << signatures.size() << std::endl;

  for (size_t idx = 0; idx < signatures.size (); idx++)
  {
    computeRollHistogram (out, centroids[idx], roll_histograms_freq_domain[idx]);
  }

  //save roll_histograms_freq_domain_
  roll_histograms_freq_domain_ = roll_histograms_freq_domain;

}
template class RollEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<640> > ;
template class RollEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class RollEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ;
template class RollEstimator<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class RollEstimator<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ;
