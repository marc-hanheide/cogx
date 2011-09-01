/*
 * roll_hist_estimator.h
 *
 *  Created on: Jun 22, 2011
 *      Author: aitor
 */

#ifndef ROLL_HIST_ESTIMATOR_H_
#define ROLL_HIST_ESTIMATOR_H_

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <estimators/estimator.h>

#include "cv.h"

template<typename PointInT, typename PointOutT, typename FeatureT>
  class RollEstimator : public Estimator<PointInT, PointOutT, FeatureT>
  {

  protected:
    /** \brief Wether the input cloud needs to be voxel grided, default=true **/
    bool apply_voxel_grid_;

    /** \brief Wether if the input cloud needs to be filtered, default=true **/
    bool apply_radius_removal_;

    /** \brief Voxel grid size, default=5mm **/
    double leaf_size_;

    /** \brief number of bins for the roll histogram, default = 90 giving an angular resolution of 4° **/
    int nr_bins_;

    /** \brief distance for segment differences **/
    double distance_diff_;

    /** \brief Roll histograms **/
    std::vector<cv::Mat> roll_histograms_freq_domain_;

    typedef Estimator<PointInT, PointOutT, FeatureT> EstimatorT;
    //EstimatorT * estimator_;
    boost::shared_ptr<EstimatorT> estimator_;


  public:

    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;
    typedef typename pcl::KdTree<PointOutT>::Ptr KdTreePtr;

    RollEstimator(bool apply_voxel_grid, bool apply_radius_removal, double leaf_size, int nr_bins) {
     apply_voxel_grid_ = apply_voxel_grid;
     apply_radius_removal_ = apply_radius_removal;
     leaf_size_ = leaf_size;
     nr_bins_ = nr_bins;
     distance_diff_ = 0.005;
    }

    /*void
    setEstimator (EstimatorT * estimator)
    {
      estimator_ = estimator;
    }*/

    void
    setEstimator(boost::shared_ptr<EstimatorT> & estimator) {
      estimator_ = estimator;
    }

    void
    getRollHistograms (std::vector<cv::Mat> & roll_histograms_freq_domain)
    {
      roll_histograms_freq_domain = roll_histograms_freq_domain_;
    }

    void estimate (PointInTPtr & in,
                   PointOutTPtr & out,
                   std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures,
                   std::vector<Eigen::Vector3f> & centroids);

    bool
    computeNormals (PointInTPtr input, PointOutTPtr cloud_normals);

    void
    computeTransformToZAxes (Eigen::Vector4f & centroid, Eigen::Affine3f & transform);

    void
    computeRollTransform (Eigen::Vector4f & centroidInput, Eigen::Vector4f & centroidResult, double roll_angle,
                          Eigen::Affine3f & final_trans);

    void
    computeRollHistogram (PointOutTPtr input, Eigen::Vector3f & centroid, cv::Mat & frequency_domain);

    int
    computeRollAngle (const cv::Mat & hist_fft_const, const cv::Mat & hist_fft_input, Eigen::Vector4f centroid_view,
                      Eigen::Vector4f centroid_input, const pcl::PointCloud<PointOutT> & view,
                      const pcl::PointCloud<PointOutT> & input);

  };

#endif /* ROLL_HIST_ESTIMATOR_H_ */
