#include "recognizers/roll_recognizer.h"
#include <cv.h>
#include "persistence_utils.h"
#include <omp.h>
#include <cmath>

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
void
RollRecognizer<Distance, PointInT, PointOutT, FeatureT>::getRoll (std::string file, cv::Mat & roll)
{
  if (use_cache_)
  {
    std::map<std::string, cv::Mat>::iterator it = roll_hists_cache_.find (file);

    if (it != roll_hists_cache_.end ())
    {
      roll = it->second;
      return;
    }
  }

  std::stringstream dir;
  dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
  std::stringstream dir_with_descr;
  dir_with_descr << dir.str() << "/" << D_NAME_;

  //PCL_WARN ("Not found in cache, fall back getRoll() -> reading from file...\n");
  PersistenceUtils::readCvMat1DFromFile (dir_with_descr.str(), file, roll);
}

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
void
RollRecognizer<Distance, PointInT, PointOutT, FeatureT>::computeFinalPose (Eigen::Vector3f & c_input,
                                                                           Eigen::Vector3f & c_match,
                                                                           Eigen::Matrix4f & homMatrixPose,
                                                                           Eigen::Matrix4f & final_pose, int i)
{
  typedef RollEstimator<PointInT, PointOutT, FeatureT> RollEstimatorT;
  //RollEstimatorT * roll_estimator = dynamic_cast<RollEstimatorT *> (estimator_);

  boost::shared_ptr<RollEstimatorT> roll_estimator =
     boost::dynamic_pointer_cast<RollEstimatorT>(estimator_);

  Eigen::Vector4f cInput (c_input[0], c_input[1], c_input[2], 0);
  Eigen::Vector4f cMatch (c_match[0], c_match[1], c_match[2], 0);
  //roll
  Eigen::Affine3f rollToRot;
  roll_estimator->computeRollTransform (cInput, cMatch, roll_rotation_angles_[i], rollToRot);
  Eigen::Matrix4f rollHomMatrix = Eigen::Matrix4f ();
  rollHomMatrix.setIdentity (4, 4);
  rollHomMatrix = rollToRot.matrix ();

  //translation matrix (2) - center the view and the input
  //Eigen::Vector4f centroidView (c_match[0], c_match[1], c_match[2], 0);
  Eigen::Matrix4f translation2;
  translation2.setIdentity (4, 4);
  Eigen::Vector3f centr;
  centr = rollToRot * c_match;

  translation2 (0, 3) = -centr[0] + c_input[0];
  translation2 (1, 3) = -centr[1] + c_input[1];
  translation2 (2, 3) = -centr[2] + c_input[2];
  Eigen::Matrix4f resultHom = Eigen::Matrix4f ();

  //append transformations
  Eigen::Matrix4f icp_trans = icp_transformations_[i];
  resultHom = icp_trans * translation2 * (rollHomMatrix * homMatrixPose); //result
  final_pose = resultHom;
}

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
void
RollRecognizer<Distance, PointInT, PointOutT, FeatureT>::specificPostProcess (
                                                                              std::vector<index_score> & indices_scores,
                                                                              PointOutTPtr cloud_out,
                                                                              std::vector<pcl::PointCloud<FeatureT>,
                                                                                  Eigen::aligned_allocator<
                                                                                      pcl::PointCloud<FeatureT> > > & signatures,
                                                                              std::vector<Eigen::Vector3f> & centroids)
{
  PointOutTPtr cloud_out_downsampled (new pcl::PointCloud<PointOutT>);
  pcl::VoxelGrid < PointOutT > grid_;
  grid_.setInputCloud (cloud_out);
  grid_.setLeafSize (leaf_size_downsampled_, leaf_size_downsampled_, leaf_size_downsampled_);
  grid_.filter (*cloud_out_downsampled);

  typedef RollEstimator<PointInT, PointOutT, FeatureT> RollEstimatorT;
  std::vector<cv::Mat> roll_histograms_freq_domain;

  boost::shared_ptr<RollEstimatorT> roll_estimator =
     boost::dynamic_pointer_cast<RollEstimatorT>(estimator_);
  roll_estimator->getRollHistograms (roll_histograms_freq_domain);

  /*RollEstimatorT * roll_estimator = dynamic_cast<RollEstimatorT *> (estimator_);
  roll_estimator->getRollHistograms (roll_histograms_freq_domain);*/

  std::vector<index_score> indices_scores_post_processing;
  std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > icp_transformations;
  std::vector<int> roll_rotation_angles;
  double fscore;

  if(use_omp_) {

    std::cout << "Parallel execution" << omp_get_max_threads() << " " << omp_get_num_procs() << std::endl;
    int length = (std::min)((int)indices_scores.size(),nmodels_);
    icp_transformations.resize (length);
    roll_rotation_angles.resize (length);
    indices_scores_post_processing.resize (length);

#pragma omp parallel for num_threads(omp_get_num_procs())
    for (size_t i = 0; (i < length); ++i)
    {

      std::string id_match = models_.at (indices_scores[i].idx_models_).first;

      //recover centroid, roll histogram and point cloud from matching view...
      Eigen::Vector3f c_match;
      getCentroid (id_match, c_match);
      //roll histogram
      cv::Mat roll_match;
      getRoll (id_match, roll_match);
      //point cloud
      PointOutTPtr cloud_match (new pcl::PointCloud<PointOutT>);
      getView (id_match, cloud_match);

      PointOutTPtr cloud_match_downsampled (new pcl::PointCloud<PointOutT>);

      pcl::VoxelGrid < PointOutT > grid2;
      grid2.setInputCloud (cloud_match);
      grid2.setLeafSize (leaf_size_downsampled_, leaf_size_downsampled_, leaf_size_downsampled_);
      grid2.filter (*cloud_match_downsampled);

      Eigen::Vector3f c_input = centroids[indices_scores[i].idx_input_];
      cv::Mat roll_input = roll_histograms_freq_domain[indices_scores[i].idx_input_];

      //compute roll angle
      Eigen::Vector4f cmatch4f (c_match[0], c_match[1], c_match[2], 0);
      Eigen::Vector4f cinput4f (c_input[0], c_input[1], c_input[2], 0);

      int angle;

      angle = roll_estimator->computeRollAngle (roll_input, roll_match, cmatch4f, cinput4f,
                                                    *cloud_match_downsampled, *cloud_out_downsampled);
      //transform match so we can compare the views...
      PointOutTPtr match_trans (new pcl::PointCloud<PointOutT>);
      Eigen::Affine3f final_trans;

      roll_estimator->computeRollTransform (cinput4f, cmatch4f, angle, final_trans);
      //pcl::transformPointCloudWithNormals (*cloud_match_downsampled, *match_trans, final_trans);
      pcl::transformPointCloud (*cloud_match_downsampled, *match_trans, final_trans);

      //translate result point cloud so centroids match...
      Eigen::Vector3f centr;
      centr = final_trans * c_match;

      Eigen::Vector4f diff = Eigen::Vector4f::Zero ();
      diff[0] = -cinput4f[0] + centr[0];
      diff[1] = -cinput4f[1] + centr[1];
      diff[2] = -cinput4f[2] + centr[2];

      PointOutTPtr match_trans_demeaned (new pcl::PointCloud<PointOutT> ());
      pcl::demeanPointCloud (*match_trans, diff, *match_trans_demeaned);

      pcl::IterativeClosestPoint < PointOutT, PointOutT > reg;
      reg.setInputCloud (match_trans_demeaned); //model aligned after CVFH
      reg.setInputTarget (cloud_out_downsampled); //cluster in the camera coordinate system

      //Perform ICP and save the final transformation
      reg.setMaximumIterations (icp_iterations_);
      reg.setMaxCorrespondenceDistance (max_icp_correspondence_dist_);
      PointOutTPtr output_ (new pcl::PointCloud<PointOutT> ());

      reg.align (*output_);

      Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();

      //fitness score
      pcl::SegmentDifferences < PointOutT > seg;
      typedef typename pcl::KdTree<PointOutT>::Ptr KdTreePtr;
      KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > (true);

      seg.setDistanceThreshold (leaf_size_downsampled_ * leaf_size_downsampled_ * 0.85);
      seg.setSearchMethod (normals_tree);
      seg.setInputCloud (cloud_out_downsampled);
      seg.setTargetCloud (output_);

      PointOutTPtr difference (new pcl::PointCloud<PointOutT> ());
      seg.segment (*difference);
      fscore = difference->points.size ();

      double fscore_percent = (cloud_out_downsampled->points.size () - difference->points.size ())
          / (double)cloud_out_downsampled->points.size ();

      index_score is;
      is.idx_models_ = i;
      is.idx_input_ = i;
      is.score_ = fscore;

      icp_transformations[i] = icp_trans;
      roll_rotation_angles[i] = angle;
      indices_scores_post_processing[i] = is;

    }

#pragma omp barrier //wait for all threads to finish

  } else {
    for (size_t i = 0; (i < nmodels_) && (i < indices_scores.size()); ++i)
    {
      std::string id_match = models_.at (indices_scores[i].idx_models_).first;

      //recover centroid, roll histogram and point cloud from matching view...
      Eigen::Vector3f c_match;
      getCentroid (id_match, c_match);
      //roll histogram
      cv::Mat roll_match;
      getRoll (id_match, roll_match);
      //point cloud
      PointOutTPtr cloud_match (new pcl::PointCloud<PointOutT>);
      getView (id_match, cloud_match);

      PointOutTPtr cloud_match_downsampled (new pcl::PointCloud<PointOutT>);
      grid_.setInputCloud (cloud_match);
      grid_.setLeafSize (leaf_size_downsampled_, leaf_size_downsampled_, leaf_size_downsampled_);
      grid_.filter (*cloud_match_downsampled);

      Eigen::Vector3f c_input = centroids[indices_scores[i].idx_input_];
      cv::Mat roll_input = roll_histograms_freq_domain[indices_scores[i].idx_input_];

      //compute roll angle
      Eigen::Vector4f cmatch4f (c_match[0], c_match[1], c_match[2], 0);
      Eigen::Vector4f cinput4f (c_input[0], c_input[1], c_input[2], 0);

      int angle;

      angle = roll_estimator->computeRollAngle (roll_input, roll_match, cmatch4f, cinput4f,
                                                    *cloud_match_downsampled, *cloud_out_downsampled);
      //transform match so we can compare the views...
      PointOutTPtr match_trans (new pcl::PointCloud<PointOutT>);
      Eigen::Affine3f final_trans;

      roll_estimator->computeRollTransform (cinput4f, cmatch4f, angle, final_trans);
      //pcl::transformPointCloudWithNormals (*cloud_match_downsampled, *match_trans, final_trans);
      pcl::transformPointCloud (*cloud_match_downsampled, *match_trans, final_trans);

      //translate result point cloud so centroids match...
      Eigen::Vector3f centr;
      centr = final_trans * c_match;

      Eigen::Vector4f diff = Eigen::Vector4f::Zero ();
      diff[0] = -cinput4f[0] + centr[0];
      diff[1] = -cinput4f[1] + centr[1];
      diff[2] = -cinput4f[2] + centr[2];

      PointOutTPtr match_trans_demeaned (new pcl::PointCloud<PointOutT> ());
      pcl::demeanPointCloud (*match_trans, diff, *match_trans_demeaned);

      pcl::IterativeClosestPoint < PointOutT, PointOutT > reg;
      reg.setInputCloud (match_trans_demeaned); //model aligned after CVFH
      reg.setInputTarget (cloud_out_downsampled); //cluster in the camera coordinate system

      //Perform ICP and save the final transformation
      reg.setMaximumIterations (icp_iterations_);
      reg.setMaxCorrespondenceDistance (max_icp_correspondence_dist_);
      PointOutTPtr output_ (new pcl::PointCloud<PointOutT> ());

      reg.align (*output_);

      Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
      icp_transformations.push_back (icp_trans);
      roll_rotation_angles.push_back (angle);

      //fitness score
      pcl::SegmentDifferences < PointOutT > seg;
      typedef typename pcl::KdTree<PointOutT>::Ptr KdTreePtr;
      KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > (true);

      seg.setDistanceThreshold (leaf_size_downsampled_ * leaf_size_downsampled_ * 0.85);
      seg.setSearchMethod (normals_tree);
      seg.setInputCloud (cloud_out_downsampled);
      seg.setTargetCloud (output_);

      PointOutTPtr difference (new pcl::PointCloud<PointOutT> ());
      seg.segment (*difference);
      fscore = difference->points.size ();

      double fscore_percent = (cloud_out_downsampled->points.size () - difference->points.size ())
          / (double)cloud_out_downsampled->points.size ();

      index_score is;
      is.idx_models_ = i;
      is.idx_input_ = i;
      is.score_ = fscore;

      indices_scores_post_processing.push_back (is);

      if (fscore_percent > 0.98)
      {
        //fill the rest of
        for (size_t k = (i + 1); k < nmodels_; k++)
        {
          index_score is;
          is.idx_models_ = k;
          is.idx_input_ = k;
          is.score_ = std::numeric_limits<double>::max ();
          indices_scores_post_processing.push_back (is);

          Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
          icp_transformations.push_back (Eigen::Matrix4f::Identity ());
          roll_rotation_angles.push_back (0);
        }

        break;
      }
    }

  }

  //we will sort indices_scores_post_processing and put it back to indices_scores...
  std::sort (indices_scores_post_processing.begin (), indices_scores_post_processing.end (), sortIndexScores);

  icp_transformations_.clear ();
  roll_rotation_angles_.clear ();

  std::vector<index_score> final_indices;
  for (size_t i = 0; i < indices_scores_post_processing .size (); ++i)
  {
    index_score is;
    is.idx_models_ = indices_scores[indices_scores_post_processing[i].idx_models_].idx_models_;
    is.idx_input_ = indices_scores[indices_scores_post_processing[i].idx_input_].idx_input_;
    is.score_ = indices_scores_post_processing[i].score_;
    final_indices.push_back (is);

    //save icp transformations and roll in the right order
    icp_transformations_.push_back (icp_transformations[indices_scores_post_processing[i].idx_models_]);
    roll_rotation_angles_.push_back (roll_rotation_angles[indices_scores_post_processing[i].idx_models_]);
  }

  indices_scores = final_indices;
}

template class RollRecognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<640> > ;
template class RollRecognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class RollRecognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ;
template class RollRecognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class RollRecognizer<flann::ChiSquareDistance, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class RollRecognizer<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class RollRecognizer<Metrics::HistIntersectionUnionDistance, pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ;

