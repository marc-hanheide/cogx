/*
 * cvfh_recognizer.h
 *
 *  Created on: Jun 21, 2011
 *      Author: aitor
 */

#ifndef ROLL_RECOGNIZER_H_
#define ROLL_RECOGNIZER_H_

#include "recognizers/recognizer.h"
#include "estimators/roll_hist_estimator.h"

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  class RollRecognizer : public Recognizer<Distance, PointInT, PointOutT, FeatureT>
  {

  public:
    RollRecognizer (bool use_cache = true) :
      Recognizer<Distance, PointInT, PointOutT, FeatureT> (use_cache)
    {
      nr_bins_ = 90;
      icp_iterations_ = 10;
      max_icp_correspondence_dist_ = 0.03;
      leaf_size_downsampled_ = 0.01;
      use_omp_ = true;
    }

    void setICPIterations(int n_iter) {
      icp_iterations_ = n_iter;
    }

    typedef Distance<float> DistT;
    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

    typedef Estimator<PointInT, PointOutT, FeatureT> EstimatorT;
    typedef Recognizer<Distance, PointInT, PointOutT, FeatureT> RecognizerT;

    using RecognizerT::estimator_;
    using RecognizerT::TRAINING_DIR_;
    using RecognizerT::use_cache_;
    using RecognizerT::nmodels_;
    using RecognizerT::models_;

    using RecognizerT::getCentroid; //(std::string file, Eigen::Vector3f & centroid);
    using RecognizerT::tesselated_sphere_level_;
    using RecognizerT::D_NAME_;

    void
    specificPostProcess (std::vector<index_score> & indices_scores, PointOutTPtr cloud_out, std::vector<
        pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
        Eigen::Vector3f> & centroids);

    void
    computeFinalPose (Eigen::Vector3f & cInput, Eigen::Vector3f & c_match, Eigen::Matrix4f & homMatrixPose,
                      Eigen::Matrix4f & final_pose, int i);

    void
    computeFinalPoseView (Eigen::Vector3f & c_input, Eigen::Vector3f & c_match, Eigen::Matrix4f & final_pose, int i)
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
      resultHom = icp_trans * translation2 * rollHomMatrix; //result
      final_pose = resultHom;
    }

  protected:
    std::map<std::string, cv::Mat> roll_hists_cache_;
    void
    getRoll (std::string file, cv::Mat & roll);
    int nr_bins_;
    int icp_iterations_;
    double max_icp_correspondence_dist_;
    double leaf_size_downsampled_;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > icp_transformations_;
    std::vector<int> roll_rotation_angles_;
    bool use_omp_;
  };

#endif /* ROLL_RECOGNIZER_H_ */
