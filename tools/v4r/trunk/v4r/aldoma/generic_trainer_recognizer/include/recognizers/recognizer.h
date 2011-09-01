/*
 * recognizer.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef RECOGNIZER_H_
#define RECOGNIZER_H_

#include <flann/flann.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/common/time.h>
#include "estimators/estimator.h"
#include "metrics/metrics.h"

typedef std::pair<std::string, std::vector<float> > flann_model;

struct index_score
{
  int idx_models_;
  int idx_input_;
  double score_;
};

inline bool
sortIndexScores (const index_score& d1, const index_score& d2)
{
  return d1.score_ < d2.score_;
}

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  class Recognizer
  {

    typedef Distance<float> DistT;
    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;
    typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

    typedef Estimator<PointInT, PointOutT, FeatureT> EstimatorT;

  public:

    Recognizer (bool use_cache = true);
    ~Recognizer ();

    /** \brief Read histograms from the filesystem and initializes flann.
     * \param training_dir Directory in the filesystem where the models have been trained.
     */
    void
    initialize (std::string training_dir);

    /** \brief Recognize a cluster
     *
     */

    bool
    recognize (PointInTPtr & cluster, std::vector<std::string>& model_ids, std::vector<Eigen::Matrix4f,
        Eigen::aligned_allocator<Eigen::Matrix4f> > & poses, std::vector<float>& confidences);

    void setComputePose(bool apply) {
      compute_pose_ = apply;
    }

    void
    setNumberOfModels (int n)
    {
      nmodels_ = n;
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
    performNNSearch (pcl::PointCloud<pcl::VFHSignature308> & signature, int k, flann::Matrix<int> &indices,
                     flann::Matrix<float> &distances)
    {
      float* hist = signature.points[0].histogram;
      std::vector<float> std_hist (hist, hist + 308);
      flann_model histogram ("", std_hist);
      nearestKSearch (index_, histogram, k, indices, distances);
    }

    //Attribute setters
    void
    setTesselationLevel (int level)
    {
      tesselated_sphere_level_ = level;
    }

    std::string
    getModel(int idx) {
      return models_.at (idx).first;
    }

    void setModelsDirectory(std::string dir) {
      MODELS_DIR_ = dir;
    }
    void setDescriptorName(std::string dname)
    {
      D_NAME_ = dname;
    }

    std::vector<PointOutTPtr> matching_views_;

  protected:
    flann::Matrix<float> data_;
    int knn_;
    flann::Index<DistT> * index_;
    std::vector<flann_model> models_;
    int nmodels_;
    bool use_cache_;
    bool compute_pose_;
    //EstimatorT * estimator_;
    boost::shared_ptr<EstimatorT> estimator_;
    int tesselated_sphere_level_;

    std::map<std::string, Eigen::Matrix4f, std::less<std::string>, Eigen::aligned_allocator<std::pair<
        const std::string, Eigen::Matrix4f> > > poses_cache_;

    std::map<std::string, Eigen::Vector3f> centroids_cache_;
    std::map<std::string, PointOutTPtr> views_cache_;

    /** \brief Directory where the data will be persistent **/
    std::string TRAINING_DIR_;

    std::string MODELS_DIR_;

    /** \brief ending of the descriptor when saved to hdd **/
    std::string D_NAME_;

    /** \brief Voxel grid size, default=5mm **/
    double leaf_size_;

    void
    nearestKSearch (flann::Index<DistT> * index, const flann_model &model, int k, flann::Matrix<int> &indices,
                    flann::Matrix<float> &distances);

    bool
    loadFileList (std::vector<flann_model> &models, const std::string &filename);

    void
    saveFileList (const std::vector<flann_model> &models, const std::string &filename);

    void
    getCentroid (std::string file, Eigen::Vector3f & centroid);
    void
    getPose (std::string file, Eigen::Matrix4f & pose_matrix);
    void
    getView (std::string file, PointOutTPtr & view);

    virtual void
    computeFinalPose (Eigen::Vector3f & cInput, Eigen::Vector3f & c_match, Eigen::Matrix4f & homMatrixPose,
                      Eigen::Matrix4f & final_pose, int i)
    {
      Eigen::Matrix4f translation2;
      translation2.setIdentity (4, 4);

      translation2 (0, 3) = -c_match[0] + cInput[0];
      translation2 (1, 3) = -c_match[1] + cInput[1];
      translation2 (2, 3) = -c_match[2] + cInput[2];

      Eigen::Matrix4f resultHom = Eigen::Matrix4f ();
      //append transformations
      resultHom = translation2 * homMatrixPose; //result
      final_pose = resultHom;
    }

    virtual void
    computeFinalPoseView (Eigen::Vector3f & cInput, Eigen::Vector3f & c_match, Eigen::Matrix4f & final_pose, int i)
    {
      Eigen::Matrix4f translation2;
      translation2.setIdentity (4, 4);

      translation2 (0, 3) = -c_match[0] + cInput[0];
      translation2 (1, 3) = -c_match[1] + cInput[1];
      translation2 (2, 3) = -c_match[2] + cInput[2];
      final_pose = translation2;
    }

  protected:
    virtual void
    specificPostProcess (std::vector<index_score> & indices_scores, PointOutTPtr cloud_out, std::vector<
        pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > & signatures, std::vector<
        Eigen::Vector3f> & centroids)
    {
      std::cout << "Do nothing here" << std::endl;
    }

  };

#endif /* RECOGNIZER_H_ */
