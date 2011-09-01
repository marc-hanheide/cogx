/*
 * cvfh_trainer.h
 *
 *  Created on: Jun 20, 2011
 *      Author: aitor
 */

#include "model.h"
#include "trainer.h"

#ifndef ROLL_TRAINER_H_
#define ROLL_TRAINER_H_

template<typename PointT, typename FeatureT>
  class ViewRoll : public View<PointT, FeatureT>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef View<PointT, FeatureT> ViewT;

    using ViewT::pointcloud_;
    using ViewT::pose_;
    using ViewT::centroids_;
    using ViewT::signatures_;
    using ViewT::entropy_;

    ViewRoll (boost::shared_ptr<View<PointT, FeatureT> > v_o)
    {
      pcl::copyPointCloud (v_o->pointcloud_, pointcloud_);
      pose_ = Eigen::Matrix4f (v_o->pose_);
      entropy_ = v_o->entropy_;

      for (size_t i = 0; i < v_o->signatures_.size (); i++)
        signatures_.push_back (v_o->signatures_[i]);

      for (size_t i = 0; i < v_o->centroids_.size (); i++)
        centroids_.push_back (v_o->centroids_[i]);
    }

    void
    makePersistent (int i, std::string path, std::string descr_name)
    {

      //check that the view is correct
      if(pointcloud_.points.size() == 0)
        return; //do not save the view...

      //std::cout << "Called makePersitent in RollView" << std::endl;
      //write roll histograms
      for (size_t j = 0; j < signatures_.size (); j++)
      {
        stringstream path_roll;
        path_roll << path << "/" << descr_name << "/roll_histogram_" << i << "_" << j << ".txt";
        PersistenceUtils::writeCvMat1DToFile (path_roll.str (), roll_histograms_[j]);
      }
    }

    std::vector<cv::Mat> roll_histograms_; //move this to a subclass??
  };

template<typename PointInT, typename PointOutT, typename FeatureT>
  class RollTrainer : public Trainer<PointInT, PointOutT, FeatureT>
  {

  typedef Trainer<PointInT, PointOutT, FeatureT> TrainerT;
  using TrainerT::remove_duplicate_views_;
  using TrainerT::D_NAME_;

  public:
    RollTrainer (std::string files_dir, std::string train_dir) :
      Trainer<PointInT, PointOutT, FeatureT> (files_dir, train_dir)
    {

    }
    ;

    typedef Estimator<PointInT, PointOutT, FeatureT> EstimatorT;

    void
    setRemoveDuplicateViews (bool value)
    {
      remove_duplicate_views_ = value;
    }

  protected:
    void
    specificTraining (boost::shared_ptr<EstimatorT> & estimator, Model<View, PointOutT, FeatureT> & model, int pos);

    void specificPersistence(int i, std::string dir, boost::shared_ptr<View<PointOutT, FeatureT> > view);

    void removeDuplicateViews (boost::shared_ptr<EstimatorT> & estimator, Model<View, PointOutT, FeatureT> & model, Model<View, PointOutT, FeatureT> & filtered);

  };

#endif /* ROLL_TRAINER_H_ */
