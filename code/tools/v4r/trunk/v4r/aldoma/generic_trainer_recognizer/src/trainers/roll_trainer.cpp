/*
 * cvfh_trainer.cpp
 *
 *  Created on: Jun 20, 2011
 *      Author: aitor
 */

#include <trainers/roll_trainer.h>
#include <estimators/roll_hist_estimator.h>

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  RollTrainer<PointInT, PointOutT, FeatureT>::specificPersistence (int i, std::string dir,
                                                                   boost::shared_ptr<View<PointOutT, FeatureT> > view)
  {
    typedef ViewRoll<PointOutT, FeatureT> ViewRollT;
    ViewRollT * cvfh_view = static_cast<ViewRollT *> (view.get ());
    cvfh_view->makePersistent (i, dir, D_NAME_);
  }

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  RollTrainer<PointInT, PointOutT, FeatureT>::specificTraining (boost::shared_ptr<EstimatorT> & estimator,
                                                                Model<View, PointOutT, FeatureT> & model, int pos)
  {
    //get roll histogram from the estimatorT which is a CVFHEstimator...
    std::vector<cv::Mat> roll_histograms_freq_domain;
    typedef RollEstimator<PointInT, PointOutT, FeatureT> RollEstimatorT;
    //RollEstimatorT * roll_estimator = dynamic_cast<RollEstimatorT *> (estimator);

    boost::shared_ptr<RollEstimatorT> roll_estimator =
       boost::dynamic_pointer_cast<RollEstimatorT>(estimator);

    roll_estimator->getRollHistograms (roll_histograms_freq_domain);
    //get generic view from model using pos
    boost::shared_ptr<View<PointOutT, FeatureT> > view = model.getView (pos);
    //create viewRoll
    boost::shared_ptr<ViewRoll<PointOutT, FeatureT> > view_roll (new ViewRoll<PointOutT, FeatureT> (view));

    for (size_t j = 0; j < view->signatures_.size (); j++)
    {
      view_roll->roll_histograms_.push_back (roll_histograms_freq_domain[j]);
    }

    //replace view in model at same position
    model.replaceView (view_roll, pos);
  }

template<typename PointInT, typename PointOutT, typename FeatureT>
  void
  RollTrainer<PointInT, PointOutT, FeatureT>::removeDuplicateViews (boost::shared_ptr<EstimatorT> & estimator, Model<View, PointOutT, FeatureT> & model,
                                                                    Model<View, PointOutT, FeatureT> & filtered)
  {
    std::vector<bool> unique (model.size (), true);
    typedef View<PointOutT, FeatureT> ViewT;
    typedef typename std::vector<boost::shared_ptr<ViewT> >::iterator view_roll_iterator;
    view_roll_iterator it_views_1;

    typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutPtr;
    typedef RollEstimator<PointInT, PointOutT, FeatureT> RollEstimatorT;
    //RollEstimatorT * roll_estimator = dynamic_cast<RollEstimatorT *> (estimator);
    boost::shared_ptr<RollEstimatorT> roll_estimator =
       boost::dynamic_pointer_cast<RollEstimatorT>(estimator);

    int i, j;
    i = j = 0;
    for (it_views_1 = model.begin (); it_views_1 != model.end (); ++it_views_1, ++i)
    {


      typedef ViewRoll<PointOutT, FeatureT> ViewRollT;
      ViewRollT * view1 = static_cast<ViewRollT *> ((*it_views_1).get ());

      Eigen::Vector4f c14f (view1->centroids_[0][0],
                            view1->centroids_[0][1],
                            view1->centroids_[0][2], 0);
      Eigen::Vector3f plane_normal;
      plane_normal[0] = -c14f[0];
      plane_normal[1] = -c14f[1];
      plane_normal[2] = -c14f[2];
      Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
      plane_normal.normalize ();
      Eigen::Vector3f axis = plane_normal.cross (z_vector);
      double rotation = -asin (axis.norm ());
      axis.normalize ();
      Eigen::Affine3f transformRM (Eigen::AngleAxisf (rotation, axis));
      PointOutPtr cloud_normals_1 (new pcl::PointCloud<PointOutT> (view1->pointcloud_));

      std::cout << "Checking view:" << i << std::endl;

      view_roll_iterator it_views_2;

      for (it_views_2 = (it_views_1+1); it_views_2 != model.end (); ++it_views_2, ++j)
      {

        ViewRollT * view2 = static_cast<ViewRollT *> ((*it_views_2).get ());

        //compare *it_views_1 with *it_views_2
        Eigen::Vector4f c24f (view2->centroids_[0][0],
                              view2->centroids_[0][1],
                              view2->centroids_[0][2], 0);

        int angle = roll_estimator->computeRollAngle (view1->roll_histograms_[0],
                                                      view2->roll_histograms_[0], c14f, c24f,
                                                      view1->pointcloud_, view2->pointcloud_);

        //align views and check if they are the same!
        //std::cout << "Computed angle:" << angle << std::endl;
        plane_normal[0] = -c24f[0];
        plane_normal[1] = -c24f[1];
        plane_normal[2] = -c24f[2];
        plane_normal.normalize ();
        z_vector = Eigen::Vector3f::UnitZ ();
        axis = plane_normal.cross (z_vector);
        rotation = -asin (axis.norm ());
        axis.normalize ();
        Eigen::Affine3f transformRS (Eigen::AngleAxisf (rotation, axis));
        transformRS = transformRS.inverse (); //invert transformation
        //roll transformation
        Eigen::Affine3f transformRoll (Eigen::AngleAxisf ((angle * M_PI / 180), Eigen::Vector3f::UnitZ ()));
        Eigen::Affine3f final_trans = transformRS * transformRoll * transformRM;

        pcl::PointCloud<PointOutT> cloud_xyz;
        PointOutPtr gridTransformed (new pcl::PointCloud<PointOutT> ());
        pcl::transformPointCloud (view2->pointcloud_, *gridTransformed, final_trans);
        cloud_xyz = *gridTransformed;

        // Demean the cloud
        PointOutPtr cloud_xyz_demean (new pcl::PointCloud<PointOutT> ());
        Eigen::Vector4f diff = Eigen::Vector4f::Zero ();

        Eigen::Vector3f centr (c24f[0], c24f[1], c24f[2]);
        centr = final_trans * centr;

        diff[0] = -c14f[0] + centr[0];
        diff[1] = -c14f[1] + centr[1];
        diff[2] = -c14f[2] + centr[2];

        pcl::demeanPointCloud (cloud_xyz, diff, *cloud_xyz_demean);

        //we can compare cloud_xyz_demean with cloud_normals_1
        //segment difference
        pcl::SegmentDifferences<PointOutT> seg;
        typedef typename pcl::KdTree<PointOutT>::Ptr KdTreePtr;
        KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<PointOutT> > (true);
        seg.setDistanceThreshold (0.01 * 0.01); //1cm
        seg.setSearchMethod (normals_tree);
        seg.setInputCloud (cloud_normals_1);
        seg.setTargetCloud (cloud_xyz_demean);

        pcl::PointCloud<PointOutT> difference;
        seg.segment (difference);

        //std::cout << "Fitness score:" << fscore << std::endl;
        double fscore = (double)(difference.points.size ()) / (double)(view1->pointcloud_.points.size ());
        //std::cout << "% of remaining points:" << fscore << " i:" << i << " j:" << j << " size diff:" << difference.points.size() << std::endl;
        if (fscore < 0.025)
        { //if less than 2.5% of the points have their nearest neighbor outside threshold, then view is not unique
          unique[i] = false;
          break; //we have found at least one view after i that is the same
        }
      }
    }

    i = 0;
    for (it_views_1 = model.begin (); it_views_1 != model.end (); ++it_views_1, ++i)
    {
      if (unique.at (i)) //if its unique, add it to the filtered model

      {
        std::cout << "Adding view to the model, is unique" << std::endl;
        //add to model filtered!
        filtered.addView (*it_views_1);
      }
    }

    std::cout << "Number of views in filtered model:" << filtered.size () << std::endl;
  }

template class RollTrainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<640> > ;
template class RollTrainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class RollTrainer<pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ;
template class RollTrainer<pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class RollTrainer<pcl::PointXYZRGB, pcl::PointXYZRGBNormal, pcl::VFHSignature308> ;
