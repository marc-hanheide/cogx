/*
 * cvfh_model.h
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#ifndef MODEL_H_
#define MODEL_H_

#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include "pcl/point_types.h"
#include <cv.h>
#include <pcl/io/pcd_io.h>
#include <persistence_utils.h>

using namespace std;

//Represents a trained model and all the needed information
//views and poses and for each of view,  descriptors, roll histograms
//and the centroids used to compute each descriptor

template<typename PointT, typename FeatureT>
  class View
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<PointT> pointcloud_;
    Eigen::Matrix4f pose_;
    std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures_;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > centroids_;
    float entropy_;

    void makePersistent(int i, std::string path, std::string descr_name) {

      //std::cout << "Called makePersitent in generic View" << std::endl;

      //check that the view is correct
      if(pointcloud_.points.size() == 0)
        return; //do not save the view...

      //save views as view_i.pcd and so on...
      stringstream path_view;
      path_view << path << "/" << descr_name << "/view_" << i << ".pcd";
      pcl::io::savePCDFileBinary (path_view.str (), pointcloud_);

      stringstream path_pose;
      path_pose << path << "/" << descr_name << "/pose_" << i << ".txt";
      PersistenceUtils::writeMatrixToFile (path_pose.str (), pose_);

      stringstream path_entropy;
      path_entropy << path << "/" << descr_name << "/entropy_" << i << ".txt";
      PersistenceUtils::writeFloatToFile (path_entropy.str (), entropy_);

      //std::cout << "signatures.size()" << signatures_.size() << std::endl;
      //for each CVFH, save descriptor, roll histogram and centroids
      for (size_t j = 0; j < signatures_.size (); j++)
      {

        if(centroids_.size() > j) {
          //write centroid
          stringstream path_centroid;
          path_centroid << path << "/" << descr_name << "/centroid_" << i << "_" << j << ".txt";
          Eigen::Vector3f centroid (centroids_[j][0], centroids_[j][1], centroids_[j][2]);

          PersistenceUtils::writeCentroidToFile (path_centroid.str (), centroid);
        }

        //write descriptor
        stringstream path_cvfh;
        path_cvfh << path << "/" << descr_name << "/descriptor_" << i << "_" << j << ".pcd";
        pcl::io::savePCDFileBinary (path_cvfh.str (), signatures_[j]);
      }
    }
  };

template<template<typename , typename > class ViewTT, typename PointT, typename FeatureT>
  //template<typename PointT, typename FeatureT>
  class Model
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef ViewTT<PointT, FeatureT> ViewT;
    typedef typename std::vector<boost::shared_ptr<ViewT> >::iterator view_iterator;

    Model (std::string id)
    {
      id_ = id;
    }

    int
    addView (pcl::PointCloud<PointT> & pointcloud, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
        pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids, Eigen::Matrix4f & pose, float entropy);

    void
    addView (boost::shared_ptr<ViewT> view)
    {
      views_.push_back (view);
    }

    int
    size ()
    {
      return views_.size ();
    }

    view_iterator
    begin ()
    {
      return views_.begin ();
    }

    view_iterator
    end ()
    {
      return views_.end ();
    }

    boost::shared_ptr<ViewT> getView(int pos) {
      return views_[pos];
    }

    void replaceView(boost::shared_ptr<ViewT> v, int pos) {
      views_[pos] = v;
    }

    std::string
    getId ()
    {
      return id_;
    }

    static std::string
    getIdFromFilename (std::string filename)
    {
      return filename.substr (0, filename.length () - 4); //basically remove the extension from the relative path
    }

  private:
    std::vector<boost::shared_ptr<ViewT> > views_;
    std::string id_;

  };

#endif /* MODEL_H_ */

