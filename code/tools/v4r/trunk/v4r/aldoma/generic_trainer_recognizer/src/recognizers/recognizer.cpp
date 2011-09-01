/*
 * recognizer.cpp
 *
 *  Created on: May 10, 2011
 *      Author: aitor
 */

#include "recognizers/recognizer.h"
#include "model.h"
#include "pcl/registration/transforms.h"

#include <boost/filesystem.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string.hpp>

#include <trainers/trainer.h>
#include <flann/io/hdf5.h>

namespace bf = boost::filesystem;

inline void
convertToFLANN (const std::vector<flann_model> &models, flann::Matrix<float> &data)
{
  data.rows = models.size ();
  data.cols = models[0].second.size (); // number of histogram bins
  data.data = (float*)malloc (data.rows * data.cols * sizeof(float));

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
    {
      data.data[i * data.cols + j] = models[i].second[j];
    }
}

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::getCentroid (std::string file, Eigen::Vector3f & centroid)
  {
    if (use_cache_)
    {
      std::map<std::string, Eigen::Vector3f>::iterator it = centroids_cache_.find (file);

      if (it != centroids_cache_.end ())
      {
        centroid = it->second;
        return;
      }

    }

    PCL_WARN ("Not found in cache, fall back");

    std::stringstream dir;
    dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
    std::stringstream dir_with_descr;
    dir_with_descr << dir.str () << "/" << D_NAME_;

    PersistenceUtils::getCentroidFromFile (dir_with_descr.str (), file, centroid);
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::getPose (std::string file, Eigen::Matrix4f & pose_matrix)
  {
    if (use_cache_)
    {
      std::map<std::string, Eigen::Matrix4f, std::less<std::string>, Eigen::aligned_allocator<Eigen::Matrix4f> >::iterator
                                                                                                                           it =
                                                                                                                               poses_cache_.find (
                                                                                                                                                  file);

      if (it != poses_cache_.end ())
      {
        pose_matrix = it->second;
        return;
      }

    }

    std::stringstream dir;
    dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
    std::stringstream dir_with_descr;
    dir_with_descr << dir.str () << "/" << D_NAME_;

    PCL_WARN ("Not found in cache, fall back");
    PersistenceUtils::readMatrixFromFile (dir_with_descr.str (), file, pose_matrix);
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::getView (std::string file, PointOutTPtr & view)
  {
    if (use_cache_)
    {
      typedef typename std::map<std::string, typename pcl::PointCloud<PointOutT>::Ptr>::iterator iteratorT;
      iteratorT it = views_cache_.find (file);

      if (it != views_cache_.end ())
      {
        view = it->second;
        return;
      }
    }

    PCL_WARN ("Not found in cache, fall back getView()");

    std::stringstream dir;
    dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
    std::stringstream dir_with_descr;
    dir_with_descr << dir.str () << "/" << D_NAME_;

    PersistenceUtils::getPointCloudFromFile<PointOutT> (dir_with_descr.str (), file, view);

  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::Recognizer (bool use_cache)
  {
    leaf_size_ = 0.005;
    use_cache_ = use_cache;
    compute_pose_ = true;
    tesselated_sphere_level_ = 1;
    //estimator_ = 0;
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::~Recognizer ()
  {
    /*if (estimator_ != 0)
    {
      //delete estimator_;
      //estimator_ = 0;
    }*/
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::initialize (std::string training_dir)
  {
    TRAINING_DIR_ = training_dir;
    bf::path trained_dir = training_dir;
    bf::directory_iterator end_itr, end_itr_in;

    std::stringstream flann_files_dir;
    flann_files_dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;

    bf::path training_data_h5_file_name = flann_files_dir.str () + "/" + D_NAME_ + "_training_data.h5";
    bf::path training_data_list_file_name = flann_files_dir.str () + "/" + D_NAME_ + "_training_data.list";
    bf::path index_filename = flann_files_dir.str () + "/" + D_NAME_ + "_kdtree.idx";

    //check if flann descriptor files exist... if yes, load them
    if (bf::exists (training_data_h5_file_name) && bf::exists (training_data_list_file_name)
        && bf::exists (index_filename))
    {
      //load flann files...
      loadFileList (models_, training_data_list_file_name.string ());
      flann::load_from_file (data_, training_data_h5_file_name.string (), "training_data");
      index_ = new flann::Index<DistT> (data_, flann::SavedIndexParams (index_filename.string ().c_str ()));
      index_->buildIndex ();

      PCL_INFO ("Using flann files\n");

    }
    else
    {
      FeatureT dummy; //to get the size of the underlying feature histogram

      //iterate over all models
      bf::path ply_files_dir = MODELS_DIR_;
      std::vector < std::string > files;
      std::string start = "";
      Trainer<PointInT, PointOutT, FeatureT>::getModelsInDirectory (ply_files_dir, start, files);

      int n_descriptors = 0;

      for (size_t fi = 0; fi < files.size (); fi++)
      {
        std::string id = Model<View, PointOutT, FeatureT>::getIdFromFilename (files[fi]);

        std::stringstream level_tes;
        level_tes << "level_" << tesselated_sphere_level_;

        bf::path inside = trained_dir / level_tes.str () / id / D_NAME_;
        for (bf::directory_iterator itr_in (inside); itr_in != end_itr_in; ++itr_in)
        {
          std::string file_name = itr_in->path ().filename ();
          std::vector < std::string > strs;
          boost::split (strs, file_name, boost::is_any_of ("_"));

          if (strs[0] == "descriptor")
          {
            std::string full_file_name = itr_in->path ().string ();
            //std::cout << full_file_name << std::endl;

            std::vector < std::string > strs;
            boost::split (strs, full_file_name, boost::is_any_of ("/"));

            FeatureTPtr cvfh_signature (new pcl::PointCloud<FeatureT> ());
            pcl::io::loadPCDFile (full_file_name, *cvfh_signature);

            flann_model descr_model;
            descr_model.first = id + "/" + Model<View, PointOutT, FeatureT>::getIdFromFilename (file_name);

            //std::cout << descr_model.first << " " << strs.size() << std::endl;

            descr_model.second.resize (getHistogramLength (dummy));
            memcpy (&descr_model.second[0], &cvfh_signature->points[0].histogram[0],
                    getHistogramLength (dummy) * sizeof(float));
            models_.push_back (descr_model);

            if (use_cache_)
            {
              Eigen::Vector3f centroid;
              Eigen::Matrix4f pose;
              cv::Mat roll_hist;
              PointOutTPtr view (new pcl::PointCloud<PointOutT>);

              std::stringstream dir;
              dir << TRAINING_DIR_ << "/level_" << tesselated_sphere_level_;
              std::stringstream dir_with_descr;
              dir_with_descr << dir.str () << "/" << D_NAME_;

              PersistenceUtils::getCentroidFromFile (dir_with_descr.str (), descr_model.first, centroid);
              //PersistenceUtils::readCvMat1DFromFile (dir.str(), descr_model.first, roll_hist);
              PersistenceUtils::readMatrixFromFile (dir_with_descr.str (), descr_model.first, pose);
              PersistenceUtils::getPointCloudFromFile<PointOutT> (dir_with_descr.str (), descr_model.first, view);

              /*
               * TODO: Build specific cache!!
               */

              //add to map...
              poses_cache_[descr_model.first] = pose;
              centroids_cache_[descr_model.first] = centroid;
              views_cache_[descr_model.first] = view;
            }

            n_descriptors++;

          }
        }
      }

      convertToFLANN (models_, data_);

      //create index
      index_ = new flann::Index<DistT> (data_, flann::LinearIndexParams ());
      index_->buildIndex ();

      PCL_INFO ("Recognizer initialized with %d descriptors\n", n_descriptors);

      //save flann files for future fast loading...
      flann::save_to_file (data_, training_data_h5_file_name.string ().c_str (), "training_data");
      saveFileList (models_, training_data_list_file_name.string ().c_str ());
      index_->save (index_filename.string ().c_str ());

      PCL_INFO ("Saving flann files\n");
    }
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  bool
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::loadFileList (std::vector<flann_model> &models,
                                                                     const std::string &filename)
  {
    std::ifstream fs;
    fs.open (filename.c_str ());
    if (!fs.is_open () || fs.fail ())
      return (false);

    std::string line;
    while (!fs.eof ())
    {
      getline (fs, line);
      if (line.empty ())
        continue;
      flann_model m;
      m.first = std::string (line.c_str ());
      models.push_back (m);
    }
    fs.close ();
    return (true);
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::saveFileList (const std::vector<flann_model> &models,
                                                                     const std::string &filename)
  {
    std::ofstream fs;
    fs.open (filename.c_str ());
    for (size_t i = 0; i < models.size (); ++i)
      fs << models[i].first << "\n";
    fs.close ();
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  void
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::nearestKSearch (flann::Index<DistT> * index,
                                                                       const flann_model &model, int k,
                                                                       flann::Matrix<int> &indices,
                                                                       flann::Matrix<float> &distances)
  {
    flann::Matrix<float> p = flann::Matrix<float> (new float[model.second.size ()], 1, model.second.size ());
    memcpy (&p.data[0], &model.second[0], p.cols * p.rows * sizeof(float));

    indices = flann::Matrix<int> (new int[k], 1, k);
    distances = flann::Matrix<float> (new float[k], 1, k);
    index->knnSearch (p, indices, distances, k, flann::SearchParams (512));
    p.free ();
  }

template<template<class > class Distance, typename PointInT, typename PointOutT, typename FeatureT>
  bool
  Recognizer<Distance, PointInT, PointOutT, FeatureT>::recognize (
                                                                  PointInTPtr & cluster,
                                                                  std::vector<std::string>& model_ids,
                                                                  std::vector<Eigen::Matrix4f,
                                                                      Eigen::aligned_allocator<Eigen::Matrix4f> > & poses,
                                                                  std::vector<float>& confidences)
  {

    matching_views_.clear ();

    PointOutTPtr cloud_out (new pcl::PointCloud<PointOutT> ());
    std::vector < pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
    std::vector < Eigen::Vector3f > centroidsInput;

    estimator_->estimate (cluster, cloud_out, signatures, centroidsInput);

    //NN-searches
    std::vector<index_score> indices_scores;
    FeatureT dummy; //to get the size of the underlying feature histogram

    //perform search of the different signatures and merge results...
    int counter = 0;
    for (size_t idx = 0; idx < signatures.size (); idx++)
    {
      //save descriptor...

      std::stringstream name_descriptor;
       name_descriptor << "descriptor_" << idx << ".pcd";
       pcl::io::savePCDFileBinary (name_descriptor.str(), signatures[idx]);

      float* hist = signatures[idx].points[0].histogram;
      std::vector<float> std_hist (hist, hist + getHistogramLength (dummy));
      flann_model histogram ("", std_hist);
      flann::Matrix<int> indices;
      flann::Matrix<float> distances;
      nearestKSearch (index_, histogram, nmodels_, indices, distances);

      //gather NN-search results
      double score = 0;
      for (size_t i = 0; i < nmodels_; ++i, ++counter)
      {
        score = distances[0][i];

        std::string cvfh_id = models_.at (indices[0][i]).first;
        //std::cout << "cvfh_id:"  <<  cvfh_id << std::endl;

        index_score is;
        is.idx_models_ = indices[0][i];
        is.idx_input_ = idx;
        is.score_ = score;
        indices_scores.push_back (is);
      }
    }

    //sort results
    std::sort (indices_scores.begin (), indices_scores.end (), sortIndexScores);

    //POST-PROCESSING - give control to the specific recognizer
    //For instance, for CVFH:
    // - compute roll orientation for first nModels...
    // - fitness scores & ICP
    // - reorder results..

    //POST-PROCESSING works directly on indices_scores

    {
      pcl::ScopeTime t ("Specific post-process");
      specificPostProcess (indices_scores, cloud_out, signatures, centroidsInput);
    }

    int nModels = nmodels_;

    //POPULATE OUTPUT...
    model_ids.resize (nModels);
    poses.resize (nModels);
    confidences.resize (nModels);

    for (size_t i = 0; i < nModels && (i < indices_scores.size ()); ++i)
    {
      std::string cvfh_id = models_.at (indices_scores[i].idx_models_).first;
      std::cout << "id:" << cvfh_id << std::endl;

      std::vector < std::string > strs;
      boost::split (strs, cvfh_id, boost::is_any_of ("/"));

      std::string str;
      for (size_t k = 0; k < (strs.size () - 1); k++)
      {
        str += strs[k];
        if (k < (strs.size () - 2))
        {
          str += "/";
        }
      }

      model_ids[i] = str;
      confidences[i] = indices_scores[i].score_;

      PointOutTPtr cloud_match (new pcl::PointCloud<PointOutT>);
      getView (cvfh_id, cloud_match);

      if (compute_pose_)
      {

        Eigen::Matrix4f homMatrixPose;
        getPose (cvfh_id, homMatrixPose);

        Eigen::Vector3f cInput = centroidsInput[indices_scores[i].idx_input_];
        Eigen::Vector3f c_match;
        getCentroid (cvfh_id, c_match);

        computeFinalPose (cInput, c_match, homMatrixPose, poses[i], i);

        Eigen::Matrix4f pose_view;
        computeFinalPoseView (cInput, c_match, pose_view, i);

        PointOutTPtr view_transformed (new pcl::PointCloud<PointOutT>);
        pcl::transformPointCloud (*cloud_match, *view_transformed, pose_view);
        matching_views_.push_back (view_transformed);
      }
      else
      {
        matching_views_.push_back (cloud_match);
        poses[i] = Eigen::Matrix4f::Identity ();
      }
    }

    return true;
  }

template class Recognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<640> > ;
//template class Recognizer<flann::L2, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<153> > ; //spin images...
template class Recognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<416> > ;
template class Recognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::Histogram<480> > ;
template class Recognizer<flann::ChiSquareDistance, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class Recognizer<flann::L1, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class Recognizer<Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::PointNormal, pcl::VFHSignature308> ;
template class Recognizer<Metrics::HistIntersectionUnionDistance, pcl::PointXYZRGB, pcl::PointXYZRGBNormal,
    pcl::VFHSignature308> ;
