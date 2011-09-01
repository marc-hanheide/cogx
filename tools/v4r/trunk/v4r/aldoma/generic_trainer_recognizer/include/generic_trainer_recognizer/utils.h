/*
 * utils.h
 *
 *  Created on: Aug 3, 2011
 *      Author: aitor, walter
 */

#ifndef GENERIC_TRAINER_RECOGNIZER_UTILS_H_
#define GENERIC_TRAINER_RECOGNIZER_UTILS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/apps/dominant_plane_segmentation.h"

using namespace pcl;

#include <sstream>

template <class T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}

template<typename PointT>
  void
  getClusters (
               typename PointCloud<PointT>::Ptr & cloud,
               typename PointCloud<PointT>::Ptr & grid,
               std::vector<typename PointCloud<PointT>::Ptr, Eigen::aligned_allocator<typename PointCloud<PointT>::Ptr> > & clusters,
               float Z_DIST_, float leaf_size_ = 0.005)
  {
    VoxelGrid<PointT> grid_;
    grid_.setInputCloud (cloud);
    grid_.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
    grid_.filter (*grid);

    pcl::PassThrough<PointT> pass_;
    pass_.setFilterLimits (0.0, Z_DIST_);
    pass_.setFilterFieldName ("z");
    pass_.setInputCloud (grid);
    pass_.filter (*grid);

    //segmentation
    apps::DominantPlaneSegmentation<PointT> dps;
    dps.setInputCloud (cloud);
    dps.setMinClusterSize (100);
    dps.setDistanceBetweenClusters (0.06);
    dps.setMaxZBounds (Z_DIST_);
    dps.setDownsamplingSize (leaf_size_);
    dps.setObjectMinHeight (0.01);
    dps.compute (clusters);
    //dps.compute_full(clusters);

    std::cout << "Number of clusters:" << clusters.size () << std::endl;

  }

template<typename PointT>
  void
  getClusters (
               typename PointCloud<PointT>::Ptr & cloud,
               typename PointCloud<PointT>::Ptr & grid,
               std::vector<typename PointCloud<PointT>::Ptr, Eigen::aligned_allocator<typename PointCloud<PointT>::Ptr> > & clusters,
               float Z_DIST_, Eigen::Vector4f & table_coeffs, float leaf_size_ = 0.005)
  {
    //segmentation
    apps::DominantPlaneSegmentation<PointT> dps;
    dps.setMinClusterSize (100);
    dps.setDistanceBetweenClusters (0.06);
    dps.setInputCloud (cloud);
    dps.setMaxZBounds (Z_DIST_);
    dps.setMinZBounds(0.4);
    dps.setDownsamplingSize (leaf_size_);
    dps.setObjectMinHeight (0.01);
    dps.setObjectMaxHeight(1.5);
    dps.setSACThreshold(0.02);
    dps.compute (clusters);
    //dps.compute_full(clusters);

    dps.getTableCoefficients (table_coeffs);

    std::cout << "Number of clusters:" << clusters.size () << std::endl;
  }


template<typename PointT>
  void
  getClusters_full (
               typename PointCloud<PointT>::Ptr & cloud,
               std::vector<typename PointCloud<PointT>::Ptr, Eigen::aligned_allocator<typename PointCloud<PointT>::Ptr> > & clusters,
               float Z_DIST_, Eigen::Vector4f & table_coeffs, float leaf_size_ = 0.02)
  {

    //segmentation
    apps::DominantPlaneSegmentation<PointT> dps;
    dps.setMinClusterSize (100);
    dps.setDistanceBetweenClusters (0.06);
    dps.setInputCloud (cloud);
    dps.setMaxZBounds (Z_DIST_);
    dps.setDownsamplingSize (leaf_size_);
    dps.setObjectMinHeight (0.01);
    dps.setMinZBounds(0.4);
    dps.setObjectMaxHeight(1.5);
    dps.setSACThreshold(0.02);
    dps.compute_full(clusters);

    dps.getTableCoefficients (table_coeffs);

    std::cout << "Number of clusters:" << clusters.size () << std::endl;
  }


/**
 * extract indices of point cloud which will be treated as edges ( points with high curvature )
 * use aitors mesh-resolution to find search-parameter for normals automatically
 */



/*void
 getClustersRGB (
 PointCloud<PointXYZRGB>::Ptr & cloud,
 PointCloud<PointXYZRGB>::Ptr & grid,
 std::vector<PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator<PointCloud<PointXYZRGB>::Ptr> > & clusters)
 {

 pcl::PassThrough < PointXYZRGB > pass_;
 pass_.setFilterLimits (0.0, Z_DIST_);
 pass_.setFilterFieldName ("z");
 pass_.setInputCloud (cloud);
 pass_.filter (*grid);

 VoxelGrid < PointXYZRGB > grid_;
 grid_.setInputCloud (grid);
 grid_.setLeafSize (0.005, 0.005, 0.005);
 grid_.filter (*grid);

 //segmentation
 apps::DominantPlaneSegmentation < PointXYZRGB > dps;
 dps.setMinClusterSize (300);
 dps.setDistanceBetweenClusters (0.03);
 dps.setInputCloud (cloud);
 dps.setMaxZBounds (Z_DIST_);
 dps.setDownsamplingSize (0.005);
 dps.compute (clusters);

 clusters.clear ();
 clusters.resize (dps.cluster_indices_.size ());
 for (size_t i = 0; i < dps.cluster_indices_.size (); i++)
 {
 clusters[i] = (PointCloud<pcl::PointXYZRGB>::Ptr) (new PointCloud<pcl::PointXYZRGB> ());
 pcl::copyPointCloud (*grid, dps.cluster_indices_[i], *clusters[i]);
 }

 std::cout << "Number of clusters:" << clusters.size () << std::endl;

 }*/

template<template<class > class MetricT, typename PointInT, typename PointOutT, typename FeatureT>
void
recognizeAndVisualize (typename pcl::PointCloud<PointInT>::Ptr & cloud,
                       Recognizer<MetricT, PointInT, PointOutT, FeatureT> & recognizer, float Z_DIST_,
                       std::string MODELS_DIR_, float model_scale_factor_, float leaf_size_ = 0.005)
{
  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  PointInTPtr grid (new PointCloud<PointInT> ());
  std::vector<PointInTPtr, Eigen::aligned_allocator<PointInTPtr> > clusters;

  getClusters<PointInT> (cloud, grid, clusters, Z_DIST_, leaf_size_);

  visualization::PCLVisualizer vis4 ("scene");
  vis4.addPointCloud<PointInT> (grid);

  for (size_t i = 0; i < clusters.size (); i++)
  {

    std::stringstream cluster_name;
    cluster_name << "cluster_" << i;

    visualization::PointCloudColorHandlerCustom<PointInT> handler (clusters[i], 255, 0, 0);
    vis4.addPointCloud<PointInT> (clusters[i], handler, cluster_name.str ());

    std::vector < std::string > model_ids;
    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> confidences;

    {
      ScopeTime t ("-------- RECOGNIZE");
      recognizer.recognize (clusters[i], model_ids, poses, confidences);
    }

    int MAX_MODELS = 1;
    for (size_t j = 0; (j < MAX_MODELS && j < model_ids.size ()); j++)
    {
      std::cout << model_ids[j] << std::endl;

      stringstream pathPly;
      pathPly << MODELS_DIR_ << "/" << model_ids[j] << ".ply";

      std::cout << pathPly.str () << std::endl;
      vtkSmartPointer < vtkTransform > trans = vtkSmartPointer<vtkTransform>::New ();
      trans->PostMultiply ();
      float unitConversionDBPlyFiles = model_scale_factor_;
      trans->Scale (unitConversionDBPlyFiles, unitConversionDBPlyFiles, unitConversionDBPlyFiles);
      vtkSmartPointer < vtkTransform > poseTransform = vtkSmartPointer<vtkTransform>::New ();

      vtkSmartPointer < vtkMatrix4x4 > mat = vtkSmartPointer<vtkMatrix4x4>::New ();
      for (size_t kk = 0; kk < 4; kk++)
      {
        for (size_t k = 0; k < 4; k++)
        {
          mat->SetElement (kk, k, poses[j] (kk, k));
        }
      }

      poseTransform->SetMatrix (mat);
      trans->Concatenate (poseTransform);
      trans->Modified ();

      std::stringstream model_name;
      model_name << "ply_model_" << i << "_" << j;

      vis4.addModelFromPLYFile (pathPly.str (), trans, model_name.str ());

      std::stringstream matching_view_name;
      matching_view_name << "matching_view_" << i << "_" << j;

      visualization::PointCloudColorHandlerCustom<PointOutT> handler_match (recognizer.matching_views_[j], 0, 255, 0);
      vis4.addPointCloud<PointOutT> (recognizer.matching_views_[j], handler_match, matching_view_name.str ());
      vis4.spin ();
    }
  }

  vis4.spin ();
}

class recognition_result
{
public:
  std::vector<std::string> ids_;
  std::vector<float> confidences_;
};

template<template<class > class MetricT, typename PointInT, typename PointOutT, typename FeatureT>
void
recognizeAndReturnResults (
                           typename pcl::PointCloud<PointInT>::Ptr & cloud,
                           Recognizer<MetricT, PointInT, PointOutT, FeatureT> & recognizer,
                           float Z_DIST_,
                           std::string MODELS_DIR_,
                           int numberNN_,
                           std::vector<recognition_result> & results,
                           Eigen::Vector4f & table_coeffs,
                           std::vector<typename pcl::PointCloud<PointInT>::Ptr, Eigen::aligned_allocator<
                               typename pcl::PointCloud<PointInT>::Ptr> > & clusters, float leaf_size_ = 0.005)
{

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

  PointInTPtr grid (new PointCloud<PointInT> ());

  getClusters<PointInT> (cloud, grid, clusters, Z_DIST_, table_coeffs, leaf_size_);

  for (size_t i = 0; i < clusters.size (); i++)
  {

    std::vector < std::string > model_ids;
    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> confidences;

    {
      ScopeTime t ("-------- RECOGNIZE");
      recognizer.recognize (clusters[i], model_ids, poses, confidences);
    }

    recognition_result rr;
    rr.ids_ = model_ids;
    rr.confidences_ = confidences;

    results.push_back (rr);

  }

}

template<template<class > class MetricT, typename PointInT, typename PointOutT, typename FeatureT>
void
recognizeAndReturnResultsTxt (
                           typename pcl::PointCloud<PointInT>::Ptr & cloud,
                           Recognizer<MetricT, PointInT, PointOutT, FeatureT> & recognizer,
                           float Z_DIST_,
                           std::string MODELS_DIR_,
                           int numberNN_,
                           std::vector<recognition_result> & results,
                           float leaf_size_ , std::string cluster_fname, bool is_cluster)
{

  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

  Eigen::Vector4f table_coeffs;
  std::vector<PointInTPtr, Eigen::aligned_allocator<PointInTPtr> > clusters;

  if ( is_cluster == false)
  {
	  getClusters_full<PointInT> (cloud, clusters, Z_DIST_, table_coeffs, leaf_size_);
	  pcl::io::savePCDFile(cluster_fname, *clusters[0]);
  }
  else
	  clusters.push_back(cloud);
//  extract_clusters_fast<PointInT> (cloud, clusters, Z_DIST_, table_coeffs);

  for (size_t i = 0; i < clusters.size (); i++)
  {
    std::vector < std::string > model_ids;
    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> confidences;

    {
      ScopeTime t ("-------- RECOGNIZE");
      recognizer.recognize (clusters[i], model_ids, poses, confidences);
    }

    recognition_result rr;
    rr.ids_ = model_ids;
    rr.confidences_ = confidences;
    results.push_back (rr);
  }
}

template<template<class > class MetricT, typename PointInT, typename PointOutT, typename FeatureT>
void
recognizeAndVisualizeNearestNeighbors (typename pcl::PointCloud<PointInT>::Ptr & cloud,
                                       Recognizer<MetricT, PointInT, PointOutT, FeatureT> & recognizer,
                                       float Z_DIST_, std::string MODELS_DIR_, int numberNN_,
                                       float leaf_size_ = 0.005)
{
  typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
  typedef typename pcl::PointCloud<PointOutT>::Ptr PointOutTPtr;

  PointInTPtr grid (new PointCloud<PointInT> ());

  std::vector<PointInTPtr, Eigen::aligned_allocator<PointInTPtr> > clusters;

  getClusters<PointInT> (cloud, grid, clusters, Z_DIST_,leaf_size_);

  for (size_t i = 0; i < clusters.size (); i++)
  {

    visualization::PCLVisualizer vis ("Nearest neighbours");
    int viewport = 0, l = 0, m = 0;

    std::stringstream cluster_name;
    cluster_name << "cluster_" << i;
    int MAX_MODELS = numberNN_;
    size_t kk = MAX_MODELS + 1;
    int y_s = (int)floor (sqrt (kk));
    int x_s = y_s + (int)ceil ((kk / (double)y_s) - y_s);
    double x_step = (double)(1 / (double)x_s);
    double y_step = (double)(1 / (double)y_s);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*clusters[i], centroid);
    PointInTPtr cluster_centered (new pcl::PointCloud<PointInT> ());
    pcl::demeanPointCloud (*clusters[i], centroid, *cluster_centered);

    vis.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);

    visualization::PointCloudColorHandlerCustom<PointInT> handler (cluster_centered, 255, 0, 0);
    vis.addPointCloud<PointInT> (cluster_centered, handler, cluster_name.str (), viewport);

    std::vector < std::string > model_ids;
    std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> confidences;

    {
      ScopeTime t ("-------- RECOGNIZE");
      recognizer.recognize (clusters[i], model_ids, poses, confidences);
    }

    for (size_t j = 0; (j < MAX_MODELS && j < model_ids.size ()); j++)
    {
      std::cout << model_ids[j] << std::endl;

      stringstream pathPly;
      pathPly << MODELS_DIR_ << "/" << model_ids[j] << ".ply";
      std::cout << pathPly.str () << std::endl;

      l++;
      if (l >= x_s)
      {
        l = 0;
        m++;
      }

      std::stringstream matching_view_name;
      matching_view_name << "matching_view_" << i << "_" << j;

      vis.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*recognizer.matching_views_[j], centroid);
      PointOutTPtr view_centered (new pcl::PointCloud<PointOutT> ());
      pcl::demeanPointCloud (*recognizer.matching_views_[j], centroid, *view_centered);

      visualization::PointCloudColorHandlerCustom<PointOutT> handler_match (view_centered, 0, 255, 0);
      vis.addPointCloud<PointOutT> (view_centered, handler_match, matching_view_name.str (), viewport);

      stringstream ss;
      ss << confidences[j];

      stringstream id_ss;
      id_ss << "confidence_" << i << "_" << j;
      vis.addText (ss.str (), 20, 60, 1, 0, 1, id_ss.str (), viewport);
    }

    vis.spin ();
  }
}



#endif /* UTILS_H_ */
