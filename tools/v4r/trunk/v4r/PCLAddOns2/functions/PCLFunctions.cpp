/**
 * @file PCLFunctions.cpp
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#include "PCLFunctions.h"
#include "v4r/PCLAddOns2/utils/PCLUtils.h"

namespace pclA
{

bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          bool useVoxelGrid,
                          double vg_size,
                          bool useZFilter,
                          double minZ,
                          double maxZ)
{
  if(useZFilter)
    pclA::FilterZ(pcl_cloud, minZ, maxZ);

  // removes zeros and NANs in the cloud
  pclA::RemoveZeros(pcl_cloud);

  if (useVoxelGrid)
  {
    printf("PCLFunctions::PreProcessPointCloud: Warning: Voxel grid do not support indexed point clouds.\n");
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(pcl_cloud);
    vg.setLeafSize (vg_size, vg_size, vg_size);
    vg.filter(*pcl_cloud);
  }
  return true;
}

bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          std::vector<int> &indexes,
                          bool useVoxelGrid,
                          double vg_size,
                          bool useZFilter,
                          double minZ,
                          double maxZ)
{
  if(useZFilter)
    pclA::FilterZ(pcl_cloud, minZ, maxZ);

  // removes zeros and NANs in the cloud and and indexes
  pclA::RemoveZerosIndexed(pcl_cloud, indexes);

  if (useVoxelGrid)
  {
    printf("PCLFunctions::PreProcessPointCloud: Warning: Voxel grid do not support indexed point clouds.\n");
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(pcl_cloud);
    vg.setLeafSize (vg_size, vg_size, vg_size);
    vg.filter(*pcl_cloud);
  }
  return true;
}

bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          std::vector<int> &indexes,
                          pcl::PointCloud<pcl::Normal>::Ptr &normals,
                          bool useVoxelGrid,
                          double vg_size,
                          bool useZFilter,
                          double minZ,
                          double maxZ)
{
  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLFunctions::PreProcessPointCloud: Warning: Size of point cloud and indexes differ: re-indexing.\n");
    indexes.resize(0);
    for(unsigned i=0; i<pcl_cloud->points.size(); i++)
      indexes.push_back(i);
  }
  
  if(useZFilter)
    pclA::FilterZ(pcl_cloud, minZ, maxZ);

  // remove zeros and NANs and remove also this points in the normal cloud.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::Normal>::Ptr newNormals(new pcl::PointCloud<pcl::Normal>);
  std::vector<int> new_indexes;
  for( unsigned idx = 0; idx < pcl_cloud->points.size(); idx++ )
  {
    if( pcl_cloud->points[idx].z != 0. && (pcl_cloud->points[idx].x == pcl_cloud->points[idx].x) )
    {
      newCloud->points.push_back(pcl_cloud->points[idx]);
      newNormals->points.push_back(normals->points[idx]);
      new_indexes.push_back(indexes[idx]);
    }
  }
  pcl_cloud = newCloud;
  normals = newNormals;
  indexes = new_indexes;
  
  if (useVoxelGrid)
  {
    printf("PCLFunctions::PreProcessPointCloud: Warning: Voxel grid do not support indexed point clouds.\n");
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(pcl_cloud);
    vg.setLeafSize (vg_size, vg_size, vg_size);
    vg.filter(*pcl_cloud);
  }
  return true;
}

bool FitMultipleModelRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                               std::vector<int> &indexes,
                               pcl::PointCloud<pcl::Normal>::Ptr normals,
                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                               std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                               std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                               std::vector<int> &sac_models,
                               std::vector<int> &result_models,
                               bool sac_optimal_distance,
                               double sac_optimal_weight_threshold,
                               double sac_distance,
                               int sac_max_iterations,
                               int sac_min_inliers,
                               double ec_cluster_tolerance,
                               int ec_min_cluster_size,
                               int ec_max_cluster_size)
{
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > pcl_clustered_normals;
  std::vector< std::vector<int> > pcl_clustered_indexes;

  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLFunctions::FitMultipleModelRecursive: Warning: Cloud and index size differs: %u-%u.\n", 
           pcl_cloud->points.size(), indexes.size());
    return false;
  }
  
  if(normals.get() == 0) // we do not have normals
  {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
  }
  else if (normals->size() == 0)  // or normals are empty
  {
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
  }
  if (MultipleSACModelSegmentation(pcl_cloud, indexes, normals, pcl_model_clouds, pcl_model_cloud_indexes, model_coefficients, sac_models,
                                   result_models, sac_optimal_distance, sac_optimal_weight_threshold,
                                   sac_distance, sac_max_iterations, sac_min_inliers))
  {
    if (EuclideanClustering(pcl_cloud, indexes, pcl_clustered_clouds, pcl_clustered_indexes, normals, pcl_clustered_normals,
                            ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
    {
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++) {
          FitMultipleModelRecursive(pcl_clustered_clouds[i], pcl_clustered_indexes[i], pcl_clustered_normals[i], pcl_model_clouds, 
                                    pcl_model_cloud_indexes, model_coefficients, sac_models, result_models, sac_optimal_distance, 
                                    sac_optimal_weight_threshold, sac_distance, sac_max_iterations,
                                    sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
      }
    }
  }
  return true;
}

bool FitModelRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                       std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                       std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                       int sac_model,
                       bool sac_optimal_distance,
                       double sac_optimal_weight_threshold,
                       double sac_distance,
                       int sac_max_iterations,
                       int sac_min_inliers,
                       double ec_cluster_tolerance,
                       int ec_min_cluster_size,
                       int ec_max_cluster_size)
{
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  if (SingleSACModelSegmentation(pcl_cloud, pcl_plane_clouds, model_coefficients, sac_model, sac_optimal_distance, 
                                 sac_optimal_weight_threshold, sac_distance, sac_max_iterations, sac_min_inliers))
  {
    if (EuclideanClustering(pcl_cloud, pcl_clustered_clouds, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
    {
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
          FitModelRecursive(pcl_clustered_clouds[i], pcl_plane_clouds, model_coefficients, sac_model, 
                            sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations,
                            sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
    }
  }
  return true;
}


bool FitModelRecursiveWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                  std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                  int sac_model,
                                  bool sac_optimal_distance,
                                  double sac_optimal_weight_threshold,
                                  double sac_distance,
                                  int sac_max_iterations,
                                  int sac_min_inliers,
                                  double ec_cluster_tolerance,
                                  int ec_min_cluster_size,
                                  int ec_max_cluster_size)
{
printf("PclFunctions::FitModelRecursiveWithNormals: Warning: Antiquated function.\n");
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  if (SingleSACModelSegmentationWithNormals(pcl_cloud, normals, pcl_plane_clouds, model_coefficients, sac_model, 
                                            sac_optimal_distance, sac_optimal_weight_threshold, 
                                            sac_distance, sac_max_iterations, sac_min_inliers))
  {
    if (EuclideanClustering(pcl_cloud, pcl_clustered_clouds, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
          FitModelRecursiveWithNormals(pcl_clustered_clouds[i], pcl_plane_clouds, model_coefficients, sac_model, 
                                       sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations, 
                                       sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
  }
  return true;
}


bool SingleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                int sac_model,
                                bool calc_optimal_sac_distance,
                                double sac_optimal_weight_factor,
                                double sac_distance,
                                int maxIterations,
                                int minInliers)
{
  if (calc_optimal_sac_distance)
    pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  bool succeed = false;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(sac_model);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.segment(*inliers, *model_coef);

  if (inliers->indices.size() >= minInliers)
  {
    // get cloud and delete points from pcl_cloud
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(inliers);
    extract_inliers.filter(*pcl_cloud);
    
    succeed = true;
  }

  if (pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}


bool SingleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                std::vector<int> &indexes,
                                std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                int sac_model,
                                bool calc_optimal_sac_distance,
                                double sac_optimal_weight_factor,
                                double sac_distance,
                                int maxIterations,
                                int minInliers)
{
  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLFunctions::SingleSACModelSegmentation: Warning: Cloud and index size differs.\n");
    return false;
  }
  
  if (calc_optimal_sac_distance)
    pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  bool succeed = false;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(sac_model);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.segment(*inliers, *model_coef);

  if (inliers->indices.size() >= minInliers)
  {
    // get cloud and delete points from pcl_cloud
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(inliers);
    extract_inliers.filter(*pcl_cloud);
    
    std::vector<int>::iterator it;
    std::vector<int> model_cloud_indexes;
    for(unsigned idx=0; idx<inliers->indices.size(); idx++)
      model_cloud_indexes.push_back(indexes[inliers->indices[idx]]);
    pcl_model_cloud_indexes.push_back(model_cloud_indexes);

    for(int idx=inliers->indices.size()-1; idx>=0; idx--)
    {
      it = indexes.begin() + inliers->indices[idx];
      indexes.erase(it);
    }
    
    succeed = true;
  }

  if (pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}

bool MultipleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                  std::vector<int> &indexes,
                                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                                  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                                  std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                  std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                  std::vector<int> &sac_models,
                                  std::vector<int> &result_models,
                                  bool calc_optimal_sac_distance,
                                  double sac_optimal_weight_factor,
                                  double sac_distance,
                                  int maxIterations,
                                  int minInliers)
{
  bool succeed = false;
  int tmp_inliers = 0;
  int necessaryInliers = 0;           /// TODO Try again

  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLFunctions::MultipleSACModelSegmentation: Warning: Cloud and index size differs.\n");
    return false;
  }

  if (calc_optimal_sac_distance)
      pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  /// TODO Calculate now the support for the given models!!!
  int max_inliers = 0;
  int max_inlier_model = 0;
  pcl::PointIndices::Ptr result_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr result_model_coef (new pcl::ModelCoefficients);

  for (unsigned i=0; i< sac_models.size(); i++)
  {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    
    seg.setModelType(sac_models[i]);
    if(sac_models[i] == pcl::SACMODEL_CYLINDER)
    {
      tmp_inliers = minInliers/**2*/;
      seg.setOptimizeCoefficients (true);
      seg.setNormalDistanceWeight (0.05);   // 0.05-0.1 ???
      seg.setRadiusLimits (0, 0.12);  // 12cm radius
      seg.setDistanceThreshold(0.02/*sac_distance*1.5*/);
      seg.setMaxIterations(10000/*maxIterations*3*/);
    }
    else if(sac_models[i] == pcl::SACMODEL_SPHERE)
    {
      tmp_inliers = minInliers;
      seg.setOptimizeCoefficients (true);
      seg.setNormalDistanceWeight (0.2);
      seg.setRadiusLimits (0, 0.15);
      seg.setDistanceThreshold(sac_distance/3.);
      seg.setMaxIterations(maxIterations);
    } 
    else
    {
      tmp_inliers = minInliers;
//           seg.setOptimizeCoefficients (true);
      seg.setDistanceThreshold(sac_distance);
      seg.setMaxIterations(maxIterations);
    }
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setInputCloud(pcl_cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *model_coef);

    if (inliers->indices.size() > max_inliers)
    {
        max_inliers = inliers->indices.size();
        max_inlier_model = sac_models[i];
        result_inliers = inliers;
        result_model_coef = model_coef;
        necessaryInliers = tmp_inliers;
    }
  }

  if (max_inliers >= necessaryInliers)
  {
    result_models.push_back(max_inlier_model);

    pcl_model_clouds.resize(pcl_model_clouds.size() + 1);
    pcl_model_clouds[pcl_model_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *result_inliers, *pcl_model_clouds[pcl_model_clouds.size()-1]);
    model_coefficients.push_back(result_model_coef);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(result_inliers);
    extract_inliers.filter(*pcl_cloud);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(normals);
    extract_normals.setIndices(result_inliers);
    extract_normals.filter (*normals);

    std::vector<int>::iterator it;
    std::vector<int> model_cloud_indexes;
    for(unsigned idx=0; idx<result_inliers->indices.size(); idx++)
      model_cloud_indexes.push_back(indexes[result_inliers->indices[idx]]);
    pcl_model_cloud_indexes.push_back(model_cloud_indexes);

    for(int idx=result_inliers->indices.size()-1; idx>=0; idx--)
    {
      it = indexes.begin() + result_inliers->indices[idx];
      indexes.erase(it);
    }

    succeed = true;
  }

  if (pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}


bool SingleSACModelSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                           int sac_model,
                                           bool calc_optimal_sac_distance,
                                           double sac_optimal_weight_factor,
                                           double sac_distance,
                                           int maxIterations,
                                           int minInliers)
{
  if (calc_optimal_sac_distance)
      pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  if(normals.get() == 0 || normals->points.size() == 0)
  {
    printf("PCLFunctions::SingleSACModelSegmentationWithNormals: Warning: No normals.\n");
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
  }

  bool succeed = false;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setModelType(sac_model); /// SACMODEL_NORMAL_PLANE
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *model_coef);

  if (inliers->indices.size() >= minInliers)
  {
    // copy inlier cloud and delete points from pcl_cloud
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(inliers);
    extract_inliers.filter(*pcl_cloud);
    
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter (*normals);

    succeed = true;
  }

  if (pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}

bool SingleSACModelSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                           std::vector<int> &indexes,
                                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                           std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                           int sac_model,
                                           bool calc_optimal_sac_distance,
                                           double sac_optimal_weight_factor,
                                           double sac_distance,
                                           int maxIterations,
                                           int minInliers)
{
  if(pcl_cloud->points.size() != indexes.size()) {
    printf("PCLFunctions::MultipleSACModelSegmentation: Warning: Cloud and index size differs.\n");
    return false;
  }
  
  if (calc_optimal_sac_distance)
      pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  if(normals.get() == 0 || normals->points.size() == 0)
  {
    printf("PCLFunctions::SingleSACModelSegmentationWithNormals: Warning: No normals.\n");
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
  }

  bool succeed = false;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setModelType(sac_model); /// SACMODEL_NORMAL_PLANE
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *model_coef);

  if (inliers->indices.size() >= minInliers)
  {
    // copy inlier cloud and delete points from pcl_cloud
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, *inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(inliers);
    extract_inliers.filter(*pcl_cloud);
    
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter (*normals);

    std::vector<int>::iterator it;
    std::vector<int> model_cloud_indexes;
    for(unsigned idx=0; idx<inliers->indices.size(); idx++)
      model_cloud_indexes.push_back(indexes[inliers->indices[idx]]);
    pcl_model_cloud_indexes.push_back(model_cloud_indexes);

    for(int idx=inliers->indices.size()-1; idx>=0; idx--)
    {
      it = indexes.begin() + inliers->indices[idx];
      indexes.erase(it);
    }
    
    succeed = true;
  }

  if (pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}


bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         double cluster_tolerance,
                         double min_cluster_size,
                         double max_cluster_size)
{
    pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree->setInputCloud(pcl_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    std::vector<pcl::PointIndices> inliers_vector;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(pcl_cloud);
    ec.extract(inliers_vector);

    pcl_cluster_clouds.resize(pcl_cluster_clouds.size() + inliers_vector.size());
    for (unsigned idx=0; idx<inliers_vector.size(); idx++)
    {
      pcl_cluster_clouds[idx].reset(new pcl::PointCloud<pcl::PointXYZRGB>); // = tmp_cloud;
      pcl::copyPointCloud (*pcl_cloud, inliers_vector[idx], *pcl_cluster_clouds[idx]);
    }

    if (inliers_vector.size() > 0) return true;
    return false;
}

bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector<int> &indexes,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         std::vector< std::vector<int> > &pcl_clustered_indexes,
                         double cluster_tolerance,
                         double min_cluster_size,
                         double max_cluster_size)
{
    pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree->setInputCloud(pcl_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    std::vector<pcl::PointIndices> inliers_vector;
    ec.setClusterTolerance (cluster_tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(pcl_cloud);
    ec.extract(inliers_vector);

    pcl_cluster_clouds.resize(pcl_cluster_clouds.size() + inliers_vector.size());
    for (unsigned idx=0; idx<inliers_vector.size(); idx++)
    {
      pcl_cluster_clouds[idx].reset(new pcl::PointCloud<pcl::PointXYZRGB>); // = tmp_cloud;
      pcl::copyPointCloud (*pcl_cloud, inliers_vector[idx], *pcl_cluster_clouds[idx]);
        
      std::vector<int> new_indexes;
      for(unsigned jdx=0; jdx<inliers_vector[idx].indices.size(); jdx++)
        new_indexes.push_back(indexes[inliers_vector[idx].indices[jdx]]);
      pcl_clustered_indexes.push_back(new_indexes);
    }

    if (inliers_vector.size() > 0) return true;
    return false;
}


bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector<int> &indexes,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         std::vector< std::vector<int> > &pcl_clustered_indexes,
                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                         std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &pcl_cluster_normals,
                         double cluster_tolerance,
                         double min_cluster_size,
                         double max_cluster_size)
{
  if(pcl_cloud->points.size() != normals->points.size()) { 
    printf("PCLFunctions::EuclideanClustering: Error: Number of points and normals is different (%u - %u).\n", 
           (unsigned)pcl_cloud->points.size(), (unsigned)normals->points.size());
    return false;
  }
// printf("PclFunctions::EuclideanClustering: pcl_cloud.size: %u.\n", pcl_cloud->points.size());
    
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
// printf("PclFunctions::EuclideanClustering: 01.\n");
  tree->setInputCloud(pcl_cloud);

// printf("PclFunctions::EuclideanClustering: 1.\n");

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> inliers_vector;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(inliers_vector);

  pcl_cluster_clouds.resize(pcl_cluster_clouds.size() + inliers_vector.size());
  pcl_cluster_normals.resize(pcl_cluster_normals.size() + inliers_vector.size());
  for (unsigned idx=0; idx<inliers_vector.size(); idx++)
  {
    pcl_cluster_clouds[idx].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cluster_normals[idx].reset(new pcl::PointCloud<pcl::Normal>);

    pcl::copyPointCloud (*pcl_cloud, inliers_vector[idx], *pcl_cluster_clouds[idx]);
    pcl::copyPointCloud (*normals, inliers_vector[idx], *pcl_cluster_normals[idx]);

    std::vector<int> new_indexes;
    for(unsigned jdx=0; jdx<inliers_vector[idx].indices.size(); jdx++)
      new_indexes.push_back(indexes[inliers_vector[idx].indices[jdx]]);
    pcl_clustered_indexes.push_back(new_indexes);
  }

  if (inliers_vector.size() > 0) return true;
  return false;
}

bool GetConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                   pcl::ModelCoefficients::Ptr model_coefficient,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_convex_hull)

{
    // Project the model inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pcl_cloud);
    proj.setModelCoefficients(model_coefficient);
    proj.filter(*cloud_projected);

    // Create a Convex Hull representation of the projected inliers
    pcl_convex_hull.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setKeepInformation(false);  // keep information about point (color)!
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*pcl_convex_hull);
}

bool GetConvexHulls(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                    std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_convex_hulls)

{
    if (pcl_clouds.size() != model_coefficients.size())
    {
        printf("PCLFunctions::GetConvexHulls: Warning: Cloud and coefficient size do not match!\n");
        return false;
    }

    for (unsigned i=0; i<pcl_clouds.size(); i++)
    {
        // Project the model inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(pcl_clouds[i]);
        proj.setModelCoefficients(model_coefficients[i]);
        proj.filter(*cloud_projected);

        // Create a Convex Hull representation of the projected inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ConvexHull<pcl::PointXYZRGB> chull;
        chull.setKeepInformation(false);   // keep information about point (color)!
        chull.setInputCloud (cloud_projected);
        chull.reconstruct (*cloud_hull);

        pcl_clouds[i] = cloud_projected;
        pcl_convex_hulls.push_back(cloud_hull);
    }
}

double GetConvexHullArea(std::vector<cv::Vec3f> &pcl_convex_hull)
{
  int num_hull_points = pcl_convex_hull.size();
  cv::Vec3f sum_area(0., 0., 0.);
  for(int i=0; i<num_hull_points-2; i++)
  {
    cv::Vec3f p0 = pcl_convex_hull[i+2] -  pcl_convex_hull[0];
    cv::Vec3f p1 = pcl_convex_hull[i+1] -  pcl_convex_hull[0];
    
    cv::Vec3f cross = p0.cross(p1);
    sum_area += cross;
  }
 return cv::norm(sum_area)/2.;
}

bool GetProjectedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        pcl::ModelCoefficients::Ptr model_coefficients,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud_projected)

{
    pcl_cloud_projected.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setModelCoefficients(model_coefficients);
    proj.setInputCloud(pcl_cloud);
    proj.setCopyAllData(true);      // copy all data, not only inliers!
    proj.filter(*pcl_cloud_projected);
}


bool GetProjectedPoints(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds,
                        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds_projected)

{
    if (pcl_clouds.size() != model_coefficients.size())
    {
        printf("PCLFunctions::GetConvexHulls: Warning: Cloud and coefficient size do not match!\n");
        return false;
    }

    for (unsigned i=0; i<pcl_clouds.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setModelCoefficients(model_coefficients[i]);
        proj.setInputCloud(pcl_clouds[i]);
        proj.setCopyAllData(true);      // copy all data, not only inliers!
        proj.filter(*cloud_projected);

        pcl_clouds_projected.push_back(cloud_projected);
    }
}

void MaxDistanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        const pcl::ModelCoefficients::Ptr &model_coefficients,
                        double &max_distance)
{
  max_distance = 0.;
  for (unsigned i=0; i<pcl_cloud->points.size(); i++)
  {
    double dist = pcl::pointToPlaneDistanceSigned(pcl_cloud->points[i], model_coefficients->values[0], 
                  model_coefficients->values[1], model_coefficients->values[2], model_coefficients->values[3]);

    if (dist > max_distance)
      max_distance = dist;
  }
}

void CalcSquareErrorToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                             const pcl::ModelCoefficients::Ptr &model_coefficients,
                             double &square_error)
{
  square_error = 0.;
  double sigma = 0.005; // 1mm
  for(unsigned i=0; i<pcl_cloud->points.size(); i++)
  {
    double dist = pcl::pointToPlaneDistanceSigned(pcl_cloud->points[i], model_coefficients->values[0], 
                  model_coefficients->values[1], model_coefficients->values[2], model_coefficients->values[3]);
    square_error += 1-exp(-(dist*dist/(sigma*sigma)));
  }
  square_error /= pcl_cloud->points.size();
}

void CreateSOI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
               const pcl::ModelCoefficients::Ptr &model_coefficients,
               const double distance,
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &soi)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newSoi (new pcl::PointCloud<pcl::PointXYZRGB>);
  soi = newSoi;
  soi->width = pcl_cloud->points.size()*2;
  soi->height = 1;

  cv::Vec3f vector;
  vector[0] = model_coefficients->values[0] * distance;
  vector[1] = model_coefficients->values[1] * distance;
  vector[2] = model_coefficients->values[2] * distance;

  for (unsigned i=0; i<pcl_cloud->points.size(); i++)
      soi->points.push_back(pcl_cloud->points[i]);

  for (unsigned i=0; i<pcl_cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = pcl_cloud->points[i].x + vector[0];
    p.y = pcl_cloud->points[i].y + vector[1];
    p.z = pcl_cloud->points[i].z + vector[2];
    soi->points.push_back(p);
  }
}


void NormalsFromPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                         cv::Mat_<cv::Vec4f> &normals,
                         float radius_search)
{
	normals = cv::Mat_<cv::Vec4f>(pcl_cloud->height, pcl_cloud->width);

	pcl::PointCloud < pcl::Normal > cloud_normals;
	pcl::NormalEstimation < pcl::PointXYZRGB, pcl::Normal > ne;
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());

	ne.setInputCloud(pcl_cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(radius_search);

	ne.compute(cloud_normals);

	for( int v = 0; v < (int) cloud_normals.height; ++v )
	{
		for( int u = 0; u < (int) cloud_normals.width; ++u )
		{
			const pcl::Normal &n = cloud_normals(u, v);

			cv::Vec4f &ptNorm = normals(v, u);

			if( n.normal != n.normal || isnan(n.normal[0]) || isnan(n.normal[1]) || isnan(n.normal[2]) )
			{
				ptNorm[0] = 0.0f;
				ptNorm[1] = 0.0f;
				ptNorm[2] = 0.0f;
				ptNorm[3] = 0.0f;
			} else
			{
				ptNorm[0] = n.normal[0];
				ptNorm[1] = n.normal[1];
				ptNorm[2] = n.normal[2];
				ptNorm[3] = n.curvature;
			}
		}
	}
}

void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr &normals,
                               float max_depth_change, float normal_smoothing_size)
{
  if(normals.get() == 0)
    normals.reset(new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(max_depth_change);
  ne.setNormalSmoothingSize(normal_smoothing_size);
  ne.setInputCloud(cloud);
  ne.compute(*normals);
  
  for( int v = 0; v < (int) normals->height; ++v )
  {
    for( int u = 0; u < (int) normals->width; ++u )
    { 
      int pos = v*normals->width + u;
      pcl::Normal &n = normals->points[pos];
      if( n.normal != n.normal || isnan(n.normal[0]) || isnan(n.normal[1]) || isnan(n.normal[2]) )
      {
        n.normal[0] = 0.0f;
        n.normal[1] = 0.0f;
        n.normal[2] = 0.0f;
        n.normal[3] = 0.0f;
      }
    }
  }
}

void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               cv::Mat_<cv::Vec4f> &normals,
                               float max_depth_change, float normal_smoothing_size)
{
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

	pcl::PointCloud < pcl::Normal > cloud_normals;

	ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor(max_depth_change);
	ne.setNormalSmoothingSize(normal_smoothing_size);
	ne.setInputCloud(cloud);
	ne.compute(cloud_normals);
	normals = cv::Mat_<cv::Vec4f>(cloud->height, cloud->width);
	for( int v = 0; v < (int) cloud_normals.height; ++v )
	{
		for( int u = 0; u < (int) cloud_normals.width; ++u )
		{
			const pcl::Normal &n = cloud_normals(u, v);
			cv::Vec4f &ptNorm = normals(v, u);

			if( n.normal != n.normal || isnan(n.normal[0]) || isnan(n.normal[1]) || isnan(n.normal[2]) )
			{
				ptNorm[0] = 0.0f;
				ptNorm[1] = 0.0f;
				ptNorm[2] = 0.0f;
				ptNorm[3] = 0.0f;
			} else
			{
				ptNorm[0] = n.normal[0];
				ptNorm[1] = n.normal[1];
				ptNorm[2] = n.normal[2];
				ptNorm[3] = n.curvature;
			}
		}
	}
}


void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr &pcl_normals,
                               cv::Mat_<cv::Vec4f> &cv_normals,
                               float max_depth_change, float normal_smoothing_size)
{
printf("NormalsFromSortedPCLCloud: werden aus den NANs hier wirklich nullen?\n");

  if(pcl_normals.get() == 0)
    pcl_normals.reset(new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(max_depth_change);
  ne.setNormalSmoothingSize(normal_smoothing_size);
  ne.setInputCloud(cloud);
  ne.compute(*pcl_normals);

  cv_normals = cv::Mat_<cv::Vec4f>(cloud->height, cloud->width);

  for( int v = 0; v < (int) pcl_normals->height; ++v )
  {
    for( int u = 0; u < (int) pcl_normals->width; ++u )
    { 
      int pos = v*pcl_normals->width + u;
      pcl::Normal &n = pcl_normals->points[pos];
      cv::Vec4f &ptNorm = cv_normals(v, u);
      
      if( n.normal != n.normal || isnan(n.normal[0]) || isnan(n.normal[1]) || isnan(n.normal[2]) )
      {
        n.normal[0] = 0.0f;
        n.normal[1] = 0.0f;
        n.normal[2] = 0.0f;
        n.normal[3] = 0.0f;
        ptNorm[0] = 0.0f;
        ptNorm[1] = 0.0f;
        ptNorm[2] = 0.0f;
        ptNorm[3] = 0.0f;
      } else 
      {
        ptNorm[0] = n.normal[0];
        ptNorm[1] = n.normal[1];
        ptNorm[2] = n.normal[2];
        ptNorm[3] = n.curvature;
      }
    }
  }
}


}









