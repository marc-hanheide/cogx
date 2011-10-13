/**
 * @file ModelFitter.h
 * @author Richtsfeld
 * @date October 2011
 * @version 0.1
 * @brief Extract shape models (plane, cylinder, sphere, ...) from kinect data.
 */

#include "ModelFitter.h"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"

namespace pclA
{


/************************************************************************************
 * Constructor/Destructor
 */

ModelFitter::ModelFitter(Parameter _param) : param(_param)
{
  have_normals = false;
}

ModelFitter::~ModelFitter()
{
}

// ================================= Private functions ================================= //

void ModelFitter::SetIndices(int cloud_size)
{
  indexes.resize(0);
  for(unsigned i=0; i<cloud_size; i++)
    indexes.push_back(i);
}



// ================================= Public functions ================================= //

void ModelFitter::AddModelType(int model_type)
{
  sac_models.push_back(model_type);
}

void ModelFitter::SetNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  if(_normals.get() == 0 || _normals->points.size() == 0) {
    printf("ModelFitter::SetNormals: Warning: Empty or invalid normals vector.\n");
    return;
  }  
  normals = _normals;
  have_normals = true;
}


void ModelFitter::Process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  SetIndices(pcl_cloud->points.size());
  
  if(sac_models.size() == 0)
    sac_models.push_back(pcl::SACMODEL_PLANE);

  if(have_normals) 
    pclA::PreProcessPointCloud(pcl_cloud, indexes, normals, param.use_voxel_grid, param.voxel_grid_size);
  else
    pclA::PreProcessPointCloud(pcl_cloud, indexes, param.use_voxel_grid, param.voxel_grid_size);
  
  pclA::FitMultipleModelRecursive(pcl_cloud, 
                                  indexes,
                                  normals,
                                  pcl_model_clouds, 
                                  pcl_model_cloud_indexes,
                                  model_coefficients, 
                                  sac_models, 
                                  pcl_model_types, 
                                  param.sac_optimal_distance,
                                  param.sac_optimal_weight_factor, 
                                  param.sac_distance, 
                                  param.sac_max_iterations,
                                  param.sac_min_inliers, 
                                  param.ec_cluster_tolerance, 
                                  param.ec_min_cluster_size, 
                                  param.ec_max_cluster_size);
}


/** @brief First extract dominant plane and do euclidean clustering, before starting model fitting **/
/** We assume that there is at least always one plane. **/
void ModelFitter::ProcessWithoutDP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  SetIndices(pcl_cloud->points.size());

  if(sac_models.size() == 0)
    sac_models.push_back(pcl::SACMODEL_PLANE);

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< std::vector<int> > pcl_clustered_indexes;

  if(!have_normals) 
  {
    pclA::PreProcessPointCloud(pcl_cloud, indexes, param.use_voxel_grid, param.voxel_grid_size);

    if(pclA::SingleSACModelSegmentation(pcl_cloud,
                                        indexes,
                                        pcl_model_clouds, 
                                        pcl_model_cloud_indexes,
                                        model_coefficients, 
                                        sac_models[0], 
                                        param.sac_optimal_distance, 
                                        param.sac_optimal_weight_factor, 
                                        param.sac_distance, 
                                        param.sac_max_iterations, 
                                        param.sac_min_inliers))
      pcl_model_types.push_back(sac_models[0]);
    
    if (pclA::EuclideanClustering(pcl_cloud, indexes, pcl_clustered_clouds, pcl_clustered_indexes, param.ec_cluster_tolerance, param.ec_min_cluster_size, param.ec_max_cluster_size))
    {
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++) 
      {
        pclA::FitMultipleModelRecursive(pcl_clustered_clouds[i], 
                                        pcl_clustered_indexes[i],
                                        normals,
                                        pcl_model_clouds, 
                                        pcl_model_cloud_indexes,
                                        model_coefficients, 
                                        sac_models, 
                                        pcl_model_types, 
                                        param.sac_optimal_distance,
                                        param.sac_optimal_weight_factor, 
                                        param.sac_distance, 
                                        param.sac_max_iterations,
                                        param.sac_min_inliers, 
                                        param.ec_cluster_tolerance, 
                                        param.ec_min_cluster_size, 
                                        param.ec_max_cluster_size);
      }
    }
  }
  
  else // have normals
  {
    std::vector< pcl::PointCloud<pcl::Normal>::Ptr > pcl_clustered_normals;
    pclA::PreProcessPointCloud(pcl_cloud, indexes, normals, param.use_voxel_grid, param.voxel_grid_size);
    if(pclA::SingleSACModelSegmentationWithNormals(pcl_cloud,
                                                   indexes,
                                                   normals,
                                                   pcl_model_clouds,
                                                   pcl_model_cloud_indexes,
                                                   model_coefficients, 
                                                   sac_models[0], 
                                                   param.sac_optimal_distance, 
                                                   param.sac_optimal_weight_factor, 
                                                   param.sac_distance, 
                                                   param.sac_max_iterations, 
                                                   param.sac_min_inliers))
      pcl_model_types.push_back(sac_models[0]);

    if (pclA::EuclideanClustering(pcl_cloud, indexes, pcl_clustered_clouds, pcl_clustered_indexes, normals, pcl_clustered_normals, param.ec_cluster_tolerance, param.ec_min_cluster_size, param.ec_max_cluster_size))
    {
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
      {
        pclA::FitMultipleModelRecursive(pcl_clustered_clouds[i], 
                                        pcl_clustered_indexes[i],
                                        pcl_clustered_normals[i],
                                        pcl_model_clouds, 
                                        pcl_model_cloud_indexes,
                                        model_coefficients, 
                                        sac_models, 
                                        pcl_model_types, 
                                        param.sac_optimal_distance, 
                                        param.sac_optimal_weight_factor, 
                                        param.sac_distance, 
                                        param.sac_max_iterations, 
                                        param.sac_min_inliers, 
                                        param.ec_cluster_tolerance, 
                                        param.ec_min_cluster_size, 
                                        param.ec_max_cluster_size);
      }
    }
  }
}


void ModelFitter::GetResults(std::vector<int> &_pcl_model_types,
                             std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &_pcl_model_clouds,
                             std::vector< std::vector<int> > &_pcl_model_cloud_indexes,
                             std::vector< pcl::ModelCoefficients::Ptr > &_model_coefficients)
{
  _pcl_model_types = pcl_model_types;
  _pcl_model_clouds = pcl_model_clouds;
  _pcl_model_cloud_indexes = pcl_model_cloud_indexes;
  _model_coefficients = model_coefficients;
}

}












