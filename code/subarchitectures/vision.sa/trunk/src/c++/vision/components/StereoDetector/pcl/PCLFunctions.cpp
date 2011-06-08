/**
 * @file PCLFunctions.hh
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#include "PCLFunctions.h"
 
namespace pclF
{
  
bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
                          bool useVoxelGrid,
                          double vg_size)
{
  pclU::RemoveZeros(pcl_cloud);

  if(useVoxelGrid)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    vg.setInputCloud(pcl_cloud.makeShared());
    vg.setLeafSize (vg_size, vg_size, vg_size);
    vg.filter(cloud_filtered);
    pcl_cloud = cloud_filtered;
  }
  return true;
}


bool FitPlanesRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                        std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
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
  if(SingleSACSegmentation(pcl_cloud, pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations, sac_min_inliers))
  {
    if(EuclideanClustering(pcl_cloud, pcl_clustered_clouds, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
      for(unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
        FitPlanesRecursive(pcl_clustered_clouds[i], pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations,
                           sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
  }
  return true;
}


bool FitPlanesRecursiveWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                   std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
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
  if(SingleSACSegmentationWithNormals(pcl_cloud, pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations, sac_min_inliers))
  {
    if(EuclideanClustering(pcl_cloud, pcl_clustered_clouds, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
      for(unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
        FitPlanesRecursiveWithNormals(pcl_clustered_clouds[i], pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations,
                           sac_min_inliers, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size);
  }
  return true;
}
  
bool SingleSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                           bool calc_optimal_sac_distance,
                           double sac_optimal_weight_factor,
                           double sac_distance, 
                           int maxIterations,
                           int minInliers)
{
  if(calc_optimal_sac_distance) 
    pclU::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);
  
  bool succeed = false;
  pcl::PointIndices inliers;
  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.segment(inliers, *model_coef);

  if(inliers.indices.size() >= minInliers)
  {
    // copy inlier cloud and delete points from pcl_cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pclU::GetInlierCloud(*pcl_cloud, inliers, *inlierCloud);
    pclU::GetOutlierCloud(*pcl_cloud, inliers);

    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_plane_clouds[pcl_plane_clouds.size()-1] = tmp_cloud;
    pcl::copyPointCloud(*inlierCloud, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    succeed = true;
  }

  if(pcl_cloud->points.size() > minInliers && succeed) return true;
  return false;
}
  
  
bool SingleSACSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                      std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                      std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                      bool calc_optimal_sac_distance,
                                      double sac_optimal_weight_factor,
                                      double sac_distance, 
                                      int maxIterations,
                                      int minInliers)
{
  if(calc_optimal_sac_distance) 
    pclU::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

  // Create normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());

  ne.setInputCloud(pcl_cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  ne.compute(cloud_normals);

  bool succeed = false;
  pcl::PointIndices inliers;
  pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);
  
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(sac_distance);
  seg.setMaxIterations(maxIterations);
  seg.setInputCloud(pcl_cloud);
  seg.setInputNormals(cloud_normals.makeShared());
  seg.segment(inliers, *model_coef);

  if(inliers.indices.size() >= minInliers)
  {
    // copy inlier cloud and delete points from pcl_cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pclU::GetInlierCloud(*pcl_cloud, inliers, *inlierCloud);
    pclU::GetOutlierCloud(*pcl_cloud, inliers);

    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_plane_clouds[pcl_plane_clouds.size()-1] = tmp_cloud;
    pcl::copyPointCloud(*inlierCloud, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    succeed = true;
  }

  if(pcl_cloud->points.size() > minInliers && succeed) return true;
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
  std::vector<pcl::PointIndices> indices;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud(pcl_cloud);
  ec.extract(indices);
  
  pcl_cluster_clouds.resize(pcl_cluster_clouds.size() + indices.size());
  for(unsigned idx=0; idx<indices.size(); idx++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cluster_clouds[idx] = tmp_cloud;
    pcl::copyPointCloud (*pcl_cloud, indices[idx], *pcl_cluster_clouds[idx]);
  }  
  
  if(indices.size() > 0) return true;
  return false;
} 


bool GetConvexHulls(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                    std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_convex_hulls)

{
  if(pcl_clouds.size() != model_coefficients.size())
  {
    printf("PCLFunctions::GetConvexHulls: Warning: Cloud and coefficient size do not match!\n");
    return false;
  }
  
  for(unsigned i=0; i<pcl_clouds.size(); i++)
  {
    // Project the model inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(pcl_clouds[i]);
    proj.setModelCoefficients(model_coefficients[i]);
    proj.filter(*cloud_projected);
    
    // return the projected point clouds    
    pcl_clouds[i] = cloud_projected;

    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    chull.setKeepInformation(true);   // keep information about point (color)!
    chull.setInputCloud (cloud_projected);
    chull.reconstruct (*cloud_hull);

    pcl_convex_hulls.push_back(cloud_hull);
  }
}


bool GetProjectedPoints(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds_projected)

{
  if(pcl_clouds.size() != model_coefficients.size())
  {
    printf("PCLFunctions::GetConvexHulls: Warning: Cloud and coefficient size do not match!\n");
    return false;
  }
  
  for(unsigned i=0; i<pcl_clouds.size(); i++)
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










/*  
bool ReverseSACSegmentation(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                            std::vector< cv::Mat_<cv::Vec4f> > &cvClouds,
                            double distanceTH)
{
  bool segment = true;
  
  Prepare SAC-Segmentation
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distanceTH);
  seg.setMaxIterations(250);                        /// TODO TODO Arbitrary value
  
  while(segment)
  {
    SEGMENT cloud
    seg.setInputCloud(cloud.makeShared());
    pcl::PointIndices inliers;
    pcl::ModelCoefficients model_coefficients;
    seg.segment(inliers, model_coefficients);
    
    check if we still have valid results
    if(inliers.indices.size() <= 100) segment = false;
    
    converse and add to cvClouds
    if(segment)
    {
printf("got another result: inliers.indices.size(): %u\n", inliers.indices.size());
      
      delete points from cloud
      pcl::PointCloud<pcl::PointXYZRGB> inlierCloud;
      pcl::PointCloud<pcl::PointXYZRGB> outlierCloud;

      pclU::GetInlierCloud(cloud, inliers, inlierCloud);
      pclU::GetOutlierCloud(cloud, inliers, outlierCloud);

printf("        in/outlier cloud size: %u - %u\n", inlierCloud.points.size(), outlierCloud.points.size());
      
      Convert plane to openCV style
      cv::Mat_<cv::Vec4f> cvCloud;
      pclU::PCLCloud2CvCloud(inlierCloud, cvCloud);
      cvClouds.push_back(cvCloud);

      cloud = outlierCloud;
    }
  }

  if(cvClouds.size() > 0) return true;
  return false; 
}
  
  
  
bool FitPlanesIncremental(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
               std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
               double cluster_tolerance, 
               int min_cluster_size,
               int max_cluster_size)
{
  bool finished = false;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clustered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_sac_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector< cv::Mat_<cv::Vec4f> > single_clustered_clouds;
  
  pcl_sac_cloud = pcl_cloud.makeShared();

  printf("  SingleSACSegmentation: before: pcl_cluster_cloud.points: %u\n", pcl_sac_cloud->points.size());
  SingleSACSegmentation(pcl_sac_cloud, clustered_clouds, 0.008, 100, 50);
  printf("  SingleSACSegmentation:  after: pcl_cluster_cloud.points: %u\n", pcl_sac_cloud->points.size());

  bool save = false;
  while(!finished) // TODO Arbitrary
  {
printf("SingleEuclideanClustering: start while!\n");
    if(SingleEuclideanCluster(pcl_sac_cloud, pcl_clustered_cloud))
    {
printf("SingleEuclideanClustering: success!\n");
      if(!save)
      {
        saveCloud = pcl_sac_cloud;            // save rest of cloud
printf("SingleEuclideanClustering: save pcl_sac_cloud to saveCloud: %u\n", saveCloud->points.size());
        save = true;
      }
      pcl_sac_cloud = pcl_clustered_cloud;
printf("SingleEuclideanClustering: new pcl_sac_cloud: %u\n", pcl_sac_cloud->points.size());
    }
    else
    {
printf("SingleEuclideanClustering: FAILED!\n");
      if(!save)
      {
        save = false;
        pcl_sac_cloud = saveCloud;
      }
      else finished = true;

      printf("SingleEuclideanClustering: failed!\n");
      printf("SingleEuclideanClustering: finished: clouds: %u\n", pcl_clustered_cloud->points.size());
    }
    
printf("  SingleSACSegmentation: before: pcl_cluster_cloud.points: %u\n", pcl_sac_cloud->points.size());
    SingleSACSegmentation(pcl_sac_cloud, clustered_clouds, 0.008, 100, 50);
printf("  SingleSACSegmentation:  after: pcl_cluster_cloud.points: %u\n", pcl_sac_cloud->points.size());
  }
  return true;
}

 
bool FitPlanesAfterDominantPlaneExtraction(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
               std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
               double cluster_tolerance, 
               int min_cluster_size,
               int max_cluster_size)
{
  SingleSACSegmentation(pcl_cloud.makeShared(), clustered_clouds);
  
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< cv::Mat_<cv::Vec4f> > single_clustered_clouds;
  
  if(EuclideanClustering(pcl_cloud.makeShared(), pcl_clustered_clouds, cluster_tolerance, min_cluster_size, max_cluster_size))
  {
printf("Euclidean clustering successful: %u clusters found!\n", pcl_clustered_clouds.size());
    for(unsigned i=0; i < pcl_clustered_clouds.size(); i++)
    {
      if(ReverseSACSegmentation(*pcl_clustered_clouds[i], clustered_clouds, cluster_tolerance))
      {
printf(" HUHU: SAC successful: %u clusters found!\n", clustered_clouds.size());
      }
    }
  }
  else return false;
  return true;
}


bool FitPlanes(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
               std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
               double cluster_tolerance, 
               int min_cluster_size,
               int max_cluster_size)
{
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< cv::Mat_<cv::Vec4f> > single_clustered_clouds;

  if(EuclideanClustering(pcl_cloud.makeShared(), pcl_clustered_clouds, cluster_tolerance, min_cluster_size, max_cluster_size))
  {
printf("Euclidean clustering successful: %u clusters found!\n", pcl_clustered_clouds.size());
    for(unsigned i=0; i < pcl_clustered_clouds.size(); i++)
    {
      if(ReverseSACSegmentation(*pcl_clustered_clouds[i], clustered_clouds, cluster_tolerance))
      {
printf(" HUHU: SAC successful: %u clusters found!\n", clustered_clouds.size());
      }
    }
  }
  else return false;
  return true;
}      

  
bool FitPlanesWithNormals(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
                          std::vector< cv::Mat_<cv::Vec4f> > &clustered_clouds,
                          double cluster_tolerance, 
                          int min_cluster_size,
                          int max_cluster_size)
{
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< cv::Mat_<cv::Vec4f> > single_clustered_clouds;

  if(EuclideanClustering(pcl_cloud.makeShared(), pcl_clustered_clouds, cluster_tolerance, min_cluster_size, max_cluster_size))
  {
printf("Euclidean clustering successful: %u clusters found!\n", pcl_clustered_clouds.size());
    for(unsigned i=0; i < pcl_clustered_clouds.size(); i++)
    {
      if(ReverseSACSegmentationFromNormals(*pcl_clustered_clouds[i], clustered_clouds, cluster_tolerance))
      {
printf(" HUHU: SAC successful: %u clusters found!\n", clustered_clouds.size());
      }
    }
  }
  else return false;
  return true;
}      
  
bool SingleEuclideanCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cluster_cloud,
                               double cluster_tolerance, 
                               double min_cluster_size,
                               double max_cluster_size)
{
printf("SingleEuclideanClustering: start! pcl_cloud.points.size: %u\n", pcl_cloud->points.size());
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01, 0.01, 0.01);      // x,y,z size
  vg.filter (*cloud_filtered);
  
printf("SingleEuclideanClustering: start! cloud_filtered->points.size: %u\n", cloud_filtered->points.size());

  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud_filtered);
  tree->setInputCloud(pcl_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> indices;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud(cloud_filtered);
  ec.setInputCloud(pcl_cloud);     /// TODO 
  ec.extract(indices);

  copy the clusters to new point clouds
  if(indices.size() > 0)
  {
printf("SingleEuclideanClustering: Loop 1!\n");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cluster_cloud = tmp_cloud;
    

    pcl::copyPointCloud(*pcl_cloud, indices[0], *pcl_cluster_cloud);
    pcl::copyPointCloud(*pcl_cloud, *cloud_filtered);
    pclU::GetInlierCloud(*pcl_cloud, indices[0], *pcl_cluster_cloud);
    pclU::GetOutlierCloud(*pcl_cloud, indices[0]);
printf("SingleEuclideanClustering: GetOutlierCloud: cloud_filtered: %u\n", cloud_filtered->points.size());

    pcl_cloud = cloud_filtered;
    pcl::copyPointCloud(*cloud_filtered, *pcl_cloud);

printf("SingleEuclideanClustering: 3: pcl_cloud->points.size: %u\n", pcl_cloud->points.size());

    return true;
  }
  else return false;
  
printf("SingleEuclideanClustering: 2: indices.size: %u\n", indices.size());
  
//   pcl_cluster_clouds.resize(indices.size());
//   for(unsigned idx=0; idx<indices.size(); idx++)
//   {
// printf("SingleEuclideanClustering: WE ARE INSIDE OF THE LOOP!\n");
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//     pcl_cluster_clouds[idx] = tmp_cloud;
//     
//     pcl::copyPointCloud(*cloud_filtered, indices[idx], *pcl_cluster_cloud);
//   }
//   
//   if(indices.size() > 0) return true;
//   return false;
}*/

}
