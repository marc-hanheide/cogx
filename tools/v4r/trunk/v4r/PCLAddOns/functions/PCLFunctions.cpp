/**
 * @file PCLFunctions.hh
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#include "PCLFunctions.h"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
 
namespace pclA
{
  
bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,                 /// TODO Rename it to voxel grid irgendwas
                          bool useVoxelGrid,
                          double vg_size)
{
//   pclA::RemoveZeros(pcl_cloud);  // TODO not neccessary

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
    pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);
  
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
    // get cloud and delete points from pcl_cloud
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    pclA::GetOutlierCloud(*pcl_cloud, inliers);

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
    pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, sac_distance, sac_optimal_weight_factor);

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
    pcl_plane_clouds.resize(pcl_plane_clouds.size() + 1);
    pcl_plane_clouds[pcl_plane_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_cloud, inliers, *pcl_plane_clouds[pcl_plane_clouds.size()-1]);
    model_coefficients.push_back(model_coef);

    pclA::GetOutlierCloud(*pcl_cloud, inliers);

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
    pcl_cluster_clouds[idx].reset(new pcl::PointCloud<pcl::PointXYZRGB>); // = tmp_cloud;
    pcl::copyPointCloud (*pcl_cloud, indices[idx], *pcl_cluster_clouds[idx]);
  }  
  
  if(indices.size() > 0) return true;
  return false;
} 


void EuclideanClustering(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,                          /// TODO weg damit => pop_out_cloud seperat berechnen
                         const pcl::PointIndices popouts,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         double cluster_tolerance,
                         double min_cluster_size,
                         double max_cluster_size)
{
  pcl::PointCloud<pcl::PointXYZRGB> pop_out_cloud;
  pcl::copyPointCloud (pcl_cloud, popouts, pop_out_cloud);
  
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
  tree->setInputCloud(pop_out_cloud.makeShared());

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> indices;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud(pop_out_cloud.makeShared());
  ec.extract(indices);
  
  pcl_cluster_clouds.resize(pcl_cluster_clouds.size() + indices.size());
  for(unsigned idx=0; idx<indices.size(); idx++)
  {
    pcl_cluster_clouds[idx].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud (pop_out_cloud, indices[idx], *pcl_cluster_clouds[idx]);
  }  
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
  chull.setKeepInformation(false);                            // TODO keep information about point (color)!
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*pcl_convex_hull);
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


bool SOISegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
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
  printf("PCLFunctions::SOISegmentation: Antiquated!\n");
//   std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
//   if(SingleSACSegmentation(pcl_cloud, pcl_plane_clouds, model_coefficients, sac_optimal_distance, sac_optimal_weight_threshold, sac_distance, sac_max_iterations, sac_min_inliers))
//   {
//     if(EuclideanClustering(pcl_cloud, pcl_clustered_clouds, ec_cluster_tolerance, ec_min_cluster_size, ec_max_cluster_size))
//     {
// printf("SOISegmentation: we have now a lot of clusters? %u\n", pcl_clustered_clouds.size()); 
// 
//       /// TODO Project clusters on the dominant plane and check if all points are inside of the convex hull of the dominant plane => New SOI found
//       std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clouds_projected;
//       for(unsigned i=0; i<pcl_clustered_clouds.size(); i++)
//       {
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_projected;
//         GetProjectedPoints(pcl_clustered_clouds[i], model_coefficients[0], pcl_cloud_projected);                   /// TODO TODO TODO Diese Funktion funktioniert nicht!!!
// 
// pclA::PrintPCLCloud(*pcl_cloud_projected);
// 
// //         pcl_clouds_projected.push_back(pcl_cloud_projected);
//        
//       }
//     }
//   }
}


void Mask2SOI(const cv::Mat_<ushort> &labels, 
                           const std::vector<unsigned> &sizeClusters, 
                           const cv::Mat_<uchar> &_mask,
                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &sois,
                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &soiHulls)
{
  printf("PCLFunctions::Mask2SOI: Antiquated!\n");
  
//   // initialize new sois and soiHulls
//   sois.resize(sois.size() + 1);
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_sois (new pcl::PointCloud<pcl::PointXYZRGB>);
//   sois[sois.size()-1] = tmp_sois;
//   soiHulls.resize(soiHulls.size() + 1);
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_soiHulls (new pcl::PointCloud<pcl::PointXYZRGB>);
//   soiHulls[soiHulls.size()-1] = tmp_soiHulls;
  
//   pcl::copyPointCloud(*inlierCloud, *sois[sois.size()-1]);
//   pcl::copyPointCloud(*inlierCloud, *soiHulls[soiHulls.size()-1]);
}

void MaxDistanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        const pcl::ModelCoefficients::Ptr &model_coefficients,
                        double &max_distance)
{
  max_distance = 0.;
  for(unsigned i=0; i<pcl_cloud->points.size(); i++)
  {
    double dist = pcl::pointToPlaneDistanceSigned(pcl_cloud->points[i], model_coefficients->values[0], model_coefficients->values[1], 
                                                  model_coefficients->values[2], model_coefficients->values[3]);
                                                  
    if(dist > max_distance)
      max_distance = dist;
  }
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

  for(unsigned i=0; i<pcl_cloud->points.size(); i++)  
    soi->points.push_back(pcl_cloud->points[i]);
  
  for(unsigned i=0; i<pcl_cloud->points.size(); i++)
  {
    pcl::PointXYZRGB p;
    p.x = pcl_cloud->points[i].x + vector[0];
    p.y = pcl_cloud->points[i].y + vector[1];
    p.z = pcl_cloud->points[i].z + vector[2];
    soi->points.push_back(p);
  }
}

}










