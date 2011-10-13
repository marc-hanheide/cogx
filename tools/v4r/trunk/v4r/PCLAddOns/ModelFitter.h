/**
 * @file ModelFitter.h
 * @author Richtsfeld
 * @date October 2011
 * @version 0.1
 * @brief Extract shape models (plane, cylinder, sphere) from kinect data.
 */

#ifndef PCLA_MODEL_FITTER_HH
#define PCLA_MODEL_FITTER_HH

#include <vector>
#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/features/normal_3d.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/organized_data.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"

#include "v4r/PCLAddOns/functions/CCLabeling.hh"

namespace pclA
{

class ModelFitter
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
  class Parameter
  {
  public:
    bool use_voxel_grid;                // vg: use the voxel grid before fitting
    double voxel_grid_size;             // vg: voxel grid size
    bool sac_optimal_distance;          // sac: use optimal distance
    double sac_optimal_weight_factor;   // sac: optimal weight factor
    double sac_distance;                // sac: distance
    int sac_max_iterations;             // sac: maximum iterations
    int sac_min_inliers;                // sac: minimum point inliers
    double ec_cluster_tolerance;        // ec: cluster tolerance
    int ec_min_cluster_size;            // ec: minimal cluster size
    int ec_max_cluster_size;            // ec: maximum cluster size

    Parameter(bool _use_voxel_grid = false,
              double _voxel_grid_size = 0.005,           // 0.005 - 0.01
              bool _sac_optimal_distance = true,
              double _sac_optimal_weight_factor = 2.0,
              double _sac_distance = 0.004,
              int _sac_max_iterations = 250,
              int _sac_min_inliers = 25,
              double _ec_cluster_tolerance = 0.015,
              int _ec_min_cluster_size = 25,
              int _ec_max_cluster_size = 1000000)
      : use_voxel_grid(_use_voxel_grid), voxel_grid_size(_voxel_grid_size), sac_optimal_distance(_sac_optimal_distance),
        sac_optimal_weight_factor(_sac_optimal_weight_factor), sac_distance(_sac_distance),
        sac_max_iterations(_sac_max_iterations), sac_min_inliers(_sac_min_inliers), ec_cluster_tolerance(_ec_cluster_tolerance), 
        ec_min_cluster_size(_ec_min_cluster_size),  ec_max_cluster_size(_ec_max_cluster_size) {}
  };

  
protected:
  Parameter param;

private:
  std::vector<int> indexes;             // index of the pcl_cloud

  bool have_normals;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  // models to fit
  std::vector<int> sac_models;

  // Results: Type of models, model point cloud and model coefficients
  std::vector<int> pcl_model_types;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< std::vector<int> > pcl_model_cloud_indexes;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  

  void SetIndices(int cloud_size);

public:
  ModelFitter(Parameter _param = Parameter());
  ~ModelFitter();
  
  /** Add (pcl) model for fitting **/
  void AddModelType(int model_type);
  
  /** Set normal point cloud **/
  void SetNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  
  /** Process the point cloud and fit (multiple) models **/
  void Process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);
  
  /** Process the point cloud and fit (multiple) models, after dominant plane extraction **/
  void ProcessWithoutDP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Get the results from the processing **/
  void GetResults(std::vector<int> &_pcl_model_types,
                  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcl_model_clouds,
                  std::vector< std::vector<int> > &_pcl_model_cloud_indexes,
                  std::vector<pcl::ModelCoefficients::Ptr> &_model_coefficients);
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

