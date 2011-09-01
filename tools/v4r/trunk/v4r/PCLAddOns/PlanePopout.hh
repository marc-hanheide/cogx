/**
 * @file PlanePopout.hh
 * @author Prankl, Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Extract a dominant plane from kinect data and estimate popouts.
 */

#ifndef PCLA_PLANE_POPOUT_HH
#define PCLA_PLANE_POPOUT_HH

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

class PlanePopout
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
  class Parameter
  {
  public:
    double minZ, maxZ;                // filter points depending on z value (default = 0.5m - 1.5m)
    double downsampleLeaf;            // Voxel size (default = 0.01m)
    double downsampleLeafObjects;     // Voxel size for objects (default = 0.01m)
    int nbNeighbours;                 // Neighbors for normal calculation (default = 10)
    double thrSacDistance;            // SAC segmentation distance (default = 0.01m)
    double normalDistanceWeight;      // Distance weight for normals (default = 0.1m)
    double minObjectHeight;           // Minimum object height (default = 0.005m)
    double maxObjectHeight;           // Maximum object height (default = 0.7m)
    float thr;                        // Minimum euc. distance to connect points (default = 0.02m)
    unsigned minClusterSize;          // Minimum cluster size for CCLabeling (default = 100)

    Parameter(double _minZ = 0.0,
              double _maxZ = 2.0,
              double dwLeaf = 0.02,
              double dwLeafObj = 0.01,
              int nb = 10,
              double thrSac = 0.01,
              double normalDistWeight = 0.1,
              double minObjH = 0.005,
              double maxObjH = 0.7,
              float eucThr = 0.02,
              unsigned minClSize = 100)
      : minZ(_minZ), maxZ(_maxZ), downsampleLeaf(dwLeaf), downsampleLeafObjects(dwLeafObj), 
        nbNeighbours(nb),thrSacDistance(thrSac), normalDistanceWeight(normalDistWeight),
        minObjectHeight(minObjH), maxObjectHeight(maxObjH), thr(eucThr), minClusterSize(minClSize) {}
  };

  
  class SOI
  {
  private:
    unsigned label;                                       // Label of the SOI
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr soi;           // Convex hull prisms of SOI
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convex_hull;   // Convex hull on table plane

  public:
    SOI(unsigned l, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &s);
        
    unsigned IsInSOI(const pcl::PointXYZRGB &p);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetSoi() {return soi;}
  };
  
protected:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;               ///< The original input cloud
  pcl::ModelCoefficients::Ptr tableCoefficients;                    ///< Coefficients of the dominant plane (table)
  cv::Mat_<ushort> roi_label_mask;                                  ///< Mask of the rois with labels!
  std::vector<SOI> sois;                                            ///< vector with stored SOIs

private:
  Parameter param;

  bool valid_computation;                                           ///< Set to true, after first calculation!

  pcl::PointIndices popouts;                                        ///< Popout indices (unclustered)

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled;
  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals;
  pcl::PointIndices::Ptr tableInliers;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableProjected;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableHull;

  pcl::PassThrough<pcl::PointXYZRGB> zFilter;
  pcl::KdTree<pcl::PointXYZRGB>::Ptr normalsTree, clustersTree;
  pcl::VoxelGrid<pcl::PointXYZRGB> grid, gridObjects;
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n3d;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud;

  cv::Ptr<pclA::CCLabeling> ccLabeling;

public:

  PlanePopout(Parameter _param = Parameter());
  ~PlanePopout();

  void CalculateSOIs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  bool CalculateROIMask();

  ushort IsInROI(int x, int y);

  void GetSOIs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &_sois);

  unsigned IsInSOI(float x, float y, float z);
  unsigned IsInSOI(const cv::Point3f &p) {return IsInSOI(p.x, p.y, p.z);}

  void GetDominantPlaneCoefficients(pcl::ModelCoefficients::Ptr &dpc) {dpc = tableCoefficients;}

  void DetectPopout(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                    pcl::PointIndices &popout);

  void FilterZ(const cv::Mat_<cv::Vec4f> &cloud,                                                                /// TODO Move to pcl functions!!!
               pcl::PointCloud<pcl::PointXYZRGB> &filtered);
  void FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
               pcl::PointCloud<pcl::PointXYZRGB> &filtered);
  void FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
               pcl::PointCloud<pcl::PointXYZRGB> &filtered);
               
  void ConvertPopout2Mat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud,                                        /// TODO Auch abgeschlossene Funktion!!!
                         const pcl::PointIndices &popout,
                         cv::Mat_<cv::Vec4f> &matCloud);
                         
  void LabelClusters(const cv::Mat_<cv::Vec4f> &cloud, 
                     cv::Mat_<ushort> &labels,
                     vector<unsigned> &sizeClusters);
  void CreateMaskAll(const cv::Mat_<ushort> &labels,
                     const vector<unsigned> &sizeClusters,
                     cv::Mat_<uchar> &mask);
  void CreateMaskLargest(const cv::Mat_<ushort> &labels,
                         const vector<unsigned> &sizeClusters,
                         cv::Mat_<uchar> &mask);
  void CreateMaskId(const cv::Mat_<ushort> &labels,
                    const ushort id,
                    cv::Mat_<uchar> &mask);
                    
private:

  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

