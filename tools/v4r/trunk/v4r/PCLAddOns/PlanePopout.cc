/**
 * @file PlanePopout.cc
 * @author Prankl, Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Extract a dominant plane from kinect data and estimate popouts.
 * 
 * TODO SOI and ROI labels are not equal.
 */


#include "PlanePopout.hh"
#include "v4r/PCLAddOns/utils/PCLUtils.h"
#include "v4r/PCLAddOns/functions/PCLFunctions.h"

namespace pclA
{

  
/**************************************  SOI **********************************************/
PlanePopout::SOI::SOI(unsigned l, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &s)
{
  label = l;
  soi = s;

  convex_hull.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(unsigned i=0; i<soi->points.size()/2; i++)
    convex_hull->push_back(soi->points[i]);
}
  
unsigned PlanePopout::SOI::IsInSOI(const pcl::PointXYZRGB &p)
{
  if(pcl::isPointIn2DPolygon(p, *convex_hull))
    return label;
  return 0;
}

/************************************************************************************
 * Constructor/Destructor
 */

PlanePopout::PlanePopout(Parameter _param)
 : param(_param)
{
  zFilter.setFilterFieldName ("z"); 
  zFilter.setFilterLimits (param.minZ, param.maxZ);
  zFilter.setKeepOrganized(true);

  normalsTree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  clustersTree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  clustersTree->setEpsilon (1);

  grid.setLeafSize(param.downsampleLeaf, param.downsampleLeaf, param.downsampleLeaf);
  gridObjects.setLeafSize(param.downsampleLeafObjects, param.downsampleLeafObjects, param.downsampleLeafObjects);
  grid.setFilterFieldName ("z");
  grid.setFilterLimits (param.minZ, param.maxZ);
  grid.setDownsampleAllData (false);
  gridObjects.setDownsampleAllData (false);

  n3d.setKSearch (param.nbNeighbours);
  n3d.setSearchMethod (normalsTree);

  seg.setDistanceThreshold (param.thrSacDistance);
  seg.setMaxIterations (2000);

  seg.setNormalDistanceWeight (param.normalDistanceWeight);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setProbability (0.95);

  proj.setModelType (pcl::SACMODEL_NORMAL_PLANE);

  prism.setHeightLimits (param.minObjectHeight, param.maxObjectHeight);

  ccLabeling = new pclA::CCLabeling(pclA::CCLabeling::Parameter(param.thr,param.minClusterSize));

  
  // initialize variables
  valid_computation = false;
  input_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloudDownsampled.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloudNormals.reset (new pcl::PointCloud<pcl::Normal> ());
  tableInliers.reset (new pcl::PointIndices());
  tableCoefficients.reset (new pcl::ModelCoefficients());
  tableProjected.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  tableHull.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
}

PlanePopout::~PlanePopout()
{
}

// ================================= Public functions ================================= //
/**
 * Calculate Space of Interest (SOI) prisms as convex hulls. Processing:
 * - First filter incoming cloud in z-direction
 * - Detect the popout points of the point cloud after removing the dominant plane.
 * - Do euclidean clustering
 * - Project the clustered popouts to the table plane and estimate the convex hull
 * - Build from the convex hull (perpendicular) a soi prism with bottom/top points of 
 *   the convex hull.
 */
bool PlanePopout::CalculateSOIs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  sois.clear();
  pclA::CopyPointCloud(*cloud, *input_cloud);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters_projected;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_hulls;

  FilterZ(cloud, *cloudFiltered);                                           // Filter the PointCloud in z-coordinate
  if (DetectPopout(cloudFiltered, popouts) == false)	return false;
  
  pclA::EuclideanClustering(*cloudFiltered, popouts, clusters);             // Do euclidean clustering with all points of the popout-cloud
  for(unsigned i=0; i<clusters.size(); i++)
  {
    if(clusters[i]->points.size() > param.minClusterSize)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_hull;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr soi;

      double max_dist = 0;
      pclA::MaxDistanceToPlane(clusters[i], tableCoefficients, max_dist);     // Calculate the maximum distance to table plane

      pclA::GetConvexHull(clusters[i], tableCoefficients, cluster_hull);      // Project points on plane and get convex hull
      cluster_hulls.push_back(cluster_hull);

      pclA::CreateSOI(cluster_hull, tableCoefficients, max_dist, soi);        // Create SOI prism from convex hull and height
      SOI s(i+1, soi);
      sois.push_back(s);
    }
  }
  
  valid_computation = true;
  return true;
}

/**
 * Calculate the ROI mask with labels, starting with 1. 0 means no ROI.
 */
bool PlanePopout::CalculateROIMask()
{
  if(!valid_computation) return false;
  cv::Mat_<cv::Vec4f> matCloud;
  std::vector<unsigned> sizeClusters;
  
  ConvertPopout2Mat(*input_cloud, popouts, matCloud);
  LabelClusters(matCloud, roi_label_mask, sizeClusters);

  // Filter ROI label mask, concerning the minimum cluster size
  cv::Mat_<ushort> new_roi_label_mask;
  new_roi_label_mask = cv::Mat::zeros(roi_label_mask.rows, roi_label_mask.cols, CV_8U);
  std::vector<unsigned> new_sizeClusters;
  new_sizeClusters.push_back(0);
  ushort cluster_id = 1;
  
  for (int i = 0; i < sizeClusters.size(); i++)
  {
    if(sizeClusters[i] > param.minClusterSize)
    {
      new_sizeClusters.push_back(sizeClusters[i]);
      for (int v = 0; v < roi_label_mask.rows; ++v)
      {
        for (int u = 0; u < roi_label_mask.cols; ++u)
        {
          const ushort &la = roi_label_mask(v,u);
          if (la == i)
            new_roi_label_mask(v,u) = cluster_id;
        }
      }
      cluster_id++;
    }
  }
  roi_label_mask = new_roi_label_mask;
  sizeClusters = new_sizeClusters;
  return true;
}


/**
 * Check, if image coordinate point is in a ROI.
 */
ushort PlanePopout::IsInROI(int x, int y)
{
  return roi_label_mask(y, x);
}

void PlanePopout::GetSOIs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &_sois)
{
  for(unsigned i=0; i<sois.size(); i++)
    _sois.push_back(sois[i].GetSoi());
}

/**
 * Check, if point is in SOI. Returns the cluster label for that point.
 * Returns zero, if point is "under" the dominant plane (view dependent!)
 */ 
unsigned PlanePopout::IsInSOI(float x, float y, float z)
{
  /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);*/

  pcl::PointXYZRGB p;
  p.x = x;
  p.y = y;
  p.z = z;
  /*pcl::PointXYZRGB pn, pp;
  pcl_cloud->push_back(p);
  pclA::GetProjectedPoints(pcl_cloud, tableCoefficients, pcl_cloud_projected);
  pn = pcl_cloud_projected->points[0];*/
  
  unsigned label = 0;
  for(unsigned i=0; i<sois.size(); i++)
  {
    unsigned newLabel = sois[i].IsInSOI(p);
    if(newLabel > 0) 
    {
      /*// check, if point is "under" table (depends on view!!!)
      pp.x = pn.x - p.x;
      pp.y = pn.y - p.y;
      pp.z = pn.z - p.z;
      double norm_p = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      double norm_pp = sqrt(pp.x*pp.x + pp.y*pp.y + pp.z*pp.z);
      double dot_p_pp = p.x*pp.x + p.y*pp.y + p.z*pp.z;
      double angle = acos(dot_p_pp/(norm_p*norm_pp));
      if(angle < 1.57)  // Pi/2
        label = newLabel;*/
      label = newLabel;
    }
  }
  return label;
}

/**
 * Detect table plane and objects which pop out
 */
bool PlanePopout::CollectTableInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::ModelCoefficients::Ptr dpc)
{
//     if (dpc->values[0]!=0 && dpc->values[1]!=0 && dpc->values[2]!=0 && dpc->values[3]!=0)
//     {
// cout<<"PlanePopout::DetectPopout: dpc= (" << dpc->values[0] << ", "<<dpc->values[1]<<", "<<","<<dpc->values[2]<<","<<dpc->values[3]<<")"<<endl;
	tableInliers.reset (new pcl::PointIndices());
	for (unsigned i=0; i<cloud->points.size(); i++)
	{
	  float dist=abs(dpc->values[0]*cloud->points[i].x
			+dpc->values[1]*cloud->points[i].y
			+dpc->values[2]*cloud->points[i].z
			+dpc->values[3])
	  /sqrt(dpc->values[0]*dpc->values[0]
	       +dpc->values[1]*dpc->values[1]
	       +dpc->values[2]*dpc->values[2]);
	  if (dist < param.thr)	(*tableInliers).indices.push_back(i);
	}
// cout<<"Done the collection of inliers"<<endl;
//     }
}

/**
 * Detect table plane and objects which pop out
 */
bool PlanePopout::DetectPopout(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                               pcl::PointIndices &popout)
{
  if ((int)cloud->points.size () < param.nbNeighbours)
    return false;
  //	else cout<<"PlanePopout::DetectPopout: Only " << (int)cloud->points.size () << " are available!"<<endl;

  if (tableCoefficients.get()==0)
    tableCoefficients.reset(new pcl::ModelCoefficients());
  
  grid.setInputCloud (cloud);
  grid.filter (*cloudDownsampled);

  n3d.setInputCloud (cloudDownsampled);
  n3d.compute (*cloudNormals);

  seg.setInputCloud (cloudDownsampled);
  seg.setInputNormals (cloudNormals);
  seg.segment (*tableInliers, *tableCoefficients);

  if (tableInliers->indices.size () == 0)
  {
//     cout<<"PlanePopout::DetectPopout: No Plane Inliers points!"<<endl;
    return false;
  }
//   else
//   {
//       for (int i=0; i<tableInliers->indices.size(); i++)
// 	cout<<"PlanePopout::DetectPopout: there is a point at ( "<<cloud->points.at(tableInliers->indices.at(i)).x <<" ,"<<cloud->points.at(tableInliers->indices.at(i)).y<<" ,"<<cloud->points.at(tableInliers->indices.at(i)).z<<" )"<<endl;
//   }
  //else	cout<<"PlanePopout::DetectPopout: There are "<< tableInliers->indices.size()<<"Plane Inliers points!"<<endl;

  proj.setInputCloud (cloudDownsampled);
  proj.setIndices (tableInliers);
  proj.setModelCoefficients (tableCoefficients);
  proj.filter (*tableProjected); 

  hull.setInputCloud (tableProjected);
  hull.reconstruct (*tableHull);

  prism.setInputCloud (cloud);
  prism.setInputPlanarHull (tableHull);
  prism.segment (popout);

  if (popout.indices.size () == 0)
  {
//     cout<<"PlanePopout::DetectPopout: No popout points!"<<endl;
    return false;
  }
  
  return true;
}

/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const cv::Mat_<cv::Vec4f> &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  if (pclCloud.get()==0) pclCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  pclA::ConvertCvMat2PCLCloud(cloud, *pclCloud);

  zFilter.setInputCloud (pclCloud);
  zFilter.filter(filtered);
}


/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  zFilter.setInputCloud (cloud);
  zFilter.filter(filtered);
}

/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  zFilter.setInputCloud (cloud);
  zFilter.filter(filtered);
}

/**
 * Convert point indices from plane popout to a cv::Mat
 */
void PlanePopout::ConvertPopout2Mat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                    const pcl::PointIndices &popout, 
                                    cv::Mat_<cv::Vec4f> &matCloud)
{
  matCloud = cv::Mat_<cv::Vec4f>(cloud.height, cloud.width);
  float bad_point =  std::numeric_limits<float>::quiet_NaN ();

  for (int v = 0; v < matCloud.rows; ++v)
  {
    for (int u = 0; u < matCloud.cols; ++u)
    {
      cv::Vec4f &matpt = matCloud(v,u);
      matpt[0] = bad_point;
      matpt[1] = bad_point;
      matpt[2] = bad_point;
      matpt[3] = bad_point;
    }
  }

  unsigned u, v;
  for (size_t i = 0; i < popout.indices.size(); ++i)
  {
    u = popout.indices[i]%cloud.width;
    v = popout.indices[i]/cloud.width;

    const pcl::PointXYZRGB &pt = cloud(u, v);
    cv::Vec4f &matpt = matCloud(v,u);

    matpt[0] = pt.x;
    matpt[1] = pt.y;
    matpt[2] = pt.z;
    matpt[3] = pt.rgb;
  }
}

/**
 * Euclidean neighbourhood clustering
 * using a connected component analysis of the point cloud grid
 */
void PlanePopout::LabelClusters(const cv::Mat_<cv::Vec4f> &cloud, 
                                cv::Mat_<ushort> &labels, 
                                vector<unsigned> &sizeClusters)
{
  ccLabeling->Operate(cloud, labels, sizeClusters);
}

/**
 * Create a mask from all selected labels
 */
void PlanePopout::CreateMaskAll(const cv::Mat_<ushort> &labels, 
                                const vector<unsigned> &sizeClusters, 
                                cv::Mat_<uchar> &mask)
{
  ccLabeling->CreateMask(labels, sizeClusters, mask);
}


/**
 * Create mask from clusters with id
 */
void PlanePopout::CreateMaskId(const cv::Mat_<ushort> &labels, const ushort id, cv::Mat_<uchar> &mask)
{
  mask = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);
  
  //filter largest cluster
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      if (labels(v,u)==id)
      {
        mask(v,u) = 255;
      }
    }
  }
}

/**
 * Create mask from largest cluster
 */
void PlanePopout::CreateMaskLargest(const cv::Mat_<ushort> &labels, 
           const vector<unsigned> &sizeClusters, cv::Mat_<uchar> &mask)
{
  ushort maxLabel;
  unsigned max=0;

  for (unsigned i=1; i<sizeClusters.size(); i++)
  {
    if (max<sizeClusters[i])
    {
      max=sizeClusters[i];
      maxLabel=i;
    }
  }

  if (max>0)
    CreateMaskId(labels, maxLabel, mask);
  else
    mask = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);
}

// ================================= Private functions ================================= //


}












