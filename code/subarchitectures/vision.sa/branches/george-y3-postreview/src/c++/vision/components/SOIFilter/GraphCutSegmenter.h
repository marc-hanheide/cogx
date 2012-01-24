/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * Changes:
 *   * May 2011 Marko Mahnic: Packed into a class
 */

#ifndef GRAPHCUTSEGMENTER_DQUT67JC
#define GRAPHCUTSEGMENTER_DQUT67JC

#include "GCoptimization.h"

#include <castutils/CastLoggerMixin.hpp>
#include <VisionData.hpp>
#include <PointCloud.hpp>
#include <PointCloudClient.h>
#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#define ID_OBJ_LAST_SEGMENTATION "soif.Last ROI Segmentation"
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/legacy/compat.hpp>

#include <vector>
#include <string>

namespace cast
{

class GraphCutSegmenter
  : public castutils::CCastLoggerMixin
{
  /**
  * Segmentation tolerances for distance and hsl
  *(gaussian dispersion)
  */
  float objHueTolerance;
  float objDistTolerance;
  float bgHueTolerance;
  float bgDistTolerance;
  int lblFixCost;
  int smoothCost;
  
  IplImage *colorFiltering; //HACK
  bool filterFlag; //HACK
  std::vector<CvScalar> filterList;

  int doDisplay;
public:
  PointCloudClient* pPcClient;
#ifdef FEAT_VISUALIZATION
  cogx::display::CDisplayClient* pDisplay;
#endif

public:
  GraphCutSegmenter();
  void configure(const std::map<std::string,std::string> & _config);

  /**
   * segment out object roi
   */
  bool segmentObject(const VisionData::SOIPtr soiPtr, Video::Image &imgPatch,
      VisionData::SegmentMask &segMask, std::vector<PointCloud::SurfacePoint> &segPoints,
      VisionData::ProtoObjectPtr& pProto);

private:
  void projectSOIPoints(const VisionData::SOI &soi, const VisionData::ROI &roi,
      std::vector<CvPoint> &projPoints, std::vector<CvPoint> &bgProjPoints,
      std::vector<int> &hull, const float ratio, const Video::CameraParameters &cam);

  void project3DPoints(const std::vector<PointCloud::SurfacePoint> surfPoints,
      const VisionData::ROI &roi, const float ratio, const Video::CameraParameters &cam,
      std::vector<CvPoint> &projPoints, std::vector<int> &hull);

  std::vector<PointCloud::SurfacePoint>  filter3DPoints(
      const std::vector<PointCloud::SurfacePoint> surfPoints,
      std::vector<CvPoint> &projPoints, std::vector<CvPoint> &errProjPoints,
      const VisionData::SegmentMask segMask);

  std::vector<PointCloud::SurfacePoint> sample3DPoints(
      std::vector<PointCloud::SurfacePoint> points, int newSize);

  void drawProjectedSOIPoints(IplImage *img, const std::vector<CvPoint> projPoints,
      const std::vector<CvPoint> bgProjPoints, const std::vector<CvPoint> errProjPoints,
      const std::vector<int> hull);

  void drawPoints(IplImage *img, const std::vector<CvPoint> projPoints);

  void drawHull(IplImage *img, const std::vector<CvPoint> projPoints,
      const std::vector<int> hull);

  std::vector<CvScalar> getSortedHlsList(std::vector<PointCloud::SurfacePoint> surfPoints);

  std::vector<unsigned char> graphCut(int width, int height, int num_labels,
      IplImage* costImg, IplImage* bgCostImg);

  int getHlsDiff(std::vector<CvScalar> hlsList, CvScalar hls, int k);

  std::vector<int> getHueDiffList(std::vector<CvScalar> hslList, int k);

  IplImage* getCostImage(IplImage *iplPatchHLS, std::vector<CvPoint> projPoints,
      std::vector<PointCloud::SurfacePoint> surfPoints, float hslmod,
      float distmod, bool distcost);

  std::vector<CvScalar> colorFilter(std::vector<CvScalar> colors,
      std::vector<CvScalar> filterColors, int k, int tolerance);
};

} // namespace

#endif /* end of include guard: GRAPHCUTSEGMENTER_DQUT67JC */

/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */


