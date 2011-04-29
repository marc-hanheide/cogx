/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * A component the filters out persistent SOIs.
 */

#ifndef SOI_FILTER_H
#define SOI_FILTER_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "GCoptimization.h"
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <StereoClient.h>
#include <../../VisionUtils.h>
#include <ConvertImage.h>

#include <VisionData.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif


namespace cast
{

class SOIFilter : public ManagedComponent,
		  public VideoClient,
      	  public StereoClient
{
private:

  /**
  * Which camera to get images from
  */
  int camId;
  /**
   * component ID of the video server to connect to
   */
  std::string videoServerName;
  std::string stereoServerName;
  /**
   * our ICE proxy to the video server
   */
  Video::VideoInterfacePrx videoServer;
  
  /**
  * Time and update thresholds
  *(part of the ROI persistency criteria)
  */
  unsigned timeThr;
  int updateThr;
  bool doDisplay;
  
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

  // snapshot support: save video images when -v flag active
  Video::Image m_LeftImage;
  Video::Image m_RightImage;
  int m_idLeftImage;
  int m_idRightImage;
  bool m_bAutoSnapshot;
  
  /**
   * status of SOI persistency
   */
  enum SOIStatus {
  	CANDIDATE, //
  	STABLE,
  	OBJECT,
  	DELETED };
  
  /** 
   * SOI data, contains also data used to evaluate SOI persistency
   */	
  struct SOIData {
  	cdl::WorkingMemoryAddress addr;
  	SOIStatus status;
 // 	VisionData::SurfacePointsSeq points;
  	int updCount;
  	std::string objId;
  	cdl::CASTTime addTime;
  	cdl::CASTTime stableTime;
  	cdl::CASTTime objectTime;
  	cdl::CASTTime deleteTime;
  };
  
  struct colorHLS {
  int h;
  float l;
  float s;
  };
  
  std::map<std::string, SOIData> SOIMap;
  
  std::queue<std::string> objToAdd;
  std::queue<std::string> objToDelete;

  boost::interprocess::named_semaphore* queuesNotEmpty;

private:
#ifdef FEAT_VISUALIZATION
  class CSfDisplayClient: public cogx::display::CDisplayClient
  {
    SOIFilter* pFilter;
  public:
    CSfDisplayClient() { pFilter = NULL; }
    void setClientData(SOIFilter* pSoiFilter) { pFilter = pSoiFilter; }
    void handleEvent(const Visualization::TEvent &event); /*override*/
  };
  CSfDisplayClient m_display;
#endif
  VisionData::ProtoObjectPtr m_LastProtoObject; // We may want to save it
  Video::Image m_ImageLeft;
  Video::Image m_ImageRight;
  Video::Image m_ImageRectLeft;
  std::string m_snapshotFiles;
  std::string m_snapshotFlags; // A:ll, p:oints, l:eft, r:ight, s:segmented, m:mask, L:eftRect, R:ightRect
  void saveSnapshot();
  bool hasSnapFlag(char ch);

private:
  /**
   * callback function called whenever a new SOI appears
   */
  void newSOI(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * callback function called whenever a SOI changes
   */
  void updatedSOI(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * callback function called whenever a SOI is deleted
   */
  void deletedSOI(const cdl::WorkingMemoryChange & _wmc);

  void updatedProtoObject(const cdl::WorkingMemoryChange & _wmc);
  
  /**
   * segment out object roi
   */
  bool segmentObject(const VisionData::SOIPtr soiPtr, Video::Image &imgPatch, VisionData::SegmentMask &segMask, std::vector<VisionData::SurfacePoint> &segPoints, VisionData::ProtoObjectPtr& pProto);
  
  
  void projectSOIPoints(const VisionData::SOI &soi, const VisionData::ROI &roi, std::vector<CvPoint> &projPoints,
					std::vector<CvPoint> &bgProjPoints, std::vector<int> &hull, const float ratio,
					const Video::CameraParameters &cam);

				
  void project3DPoints(const std::vector<VisionData::SurfacePoint> surfPoints, const VisionData::ROI &roi,
                    const float ratio, const Video::CameraParameters &cam,
                    std::vector<CvPoint> &projPoints, std::vector<int> &hull);
                    
  std::vector<VisionData::SurfacePoint>  filter3DPoints(const std::vector<VisionData::SurfacePoint> surfPoints,
  					std::vector<CvPoint> &projPoints, std::vector<CvPoint> &errProjPoints, const VisionData::SegmentMask segMask);
  					
  std::vector<VisionData::SurfacePoint> sample3DPoints(std::vector<VisionData::SurfacePoint> points, int newSize);

					   
  void drawProjectedSOIPoints(IplImage *img, const std::vector<CvPoint> projPoints, const std::vector<CvPoint> bgProjPoints,
  					const std::vector<CvPoint> errProjPoints, const std::vector<int> hull);

  					
  void drawPoints(IplImage *img, const std::vector<CvPoint> projPoints);

  
  void drawHull(IplImage *img, const std::vector<CvPoint> projPoints, const std::vector<int> hull);

  
  std::vector<CvScalar> getSortedHlsList(std::vector<VisionData::SurfacePoint> surfPoints);

  
  std::vector<unsigned char> graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg);
  
  int getHlsDiff(std::vector<CvScalar> hlsList, CvScalar hls, int k);

  std::vector<int> getHueDiffList(std::vector<CvScalar> hslList, int k);

  
  IplImage* getCostImage(IplImage *iplPatchHLS, std::vector<CvPoint> projPoints,
                    std::vector<VisionData::SurfacePoint> surfPoints, float hslmod, float distmod, bool distcost);

  std::vector<CvScalar> colorFilter(std::vector<CvScalar> colors, std::vector<CvScalar> filterColors, int k, int tolerance);

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start compnent run loop
   */
  virtual void runComponent();
 
public:
  SOIFilter();
  virtual ~SOIFilter() {}
};

}

#endif

