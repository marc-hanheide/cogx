/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * A component the filters out persistent SOIs.
 */

#ifndef SOI_FILTER_H
#define SOI_FILTER_H

#include "GraphCutSegmenter.h"
#include "Snapper.h"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <PointCloudClient.h>
#include <../../VisionUtils.h>
#include <ConvertImage.h>

#include <VisionData.hpp>

#ifdef FEAT_VISUALIZATION
#include <CDisplayClient.hpp>
#endif

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <IceUtil/IceUtil.h>
namespace cast
{

class SOIFilter : public ManagedComponent,
  public VideoClient, public PointCloudClient
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
  GraphCutSegmenter m_segmenter;
  Snapper m_snapper;

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
  
  enum WmOperation { WMO_ADD, WMO_DELETE };
  struct WmTask
  {
    WmOperation operation;
    std::string soi_id;
    WmTask(WmOperation op, std::string wmid)
    {
      operation = op;
      soi_id = wmid; // Current SA is implied
    }
  };
  std::deque<WmTask> m_TaskQueue;
  IceUtil::Monitor<IceUtil::Mutex> m_TaskQueueMonitor;

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

} // namespace

#endif
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */
