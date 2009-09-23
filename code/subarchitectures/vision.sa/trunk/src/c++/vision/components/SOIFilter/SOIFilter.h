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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "GCoptimization.h"
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <StereoClient.h>
#include <../../VisionUtils.h>

#include <VisionData.hpp>


namespace cast
{

class SOIFilter : public ManagedComponent,
		  public VideoClient
{
private:

  /**
  * Which camera to get images from
  */
  int camId;
  
  /**
  * Time and update thresholds
  *(part of the ROI persistency criteria)
  */
  unsigned timeThr;
  int updateThr;
  bool doDisplay;
  
  /**
   * stetus of SOI persistency
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
  	VisionData::Vector3Seq points;
  	int updCount;
  	std::string objId;
  	cdl::CASTTime addTime;
  	cdl::CASTTime stableTime;
  	cdl::CASTTime objectTime;
  	cdl::CASTTime deleteTime;
  };
  
  std::map<std::string, SOIData> SOIMap;
  
  std::queue<std::string> objToAdd;
  std::queue<std::string> objToDelete;

  boost::interprocess::named_semaphore* queuesNotEmpty;

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
  
  void segmentObject(const VisionData::SOIPtr soiPtr, Video::Image &imgPatch, VisionData::SegmentMaskPtr &segMask);
  
  void projectSOIPoints(const VisionData::SOI &soi, const VisionData::ROI &roi, std::vector<CvPoint> &projPoints,
					std::vector<CvPoint> &bgProjPoints, std::vector<int> &hull, const float ratio, const Video::CameraParameters &cam);
					   
  void drawProjectedSOIPoints(IplImage *img, const std::vector<CvPoint> projPoints, const std::vector<CvPoint> bgProjPoints,
  					const std::vector<int> hull);
  
  std::list<int> getSortedHueList(std::vector<CvPoint> projPoints, const IplImage* hueImg);
  
  std::vector<int> graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg, int k);
  
  std::vector<int> getHueCostList(std::list<int> hueList, int k);
  
  IplImage* getCostImage(IplImage *iplPatchHLS, std::vector<CvPoint> projPoints, float huemod, float distmod);

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
  virtual ~SOIFilter() {}
};

}

#endif

