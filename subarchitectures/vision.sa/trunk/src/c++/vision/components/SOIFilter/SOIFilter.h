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

#include <VisionData.hpp>


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
  * Segmentation tolerances for distance and hue
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
  
  /**
   * segment out object roi
   */
  void segmentObject(const VisionData::SOIPtr soiPtr, Video::Image &imgPatch, VisionData::SegmentMask &segMask);
  
  
  void projectSOIPoints(const VisionData::SOI &soi, const VisionData::ROI &roi, std::vector<CvPoint> &projPoints,
					std::vector<CvPoint> &bgProjPoints, std::vector<int> &hull, const float ratio,
					const Video::CameraParameters &cam);

				
  void project3DPoints(const std::vector<VisionData::SurfacePoint> surfPoints, const VisionData::ROI &roi,
                    const float ratio, const Video::CameraParameters &cam,
                    std::vector<CvPoint> &projPoints, std::vector<int> &hull);			

					   
  void drawProjectedSOIPoints(IplImage *img, const std::vector<CvPoint> projPoints, const std::vector<CvPoint> bgProjPoints,
  					const std::vector<int> hull);

  					
  void drawPoints(IplImage *img, const std::vector<CvPoint> projPoints);

  
  void drawHull(IplImage *img, const std::vector<CvPoint> projPoints, const std::vector<int> hull);

  
  std::vector<CvScalar> getSortedHlsList(std::vector<VisionData::SurfacePoint> surfPoints);

  
  std::vector<unsigned char> graphCut(int width, int height, int num_labels, IplImage* costImg, IplImage* bgCostImg);
  
  int getHlsDiff(std::vector<CvScalar> hlsList, CvScalar hls, int k);

  std::vector<int> getHueDiffList(std::vector<CvScalar> hslList, int k);

  
  IplImage* getCostImage(IplImage *iplPatchHLS, std::vector<CvPoint> projPoints,
                    std::vector<VisionData::SurfacePoint> surfPoints, float huemod, float distmod, bool distcost);

  std::vector<CvScalar> colorFilter(std::vector<CvScalar> colors, std::vector<CvScalar> filterColors, int k);

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

