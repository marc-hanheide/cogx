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

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
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
  
  Video::Image getImgPatch(cdl::WorkingMemoryAddress soiAddr);

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

