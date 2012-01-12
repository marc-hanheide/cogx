/**
 * @file Soi2Roi.h
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Convert instable SOIs to stable ROIs
 */

#ifndef SOI2ROI_H
#define SOI2ROI_H

#include <cast/architecture/ManagedComponent.hpp>

#include <VideoClient.h>
#include <StereoClient.h>
#include <opencv2/highgui/highgui.hpp>
#include <../../VisionUtils.h>


namespace cast
{

/**
 * @class Soi2Roi
 * @brief Convert instable SOIs to stable ROIs. Calculates the mean of the ROI from the last ten positions of the ROI.
 */
class Soi2Roi : public ManagedComponent,
                public VideoClient,
                public StereoClient
{
private:

  /** 
   * @brief ROI data, containing the ROI rect
   */	
	struct ROIData {
		CvRect rect[10];														///< The rectangle of the last ten received SOI => ROI
		std::string addrID;													///< Address of the ROI in WM
		VisionData::ROIPtr stableROI;								///< The stable calculated ROI
		unsigned roiCounter;												///< The actual (next) entry in the ROIs array. Reset to 0, when more then 10 received.
		bool roiOverflow;														///< True, if more than 10 ROI-updates received.
	};
	std::map<std::string, ROIData> rois;					///< Received region of interests (ROIs), stored with the SOI-address

	Video::Image image;														///< Video image

	void receiveSOI(const cdl::WorkingMemoryChange & _wmc);
	void updatedSOI(const cdl::WorkingMemoryChange & _wmc);
	void deletedSOI(const cdl::WorkingMemoryChange & _wmc);

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  Soi2Roi() {}
  virtual ~Soi2Roi();
};

}

#endif



