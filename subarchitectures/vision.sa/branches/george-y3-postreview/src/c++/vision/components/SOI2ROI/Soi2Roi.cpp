/**
 * @file Soi2Roi.cpp
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Convert instable SOIs to stable ROIs
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include "Soi2Roi.h"

using namespace VisionData;

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
	cast::CASTComponentPtr newComponent() {
    return new cast::Soi2Roi();
  }
}

namespace cast
{

/**
 *	@brief Destructor of class StereoDetector
 */
Soi2Roi::~Soi2Roi()
{}


/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config
 */
void Soi2Roi::configure(const std::map<std::string,std::string> & _config)
{
  // first let the base classes configure themselves
  configureStereoCommunication(_config);
}

/**
 *	@brief Called by the framework after configuration, before run loop.
 */
void Soi2Roi::start()
{
	// videoServer = getIceServer<Video::VideoInterface>(videoServerName);
	startStereoCommunication(*this);

	// add change filter for SOI changes (add / update / delete)
  addChangeFilter(createLocalTypeFilter<SOI>(cdl::ADD),
    new MemberFunctionChangeReceiver<Soi2Roi>(this, &Soi2Roi::receiveSOI));
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<Soi2Roi>(this, &Soi2Roi::updatedSOI));
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
	  new MemberFunctionChangeReceiver<Soi2Roi>(this, &Soi2Roi::deletedSOI));
}


/**
 *	@brief Called by the framework to start component run loop.
 */
void Soi2Roi::runComponent()
{
}


/**
 *	@brief Receive a new created SOI. Calculate ROI and add to stableROI.
 *	@param wmc Working memory change.
 */
void Soi2Roi::receiveSOI(const cdl::WorkingMemoryChange & _wmc)
{
	log("received SOI @ %s", _wmc.address.id.c_str());
// 	try
// 	{

	SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
	Video::Image image;
	getRectImage(0, 640, image);	// 0 = left / 640 = width
	ROIPtr roiPtr = projectSOI(image.camPars, *soi);

	ROIData roiData;
	roiData.rect[0].width = roiPtr->rect.width;
	roiData.rect[0].height = roiPtr->rect.height;
	roiData.rect[0].x = roiPtr->rect.pos.x;
	roiData.rect[0].y = roiPtr->rect.pos.y;

	// get new WM address and set roiCounter and roiOverflow
	roiData.addrID = newDataID();
	roiData.roiCounter = 1;
	roiData.roiOverflow = false;

	// calculate ROI
	roiData.stableROI = new VisionData::ROI;
	roiData.stableROI->rect.pos.x = roiData.rect[0].x;
	roiData.stableROI->rect.pos.y = roiData.rect[0].y;
	roiData.stableROI->rect.width = roiData.rect[0].width;
	roiData.stableROI->rect.height = roiData.rect[0].height;

	// add calculated, stable ROI to WM
	addToWorkingMemory(roiData.addrID, roiData.stableROI);

	// insert into ROI map
	rois[_wmc.address.id] = roiData;

	log("new roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect[0].x, roiData.rect[0].y, roiData.rect[0].width, roiData.rect[0].height, roiData.addrID.c_str());

// 	}
// 	catch (DoesNotExistOnWMException e)
// 	{
// 		log("SOI ID: %s was removed before it could be processed", _wmc.address.id.c_str());
// 	}
// 	catch (std::exception e)
// 	{
// 		log("unknown exception.");
// 	}
}

/**
 *	@brief Receive a updated SOI. Calculate ROI and add to stableROI.
 *	@param wmc Working memory change.
 */
void Soi2Roi::updatedSOI(const cdl::WorkingMemoryChange & _wmc)
{
	log("updated SOI @ %s", _wmc.address.id.c_str());

	SOIPtr soi = getMemoryEntry<SOI>(_wmc.address);
	Video::Image image;
	getRectImage(0, 640, image);
	ROIPtr roiPtr = projectSOI(image.camPars, *soi);

	// get data from map and enter new ROI
	ROIData roiData = rois[_wmc.address.id];
	roiData.rect[roiData.roiCounter].width = roiPtr->rect.width;
	roiData.rect[roiData.roiCounter].height = roiPtr->rect.height;
	roiData.rect[roiData.roiCounter].x = roiPtr->rect.pos.x;
	roiData.rect[roiData.roiCounter].y = roiPtr->rect.pos.y;

	// calculate stable mean from the last updates
	roiData.stableROI->rect.pos.x = 0;
	roiData.stableROI->rect.pos.y = 0;
	roiData.stableROI->rect.width = 0;
	roiData.stableROI->rect.height = 0;
	unsigned counter = 0;
	if(roiData.roiOverflow) counter = 10;
	else counter = roiData.roiCounter;
	for(unsigned i=0; i<counter; i++)
	{
		roiData.stableROI->rect.pos.x += roiData.rect[i].x;
		roiData.stableROI->rect.pos.y += roiData.rect[i].y;
		roiData.stableROI->rect.width += roiData.rect[i].width;
		roiData.stableROI->rect.height += roiData.rect[i].height;
	}
	roiData.stableROI->rect.pos.x /= counter;
	roiData.stableROI->rect.pos.y /= counter;
	roiData.stableROI->rect.width /= counter;
	roiData.stableROI->rect.height /= counter;

	// increase counter
	roiData.roiCounter++;
	if(roiData.roiCounter == 10)
	{
		roiData.roiCounter = 0;
		roiData.roiOverflow = true;
	}

	// add calculated, stable ROI to WM
	overwriteWorkingMemory(roiData.addrID, roiData.stableROI);

	// insert into ROI map
	rois[_wmc.address.id] = roiData;

	log("updated roi: x=%i, y=%i, width=%i, height=%i @ %s", roiData.rect[0].x, roiData.rect[0].y, roiData.rect[0].width, roiData.rect[0].height, roiData.addrID.c_str());
}


/**
 *	@brief Receive a deleted SOI.
 *	@param wmc Working memory change.
 */
void Soi2Roi::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{
	log("deleted SOI @ %s", _wmc.address.id.c_str());
	ROIData roiData = rois[_wmc.address.id];
	deleteFromWorkingMemory(roiData.addrID);
	rois.erase(_wmc.address.id);
}

}








