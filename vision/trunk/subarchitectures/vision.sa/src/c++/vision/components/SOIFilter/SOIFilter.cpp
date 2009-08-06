/**
 * @author Alen Vrecko
 * @date July 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "SOIFilter.h"

#define TIME_THR_DEFAULT 500
#define UPD_THR_DEFAULT 4
#define CAM_ID_DEFAULT 0
#define DILATE_FACTOR 1.5

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::SOIFilter();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;
using namespace VisionData;

void SOIFilter::configure(const map<string,string> & _config)
{
  configureVideoCommunication(_config);
  configureStereoCommunication(_config);

  map<string,string>::const_iterator it;
  
  updateThr = UPD_THR_DEFAULT;
  timeThr = TIME_THR_DEFAULT;
  camId = CAM_ID_DEFAULT;
  doDisplay = false;
  
  if((it = _config.find("--upd")) != _config.end())
  {
  	istringstream str(it->second);
    str >> updateThr;
  }
  
  if((it = _config.find("--time")) != _config.end())
  {
    istringstream str(it->second);
    str >> timeThr;
  }
  timeThr*= 1000;
  
  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }
  
  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }
}

void SOIFilter::start()
{
  startVideoCommunication(*this);
  startStereoCommunication(*this);
  
  if (doDisplay)
  	cvNamedWindow("Last ROI", 1);
  
  // we want to receive detected SOIs
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::newSOI));
  // .., when they are updated
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::updatedSOI));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::deletedSOI));

}

void SOIFilter::runComponent()
{
  while(isRunning())
  {
    if(!objToAdd.empty())
    {
      SOIData &soi = SOIMap[objToAdd.front()];
      
      if(soi.status == STABLE)
      { 
        ProtoObjectPtr pobj = new ProtoObject;
        pobj->time = getCASTTime();
        pobj->ROIList.push_back(soi.addr.id);
        pobj->image = getImgPatch(soi.addr);
        
        //pobj->points = getPointCloud();
        // getPoints(pobj->points);
        //log("Object has a cloud of %i points", pobj->points.size());
        
        string objId = newDataID();
        addToWorkingMemory(objId, pobj);
        
        soi.objectTime = getCASTTime();
        soi.status = OBJECT;
        soi.objId = objId;
        
        log("A proto-object added ID %s count %u at %u ",
   			soi.addr.id.c_str(), soi.updCount,
   			soi.stableTime.s, soi.stableTime.us);
   	   }
   	   
   	   objToAdd.pop();
     }
      
  }
  
  if (doDisplay)
  	cvDestroyWindow("Last ROI");
}

void SOIFilter::newSOI(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::SOIPtr obj =
    getMemoryEntry<VisionData::SOI>(_wmc.address);
    
   SOIData data;
   
   data.addr = _wmc.address;
   data.addTime = obj->time;
   data.updCount = 0;
   data.status = CANDIDATE;
   
   SOIMap.insert(make_pair(data.addr.id, data));

   debug("A new SOI ID %s ", data.addr.id.c_str());
   

}

void SOIFilter::updatedSOI(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::SOIPtr obj =
    getMemoryEntry<VisionData::SOI>(_wmc.address);
     
  SOIData &soi = SOIMap[_wmc.address.id];
  soi.updCount++;
  
  CASTTime time=getCASTTime();
  
  if(soi.status == CANDIDATE)
  	if(soi.updCount >= updateThr && time.s > soi.addTime.s) // a very rudimental check for now
  	{  	  
  	  soi.status= STABLE;
  	  soi.stableTime = time;
  	  objToAdd.push(soi.addr.id);
  	  
  	  log("An object candidate ID %s count %u at %u ",
   		soi.addr.id.c_str(), soi.updCount,
   		soi.stableTime.s, soi.stableTime.us
   		);
  	}
  
/*  log("#%u: changed SOI ID %s at %u:%u",
   		soi.updCount,
   		soi.id.c_str(),
   		time.s, time.us);
*/

}

void SOIFilter::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{
  
  SOIData &soi = SOIMap[_wmc.address.id];
  
  CASTTime time=getCASTTime();
  soi.status= DELETED;
  soi.deleteTime = time;
  objToDelete.push(soi.addr.id);
  
   debug("deleted SOI ID %s count %u at %u:%u",
   		soi.addr.id.c_str(), soi.updCount,
   		time.s, time.us
   		);
   		 
}

Video::Image SOIFilter::getImgPatch(WorkingMemoryAddress soiAddr)
{
	Video::Image image;
	getImage(camId,image);

	VisionData::SOIPtr soiPtr =
    		getMemoryEntry<VisionData::SOI>(soiAddr);
    		
   	soiPtr->boundingSphere.rad*=DILATE_FACTOR;
	
	ROIPtr roi = projectSOI(image.camPars, *soiPtr);
	
	IplImage *iplImg = convertImageToIpl(image);
	
	CvRect rect;
	
	rect.width = roi->rect.width;
	rect.height = roi->rect.height;
	rect.x = roi->rect.pos.x - rect.width/2;
	rect.y = roi->rect.pos.y - rect.height/2;
	
	debug("Calculated ROI x=%i, y=%i, width=%i, height=%i",
		rect.x, rect.y, rect.width, rect.height);	
	
	cvSetImageROI(iplImg, rect);
	
	IplImage *iplPatch =
		cvCreateImage(cvGetSize(iplImg),
                          iplImg->depth,
                          iplImg->nChannels);
                          
    cvCopy(iplImg, iplPatch);

    if (doDisplay)
    	cvShowImage("Last ROI", iplPatch);

    Video::Image patch;
    convertImageFromIpl(iplPatch, patch); 
	
	return patch;
}

//Stereo::Vector3Seq SOIFilter::getPointCloud()
//{}

}

