/**
 * @author Alen Vrecko
 * @date July 2009
 */

#include "SOIFilter.h"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <fstream>

#define TIME_THR_DEFAULT 500
#define UPD_THR_DEFAULT 5
#define CAM_ID_DEFAULT 0

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
using namespace Video;

using namespace boost::interprocess;
using namespace boost::posix_time;

SOIFilter::SOIFilter()
{
  m_segmenter.pPcClient = this;
  m_snapper.logger = this;
  m_snapper.videoServer = videoServer;

#ifdef FEAT_VISUALIZATION
  m_segmenter.pDisplay = &m_display;
#endif

}

void SOIFilter::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  // first let the base classes configure themselves
  configureServerCommunication(_config);

  m_segmenter.configure(_config);

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

  if((it = _config.find("--videoname")) != _config.end())
  {
    videoServerName = it->second;
  }

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

void SOIFilter::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);
  m_snapper.videoServer = videoServer;

  startPCCServerCommunication(*this);

  char *name = "filterSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.setClientData(this);
  m_display.installEventReceiver();
  m_display.addButton(ID_OBJ_LAST_SEGMENTATION, "take.snapshot", "&Snapshot");
#else
  if (doDisplay)
  {
    cvNamedWindow("Full image", 1);
    cvNamedWindow("Last ROI Segmentation", 1);
    cvNamedWindow("Color Filtering", 1);
  }
#endif

  // we want to receive detected SOIs
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::ADD),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::newSOI));
  // .., when they are updated
  //	addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
  //		new MemberFunctionChangeReceiver<SOIFilter>(this,
  //		  &SOIFilter::updatedSOI));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::SOI>(cdl::DELETE),
      new MemberFunctionChangeReceiver<SOIFilter>(this,
        &SOIFilter::deletedSOI));

  // XXX: added to save SurfacePatches with saveSnapshot
  if (m_snapper.m_bAutoSnapshot) {
    log("AUTOSNAP is ON; Triggered on ProtoObject OVERWRITE");
    addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<SOIFilter>(this,
          &SOIFilter::updatedProtoObject));
  }
}

#ifdef FEAT_VISUALIZATION
void SOIFilter::CSfDisplayClient::handleEvent(const Visualization::TEvent &event)
{
  if (event.type == Visualization::evButtonClick) {
    if (event.sourceId == "take.snapshot") {
      pFilter->m_snapper.saveSnapshot();
    }
  }
}
#endif

void SOIFilter::runComponent()
{
  while(isRunning())
  {
    ptime t(second_clock::universal_time() + seconds(2));

    if (queuesNotEmpty->timed_wait(t))
    {
      log("Got something in my queues");

      if(!objToAdd.empty())
      { 

        log("An add proto-object instruction");
        SOIData &soi = SOIMap[objToAdd.front()];

        log("SOI retrieved");

        if(soi.status == STABLE)
        {
          soi.status = OBJECT;
          try
          { 
            SOIPtr soiPtr = getMemoryEntry<VisionData::SOI>(soi.addr);

            ProtoObjectPtr pobj = new ProtoObject;
            m_snapper.m_LastProtoObject = pobj;
            if(m_segmenter.segmentObject(soiPtr, pobj->image, pobj->mask, pobj->points, pobj))
            {
              pobj->time = getCASTTime();
              pobj->SOIList.push_back(soi.addr.id);

              //m_LastProtoObject = pobj;
              if (pobj == NULL) {
                println(" *********** WTF ************");
              }

              string objId = newDataID();
              addToWorkingMemory(objId, pobj);

              soi.objectTime = getCASTTime();
              soi.objId = objId;

              log("A proto-object added ID %s",
                  objId.c_str(), soi.updCount);
            }
            else
              log("Nothing segmented, no proto-object added");

          }
          catch (DoesNotExistOnWMException e)
          {
            log("SOI ID: %s was removed before it could be processed", soi.addr.id.c_str());
          }
        }
        else if(soi.status == DELETED) {
          log("SOI was removed before it could be processed");
          try
          { 
            SOIPtr soiPtr = getMemoryEntry<VisionData::SOI>(soi.addr);
          }
          catch (DoesNotExistOnWMException e)
          {
            log("SOI ID: %s was removed before it could be processed", soi.addr.id.c_str());
          }
        }

        objToAdd.pop();
      }
      else if(!objToDelete.empty())
      {
        log("A delete proto-object instruction");
        SOIData &soi = SOIMap[objToDelete.front()];

        //		if(soi.status == OBJECT)
        //		{
        soi.status = DELETED;
        try
        {
          if (soi.objId.size()>0 && existsOnWorkingMemory(soi.objId)) {
            deleteFromWorkingMemory(soi.objId);

            log("A proto-object deleted ID: %s ", soi.objId.c_str());
          }
          else
            log("WARNING: Proto-object ID %s not in WM", soi.objId.c_str());

          //			SOIMap.erase(objToDelete.front());	  	
        }
        catch (DoesNotExistOnWMException e)
        {
          log("WARNING: Proto-object ID %s not in WM (exception)", soi.objId.c_str());
        }
        //		}
        //		else if(soi.status == STABLE)
        //		{
        //		  log("Have to wait until the object is processed");
        //		  objToDelete.push(soi.objId);
        //		  queuesNotEmpty->post();
        //		}

        objToDelete.pop(); 
      }
      else
        log("Timeout"); 
    }
    //      
  }

  log("Removing semaphore ...");
  queuesNotEmpty->remove("filterSemaphore");
  delete queuesNotEmpty;

#ifndef FEAT_VISUALIZATION
  if (doDisplay)
  {
    log("Destroying OpenCV windows..");
    cvDestroyWindow("Full image");
    cvDestroyWindow("Last ROI Segmentation");
    cvDestroyWindow("Color Filtering");
  }
#endif
}

void SOIFilter::newSOI(const cdl::WorkingMemoryChange & _wmc)
{
  SOIPtr obj =
    getMemoryEntry<VisionData::SOI>(_wmc.address);

  SOIData data;

  data.addr = _wmc.address;
  data.addTime = obj->time;
  data.stableTime = getCASTTime();
  data.updCount = 0;
  data.status = STABLE;
  //  data.objId = "";

  SOIMap.insert(make_pair(data.addr.id, data));

  debug("A new SOI ID %s ", data.addr.id.c_str());
  log("An object candidate ID %s at %u",
      data.addr.id.c_str(), data.addTime.s, data.addTime.us);

  objToAdd.push(data.addr.id);  

  queuesNotEmpty->post();

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
          soi.addr.id.c_str(), soi.updCount, soi.stableTime.s, soi.stableTime.us);

      queuesNotEmpty->post();
    }
}

// XXX: Added to saveSnapshot with SurfacePatches
void SOIFilter::updatedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  // XXX: ASSUME it's the same protoobject
  m_snapper.m_LastProtoObject = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  m_snapper.saveSnapshot();
}

void SOIFilter::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{

  SOIData &soi = SOIMap[_wmc.address.id];

  CASTTime time=getCASTTime();
  soi.deleteTime = time;

  if(soi.status == OBJECT) // || soi.status == STABLE)
  {
    objToDelete.push(soi.addr.id);
    queuesNotEmpty->post();
  }
  else
    soi.status= DELETED;


  debug("Detected SOI deletion ID %s count %u at %u:%u",
      soi.addr.id.c_str(), soi.updCount, time.s, time.us);		 
}

}
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

