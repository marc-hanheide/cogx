/**
 * @author Alen Vrecko
 * @date July 2009
 *
 * Changes:
 *   - May 2011 Marko Mahnic:
 *     Refactored into multiple classes/files.
 *   - May 2011 Marko Mahnic:
 *     Refactored the task queue.
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

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream str(it->second);
    str >> camId;
  }

  if((it = _config.find("--coarseSource")) != _config.end())
  {
    m_coarseSource = it->second;
  }

  if((it = _config.find("--fineSource")) != _config.end())
  {
    m_fineSource = it->second;
  }

  if (m_fineSource.size() == 0 && m_coarseSource.size() != 0)
    m_fineSource = m_coarseSource;

  if (m_coarseSource.size() == 0 && m_fineSource.size() != 0)
    m_coarseSource = m_fineSource;

  m_bSameSource = m_coarseSource == m_fineSource;

#ifdef FEAT_VISUALIZATION
  m_display.configureDisplayClient(_config);
#endif
}

void SOIFilter::start()
{
  videoServer = getIceServer<Video::VideoInterface>(videoServerName);
  m_snapper.videoServer = videoServer;

  startPCCServerCommunication(*this);

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

  {
    // SYNC: Lock the monitor
    IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_TaskQueueMonitor);
    m_TaskQueue.push_back(new WmTask(this, WMO_ADD, data.addr.id));
  }
  m_TaskQueueMonitor.notify();
}

void SOIFilter::deletedSOI(const cdl::WorkingMemoryChange & _wmc)
{

  SOIData &soi = SOIMap[_wmc.address.id];

  CASTTime time=getCASTTime();
  soi.deleteTime = time;

  if(soi.status == OBJECT) // || soi.status == STABLE)
  {
    {
      // SYNC: Lock the monitor
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_TaskQueueMonitor);
      m_TaskQueue.push_back(new WmTask(this, WMO_DELETE, soi.addr.id));
    }
    m_TaskQueueMonitor.notify();
  }
  else
    soi.status= DELETED;


  debug("Detected SOI deletion ID %s count %u at %u:%u",
      soi.addr.id.c_str(), soi.updCount, time.s, time.us);		 
}

// Y2: unused, PPO generates only stable SOIs, well ... it should ...
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

      log("An object candidate ID %s count %u at %u ",
          soi.addr.id.c_str(), soi.updCount, soi.stableTime.s, soi.stableTime.us);
      {
        // SYNC: Lock the monitor
        IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_TaskQueueMonitor);
        m_TaskQueue.push_back(new WmTask(this, WMO_ADD, soi.addr.id));
      }
      m_TaskQueueMonitor.notify();

    }
}

// XXX: Added to saveSnapshot with SurfacePatches
void SOIFilter::updatedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  // XXX: ASSUME it's the same protoobject
  m_snapper.m_LastProtoObject = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  m_snapper.saveSnapshot();
}

void SOIFilter::runComponent()
{
  while(isRunning())
  {
    std::deque<WmTask*> tasks;
    tasks.clear();
    {
      // SYNC: Lock the monitor
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_TaskQueueMonitor);

      // SYNC: If queue is empty, unlock the monitor and wait for notify() or timeout
      if (m_TaskQueue.size() < 1)
        m_TaskQueueMonitor.timedWait(IceUtil::Time::seconds(2));

      // SYNC: Continue with a locked monitor

#if 0
      // Process whole queue at once
      tasks = m_TaskQueue;
      m_TaskQueue.clear();
#else
      // Process queued items one by one
      if (m_TaskQueue.size() > 0) {
        tasks.push_back(m_TaskQueue.front());
        m_TaskQueue.pop_front();
      }
#endif
      // SYNC: unlock the monitor when going out of scope
    }

    // if (tasks.empty()) debug("Timeout");

    while (!tasks.empty())
    { 
      WmTask* ptask = tasks.front();
      tasks.pop_front();
      ptask->execute();
      delete ptask;
    }
  }

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

/*
 * TODO: 2 step VisualObject generation.
 *   1. If the SOI comes from coarseSource, create a proto-object with the
 *      desired ViewCones
 *   2. If the SOI comes from fineSource, we need to know which PO it belongs
 *      to (SOI matching by position), then we update the PO and create the VO.
 * TODO: Decide how to process stuff when there is only a single source ...
 */
void SOIFilter::WmTask::exec_add()
{
}

void SOIFilter::WmTask::exec_delete()
{
}


void SOIFilter::WmTask_Year2::exec_add()
{
  pSoiFilter->log("An add proto-object instruction");
  SOIData &soi = pSoiFilter->SOIMap[soi_id];

  pSoiFilter->log("SOI retrieved");

  if(soi.status == STABLE)
  {
    soi.status = OBJECT;
    try
    { 
      SOIPtr soiPtr = pSoiFilter->getMemoryEntry<VisionData::SOI>(soi.addr);

      ProtoObjectPtr pobj = new ProtoObject;
      pSoiFilter->m_snapper.m_LastProtoObject = pobj;
      if(pSoiFilter->m_segmenter.segmentObject(soiPtr, pobj->image, pobj->mask, pobj->points, pobj))
      {
        pobj->time = pSoiFilter->getCASTTime();
        pobj->SOIList.push_back(soi.addr.id);

        //m_LastProtoObject = pobj;
        if (! pobj.get()) {
          pSoiFilter->println(" *********** WTF ************");
        }

        string objId = pSoiFilter->newDataID();
        pSoiFilter->addToWorkingMemory(objId, pobj);

        soi.objectTime = pSoiFilter->getCASTTime();
        soi.objId = objId;

        pSoiFilter->log("A proto-object added ID %s",
            objId.c_str(), soi.updCount);
      }
      else
        pSoiFilter->log("Nothing segmented, no proto-object added");

    }
    catch (DoesNotExistOnWMException e)
    {
      pSoiFilter->log("SOI ID: %s was removed before it could be processed", soi.addr.id.c_str());
    }
  }
  else if(soi.status == DELETED) {
    pSoiFilter->log("SOI was removed before it could be processed");
    try
    { 
      SOIPtr soiPtr = pSoiFilter->getMemoryEntry<VisionData::SOI>(soi.addr);
    }
    catch (DoesNotExistOnWMException e)
    {
      pSoiFilter->log("SOI ID: %s was removed before it could be processed", soi.addr.id.c_str());
    }
  }
}

void SOIFilter::WmTask_Year2::exec_delete()
{
  pSoiFilter->log("A delete proto-object instruction");
  SOIData &soi = pSoiFilter->SOIMap[soi_id];

  //		if(soi.status == OBJECT)
  //		{
  soi.status = DELETED;
  try
  {
    if (soi.objId.size()>0 && pSoiFilter->existsOnWorkingMemory(soi.objId)) {
      pSoiFilter->deleteFromWorkingMemory(soi.objId);

      pSoiFilter->log("A proto-object deleted ID: %s ", soi.objId.c_str());
    }
    else
      pSoiFilter->log("WARNING: Proto-object ID %s not in WM", soi.objId.c_str());

    //			SOIMap.erase(objToDelete.front());	  	
  }
  catch (DoesNotExistOnWMException e)
  {
    pSoiFilter->log("WARNING: Proto-object ID %s not in WM (exception)", soi.objId.c_str());
  }
  //		}
  //		else if(soi.status == STABLE)
  //		{
  //		  log("Have to wait until the object is processed");
  //		  objToDelete.push(soi.objId);
  //		  queuesNotEmpty->post();
  //		}
}


} // namespace
/* vim:set fileencoding=utf-8 sw=2 ts=8 et:vim */

