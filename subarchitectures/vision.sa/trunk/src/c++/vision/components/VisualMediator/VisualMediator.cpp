/**
 * @author Alen Vrecko
 * @date October 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "VisualMediator.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::VisualMediator();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;
using namespace VisionData;

using namespace boost::interprocess;
using namespace boost::posix_time;

using namespace binder::autogen::core;
using namespace binder::autogen::featvalues;

void VisualMediator::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
  
  updateThr = UPD_THR_DEFAULT;
  
  if((it = _config.find("--upd")) != _config.end())
  {
  	istringstream str(it->second);
    str >> updateThr;
  }
  
  if((it = _config.find("--display")) != _config.end())
  {
    doDisplay = true;
  }
}

void VisualMediator::start()
{
  char *name = "mediatorSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);
  
  if (doDisplay)
  {
  }
  
  // we want to receive detected VisualObjects
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<VisualMediator>(this,
        &VisualMediator::newVisualObject));
  // .., when they are updated
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VisualMediator>(this,
        &VisualMediator::updatedVisualObject));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::DELETE),
      new MemberFunctionChangeReceiver<VisualMediator>(this,
        &VisualMediator::deletedVisualObject));

}

void VisualMediator::runComponent()
{
  while(isRunning())
  {
    ptime t(second_clock::universal_time() + seconds(2));

    if (queuesNotEmpty->timed_wait(t))
    {
	    log("Got something in my queues");

	    if(!proxyToAdd.empty())
	    { 
	    
	      log("An add object instruction");
	      VisualObjectData &data = VisualObjectMap[proxyToAdd.front()];
	      
	      if(data.status == STABLE)
	      {
		    try
		    {
		        VisualObjectPtr objPtr = getMemoryEntry<VisionData::VisualObject>(data.addr);
		        
		        WorkingMemoryPointerPtr origin = createWorkingMemoryPointer(getSubarchitectureID(), data.addr.id, "VisualObject");
		        
		        FeatureValuePtr value = createStringValue (objPtr->label.c_str(), objPtr->labelConfidence);
  				FeaturePtr label = createFeatureWithUniqueFeatureValue ("label", value);
  
  				ProxyPtr proxy = createNewProxy (origin, 1.0f);
  
  				addFeatureToProxy (proxy, label);
  				
  				addProxyToWM(proxy);
		
		        data.proxyId = proxy->entityID;
		
		        log("A visual proxy ID %s added for object ID %s",
		        	proxy->entityID.c_str(), data.addr.id.c_str());
  
		    }
		    catch (DoesNotExistOnWMException e)
		    {
	            log("VisualObject ID: %s was removed before it could be processed", data.addr.id);
		    }
		  }
	   	   
	      proxyToAdd.pop();
	   }
	   else if(!proxyToDelete.empty())
	   {
	   	  log("A delete proto-object instruction");
/*	      VisualObjectData &obj = VisualObjectMap[proxyToDelete.front()];
	       
	      if(obj.status == DELETED)
	      {
		    try
		    {
		       	deleteFromWorkingMemory(obj.objId);
		        
		        VisualObjectMap.erase(proxyToDelete.front());
		
		        log("A proto-object deleted ID: %s TIME: %u",
	           			obj.objId, obj.stableTime.s, obj.stableTime.us);
		    }
		    catch (DoesNotExistOnWMException e)
		    {
	            log("WARNING: Proto-object ID %s already removed", obj.objId);
		    }
		  }
	   	   
	      proxyToDelete.pop(); */
	   }
    }
//    else
//		log("Timeout");   
  }

  log("Removing semaphore ...");
  queuesNotEmpty->remove("mediatorSemaphore");
  delete queuesNotEmpty;
  
  if (doDisplay)
  {
  }
}

void VisualMediator::newVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
    
   VisualObjectData data;
   
   data.addr = _wmc.address;
   data.addedTime = obj->time;
   data.status = STABLE;
   
   VisualObjectMap.insert(make_pair(data.addr.id, data));
   proxyToAdd.push(data.addr.id);
   debug("A new VisualObject ID %s ", data.addr.id.c_str());  
   
   queuesNotEmpty->post();
}

void VisualMediator::updatedVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
     
  VisualObjectData &data = VisualObjectMap[_wmc.address.id];
  
  CASTTime time=getCASTTime();
   	  
  data.status= STABLE;
  data.lastUpdateTime = time;
//	queuesNotEmpty->post();proxyToAdd.push(obj.addr.id);
  	  
  debug("A VisualObject ID %s ",data.addr.id.c_str());
   		
//  queuesNotEmpty->post();
}

void VisualMediator::deletedVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  
  VisualObjectData &obj = VisualObjectMap[_wmc.address.id];
  
  CASTTime time=getCASTTime();
  obj.status= DELETED;
  obj.deleteTime = time;
  proxyToDelete.push(obj.addr.id);
    
  log("Deleted Visual object ID %s ",
  	obj.addr.id.c_str());
  	
  queuesNotEmpty->post();		 
}


}

