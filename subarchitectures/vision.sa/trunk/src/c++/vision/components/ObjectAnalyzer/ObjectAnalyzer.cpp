/**
 * @author Alen Vrecko
 * @date October 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/format.hpp>
#include "ObjectAnalyzer.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ObjectAnalyzer();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;
using namespace VisionData;

using namespace boost::interprocess;
using namespace boost::posix_time;


void ObjectAnalyzer::configure(const map<string,string> & _config)
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

void ObjectAnalyzer::start()
{
  char *name = "analyzerSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);

  if (doDisplay)
  {
  }

  // we want to receive detected ProtoObjects
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::ADD),
	  new MemberFunctionChangeReceiver<ObjectAnalyzer>(this,
		&ObjectAnalyzer::newProtoObject));
  // .., when they are updated
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectAnalyzer>(this,
		&ObjectAnalyzer::updatedProtoObject));
  // .. and when they are deleted
  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::DELETE),
	  new MemberFunctionChangeReceiver<ObjectAnalyzer>(this,
		&ObjectAnalyzer::deletedProtoObject));

  addChangeFilter(createLocalTypeFilter<VisionData::VisualLearnerRecognitionTask>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectAnalyzer>(this,
		&ObjectAnalyzer::onChange_VL_RecognitionTask));

  addChangeFilter(createLocalTypeFilter<VisionData::ObjectRecognitionTask>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<ObjectAnalyzer>(this,
		&ObjectAnalyzer::onChange_OR_RecognitionTask));

}

void ObjectAnalyzer::start_VL_RecognitionTask(const ProtoObjectPtr& pproto, const WorkingMemoryAddress &addr)
{
   log("Adding new VisualLearnerRecognitionTask");
   VisualLearnerRecognitionTaskPtr ptask = new VisualLearnerRecognitionTask();
   ptask->protoObjectId = addr.id;
   // TODO: Add learning data: labels, confidences!

   string reqId(newDataID());
   addToWorkingMemory(reqId, ptask);
}

void ObjectAnalyzer::onChange_VL_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
  VisualLearnerRecognitionTaskPtr ptask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
  log("Recieved results for VisualLearnerRecognitionTask %s", _wmc.address.id.c_str());
  // ProtoObjectData &data = ProtoObjectMap[ptask->protoObjectId];

  AttrObjectPtr pAttrObject = new AttrObject();
  pAttrObject->protoObjectID = ptask->protoObjectId;
  vector<int>::const_iterator plabel;
  for( plabel = ptask->colorLabel.begin(); plabel != ptask->colorLabel.end(); plabel++) {
    pAttrObject->colorLabel.push_back(str(boost::format("%d") % *plabel));
  }
  vector<double>::const_iterator pdbl;
  for( pdbl = ptask->colorDistr.begin(); pdbl != ptask->colorDistr.end(); pdbl++) {
    pAttrObject->colorDistr.push_back(*pdbl);
  }
  string attrId = newDataID();
  pAttrObject->time = getCASTTime();
  addToWorkingMemory(attrId, pAttrObject);
}

void ObjectAnalyzer::start_OR_RecognitionTask(const ProtoObjectPtr& pproto, const WorkingMemoryAddress &addr)
{
  ObjectRecognitionTaskPtr ptask = new  ObjectRecognitionTask();
  ptask->protoObjectAddr = addr;

  string reqId(newDataID());
  log("Adding new ObjectRecognitionTask: %s", reqId.c_str());
  addToWorkingMemory(reqId, ptask);
}

void ObjectAnalyzer::onChange_OR_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
  ObjectRecognitionTaskPtr ptask = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
  log("Recieved results for ObjectRecognitionTask %s", _wmc.address.id.c_str());
  // TODO: Do sth with the labels of recognized objects
}

void ObjectAnalyzer::runComponent()
{
  while(isRunning())
  {
	ptime t(second_clock::universal_time() + seconds(2));

	if (queuesNotEmpty->timed_wait(t))
	{
	  log("Got something in my queues");

	  if(!objToAdd.empty())
	  { 

		log("An add object instruction");
		ProtoObjectData &data = ProtoObjectMap[objToAdd.front()];

		if(data.status == STABLE)
		{
		  try
		  {
			ProtoObjectPtr objPtr = getMemoryEntry<VisionData::ProtoObject>(data.addr);

			VisualObjectPtr pvobj = new VisualObject;
			pvobj->time = getCASTTime();
			pvobj->label = "unkknown";
			pvobj->labelConfidence = 1.0f;

			string objId = newDataID();
			addToWorkingMemory(objId, pvobj);

			data.visualObjId = objId;

			log("A visual object added for protoObject ID %s", data.addr.id.c_str());
			start_OR_RecognitionTask(objPtr, data.addr); 
			start_VL_RecognitionTask(objPtr, data.addr); 
		  }
		  catch (DoesNotExistOnWMException e)
		  {
			log("ProtoObject ID: %s was removed before it could be processed", data.addr.id.c_str());
		  }
		}

		objToAdd.pop();
	  }
	  else if(!objToDelete.empty())
	  {
		log("A delete proto-object instruction");
		/* ProtoObjectData &obj = ProtoObjectMap[objToDelete.front()];

		   if(obj.status == DELETED)
		   {
			 try
			 {
			   deleteFromWorkingMemory(obj.objId);

			   ProtoObjectMap.erase(objToDelete.front());

			   log("A proto-object deleted ID: %s TIME: %u",
			   obj.objId, obj.stableTime.s, obj.stableTime.us);
			 }
			 catch (DoesNotExistOnWMException e)
			 {
			   log("WARNING: Proto-object ID %s already removed", obj.objId);
			 }
		   }

		   objToDelete.pop(); */
	  }
	}
	//    else
	//		log("Timeout");   
  }

  log("Removing semaphore ...");
  queuesNotEmpty->remove("analyzerSemaphore");
  delete queuesNotEmpty;

  if (doDisplay)
  {
  }
}

void ObjectAnalyzer::newProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  ProtoObjectPtr obj =
	getMemoryEntry<VisionData::ProtoObject>(_wmc.address);

  ProtoObjectData data;

  data.addr = _wmc.address;
  data.addedTime = obj->time;
  data.status = STABLE;

  ProtoObjectMap.insert(make_pair(data.addr.id, data));
  objToAdd.push(data.addr.id);
  debug("A new ProtoObject ID %s ", data.addr.id.c_str());  

  queuesNotEmpty->post();
}

void ObjectAnalyzer::updatedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::ProtoObjectPtr obj =
	getMemoryEntry<VisionData::ProtoObject>(_wmc.address);

  ProtoObjectData &data = ProtoObjectMap[_wmc.address.id];

  CASTTime time=getCASTTime();

  data.status= STABLE;
  data.lastUpdateTime = time;
  //	queuesNotEmpty->post();objToAdd.push(obj.addr.id);

  debug("A ProtoObject ID %s ",data.addr.id.c_str());

  //  queuesNotEmpty->post();
}

void ObjectAnalyzer::deletedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{

  ProtoObjectData &obj = ProtoObjectMap[_wmc.address.id];

  CASTTime time=getCASTTime();
  obj.status= DELETED;
  obj.deleteTime = time;
  objToDelete.push(obj.addr.id);

  log("Deleted ProtoObject ID %s ",
	  obj.addr.id.c_str());

  queuesNotEmpty->post();		 
}


}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim */

