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
    return new cogx::ObjectAnalyzer();
  }
}

namespace cogx
{

using namespace cast;
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

  m_TypeMapper.configure(_config);
  m_TypeEnumerator = CVisualTypeMapper::getTypeEnumerator();
}

void ObjectAnalyzer::start()
{
  const char *name = "analyzerSemaphore";
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

enum {
  NewObject, WmObject
};
long ObjectAnalyzer::getOrCreateVisualObject(const string &objectId, VisualObjectPtr &pobject)
{
  try {
	pobject = getMemoryEntry<VisualObject>(objectId);
	return WmObject;
  }
  catch (DoesNotExistOnWMException e) {
	VisualObjectPtr pvobj = new VisualObject;
	pvobj->label = "unknown";
	pvobj->labelConfidence = 1.0f;
	pvobj->protoObjectID = objectId;
	return NewObject;
  }
}

void ObjectAnalyzer::onChange_VL_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
  VisualLearnerRecognitionTaskPtr ptask = getMemoryEntry<VisualLearnerRecognitionTask>(_wmc.address);
  log("Recieved results for VisualLearnerRecognitionTask %s", _wmc.address.id.c_str());
  // ProtoObjectData &data = ProtoObjectMap[ptask->protoObjectId];

  VisualObjectPtr pvobj = new VisualObject;
	pvobj->label = "unknown";
	pvobj->labelConfidence = 1.0f;
	pvobj->protoObjectID = ptask->protoObjectId;
	
  vector<int>::const_iterator plabel;
  for( plabel = ptask->labels.begin(); plabel != ptask->labels.end(); plabel++) {
    pvobj->labels.push_back(*plabel);
  }
  
  vector<double>::const_iterator pdbl;
  for( pdbl = ptask->distribution.begin(); pdbl != ptask->distribution.end(); pdbl++) {
    pvobj->distribution.push_back(*pdbl);
  }

  pvobj->time = getCASTTime();
  if(ProtoObjectMap.find(ptask->protoObjectId) != ProtoObjectMap.end())
	overwriteWorkingMemory(ProtoObjectMap[ptask->protoObjectId].visualObjId, pvobj);
  else
	log("Proto object ID %s deleted. Visual object will not be updated", ptask->protoObjectId.c_str());
}


void ObjectAnalyzer::start_OR_RecognitionTask(const ProtoObjectPtr& pproto, const WorkingMemoryAddress &addr)
{
  ObjectRecognitionTaskPtr ptask = new ObjectRecognitionTask();
  ptask->protoObjectAddr = addr;

  string reqId(newDataID());
  log("Adding new ObjectRecognitionTask: %s", reqId.c_str());
  addToWorkingMemory(reqId, ptask);
}

void ObjectAnalyzer::onChange_OR_RecognitionTask(const cdl::WorkingMemoryChange & _wmc)
{
  ObjectRecognitionTaskPtr ptask = getMemoryEntry<ObjectRecognitionTask>(_wmc.address);
  log("Recieved results for ObjectRecognitionTask %s", _wmc.address.id.c_str());

  VisualObjectPtr pvobj;
  string objid = ProtoObjectMap[ptask->protoObjectAddr.id].visualObjId;
  long mode = getOrCreateVisualObject(objid, pvobj);
  ObjectRecognitionMatchPtr pmatch = ptask->matches[0]; // TODO: error checking, array boundaries

  // Identity labels: separate fields in VisualObject
  TStringVector::const_iterator plabel;
  for( plabel = pmatch->objectId.begin(); plabel != pmatch->objectId.end(); plabel++) {
    pvobj->identLabels.push_back(*plabel);
  }
  TDoubleVector::const_iterator pdbl;
  for( pdbl = pmatch->probability.begin(); pdbl != pmatch->probability.end(); pdbl++) {
    pvobj->identDistrib.push_back(*pdbl);
  }

  // Type labels: identity-labels=>type-labels, type-labels=>type-enum
  if (m_TypeMapper.enabled()) {
	TStringVector typeLabs;
	TDoubleVector typeDist;
	m_TypeMapper.mapLabels(pmatch->objectId, pmatch->probability, typeLabs, typeDist);
	pdbl = typeDist.begin();
	for( plabel = typeLabs.begin(); plabel != typeLabs.end(); plabel++) {
	  long tid = m_TypeEnumerator.getEnum(*plabel);
	  if (tid >= 0) {
		pvobj->labels.push_back(tid);
		pvobj->distribution.push_back(*pdbl);
	  }
	  pdbl++;
	  if (pdbl == typeDist.end()) break;
	}
  }

  if (mode == NewObject) {
	addToWorkingMemory(objid, pvobj);
  }
  else if (mode == WmObject) {
	overwriteWorkingMemory(objid, pvobj);
  }
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

		if(data.status == PROTO)
		{
		  data.status == OBJECT;
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
		ProtoObjectData &obj = ProtoObjectMap[objToDelete.front()];

		if(obj.status == OBJECT)
		{
		  obj.status= DELETED;
		   try
		   {
			 deleteFromWorkingMemory(obj.visualObjId); 
			 CASTTime time=getCASTTime();
			 obj.deleteTime = time;
			 log("A VisualObject deleted ID: %s", obj.visualObjId.c_str());
			 
//			 ProtoObjectMap.erase(objToDelete.front());
		   }
		   catch (DoesNotExistOnWMException e)
		   {
			 log("WARNING: Proto-object ID %s already removed", obj.visualObjId.c_str());
		   }
		}

		objToDelete.pop();
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
  try
  {
	ProtoObjectPtr obj =
	  getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
	
  }
  catch(DoesNotExistOnWMException e)
  {
	log("WARNING: Proto-object ID %s removed before it could be processed", _wmc.address.id.c_str());
	return;
  }

  ProtoObjectData data;

  data.addr = _wmc.address;
  data.addedTime = obj->time;
  data.status = PROTO;

  ProtoObjectMap.insert(make_pair(data.addr.id, data));
  objToAdd.push(data.addr.id);
  debug("A new ProtoObject ID %s ", data.addr.id.c_str());  

  queuesNotEmpty->post();
}

void ObjectAnalyzer::updatedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  try
  {
	ProtoObjectPtr obj =
	  getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
	
  }
  catch(DoesNotExistOnWMException e)
  {
	log("WARNING: Proto-object ID %s removed before it could be processed", _wmc.address.id.c_str());
	return;
  }

  ProtoObjectData &data = ProtoObjectMap[_wmc.address.id];

  CASTTime time=getCASTTime();

//  data.status= STABLE;
  data.lastUpdateTime = time;
  //	queuesNotEmpty->post();objToAdd.push(obj.addr.id);

  debug("A ProtoObject ID %s ",data.addr.id.c_str());

  //  queuesNotEmpty->post();
}

void ObjectAnalyzer::deletedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{

  ProtoObjectData &obj = ProtoObjectMap[_wmc.address.id];
  debug("Detected deletion of the ProtoObject ID %s ", obj.addr.id.c_str());

  if(obj.status == OBJECT)
  {
	objToDelete.push(obj.addr.id);
	queuesNotEmpty->post();
  }
  else
  {
	CASTTime time=getCASTTime();
	obj.status= DELETED;
	obj.deleteTime = time;
  }
  	 
}


}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim */

