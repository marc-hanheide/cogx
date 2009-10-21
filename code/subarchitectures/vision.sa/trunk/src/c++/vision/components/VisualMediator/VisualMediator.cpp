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

using namespace beliefmodels::adl;
using namespace beliefmodels::domainmodel::cogx;

void VisualMediator::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
  
  map<string,string>::const_iterator it;

  updateThr = UPD_THR_DEFAULT;
  
  if (_config.find("-bsa") != _config.end()) {
	m_bindingSA=_config.find("-bsa")->second;
  } else if (_config.find("--bsa") != _config.end()) {
	m_bindingSA=_config.find("--bsa")->second;
  } else {
   m_bindingSA="binder";
  }

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
  const char *name = "mediatorSemaphore";
  named_semaphore(open_or_create, name, 0);
  queuesNotEmpty = new named_semaphore(open_only, name);
log("HELLO, Mediator active");
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

 // a filter for belief updates
  addChangeFilter(createGlobalTypeFilter<Belief>(cdl::ADD),
	  new MemberFunctionChangeReceiver<VisualMediator>(this,
		&VisualMediator::updatedBelief));

  
  addChangeFilter(createGlobalTypeFilter<Belief>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<VisualMediator>(this,
		&VisualMediator::updatedBelief));
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
			log("VisualObject ID: %s was removed before it could be processed", data.addr.id.c_str());
		  }
		}

		proxyToAdd.pop();
	  }
	  else if(!proxyToDelete.empty())
	  {
		log("A delete proto-object instruction");
		/* VisualObjectData &obj = VisualObjectMap[proxyToDelete.front()];

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


void VisualMediator::updatedBelief(const cdl::WorkingMemoryChange & _wmc)
{
  BeliefPtr obj =
	getMemoryEntry<Belief>(_wmc.address);

  string unionID;
  string visObjID;
  
  if(unionRef(obj->phi, unionID))
  {
	log("Found reference to union ID %s in belief", unionID.c_str());
	
	UnionPtr uni = getMemoryEntry<Union>(unionID, m_bindingSA);
	
	vector<ProxyPtr>::iterator it;
	
	bool found = false;
	
	for(it = uni->includedProxies.begin(); it != uni->includedProxies.end() && !found; it++)
	  if((*it)->origin->address.subarchitecture == getSubarchitectureID())
	  {
		  found = true;
		  visObjID = (*it)->origin->address.id;
	  }
		
	 if(!found)
	  return;
	  
	log("Found the object of the belief: visualObject ID %s", visObjID.c_str());
  }
  else
	return;
	
  vector<Shape> shapes;
  vector<Color> colors;
  vector<float> colorDist, shapeDist;
	
  if(findAsserted(obj->phi, colors, shapes, colorDist, shapeDist))
  {
	log("Found asserted colors or shapes");
  }
  else
	return;

//  VisualObjectData &data = VisualObjectMap[_wmc.address.id];

//  CASTTime time=getCASTTime();

//  data.status= STABLE;
//  data.lastUpdateTime = time;
  //	queuesNotEmpty->post();proxyToAdd.push(obj.addr.id);

//  debug("A VisualObject ID %s ",data.addr.id.c_str());

  //  queuesNotEmpty->post();
}


bool VisualMediator::unionRef(FormulaPtr fp, string &unionID)
{
  Formula *f = &(*fp);

  if(typeid(*f).name() == "UnionRefProperty")
  {
	UnionRefPropertyPtr unif = dynamic_cast<UnionRefProperty*>(f);
	unionID =  unif->unionRef;	

	return true;
  }
  else if(typeid(*f).name() == "ComplexFormula")
  {
	ComplexFormulaPtr unif = dynamic_cast<ComplexFormula*>(f);
	vector<SuperFormulaPtr>::iterator it; 

	for(it = unif->formulae.begin(); it != unif->formulae.end(); it++)
	{
	  SuperFormulaPtr sf = *it;
  
	  if(unionRef(sf, unionID))
		return true;
	}
  
	unionID = "";
	return false;
	
  }
  else
  {
	unionID = "";
	return false;
  }

}


bool VisualMediator::findAsserted(FormulaPtr fp, vector<Color> &colors, vector<Shape> &shapes, vector<float> colorDist, vector<float> shapeDist)
{
  Formula *f = &(*fp);

  if(typeid(*f).name() == "ColorProperty")
  {
	ColorPropertyPtr color = dynamic_cast<ColorProperty*>(f);
	
	if(color->cstatus == assertion)
	{ 
	  colors.push_back(color->colorValue);
	
	  if(color->polarity)
		colorDist.push_back(color->prob);
	  else
		colorDist.push_back(-color->prob);

	  return true;
	}
	else
	  return false;
  }
  else if(typeid(*f).name() == "ShapeProperty")
  {
	ShapePropertyPtr shape = dynamic_cast<ShapeProperty*>(f);
	
	if(shape->cstatus == assertion)
	{
	  shapes.push_back(shape->shapeValue);
	
	  if(shape->polarity)
		shapeDist.push_back(shape->prob);
	  else
		shapeDist.push_back(-shape->prob);

	  return true;
	}
	else
	  return false;
  }
  else if(typeid(*f).name() == "ComplexFormula")
  {
	ComplexFormulaPtr unif = dynamic_cast<ComplexFormula*>(f);
	vector<SuperFormulaPtr>::iterator it;
	
	bool found = false;

	for(it = unif->formulae.begin(); it != unif->formulae.end(); it++)
	{
	  SuperFormulaPtr sf = *it;
  
	  if(findAsserted(sf, colors, shapes, colorDist, shapeDist))
		found = true;
	}
  
	return found;
	
  }
  else
	return false;

}



}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

