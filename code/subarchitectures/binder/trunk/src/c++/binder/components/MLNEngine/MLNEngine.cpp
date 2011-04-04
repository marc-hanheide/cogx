/**
 * @author Alen Vrecko
 * @date October 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNEngine.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::MLNEngine();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;

using namespace boost::interprocess;
using namespace boost::posix_time;

using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::distribs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace org::cognitivesystems::binder::mln;

void MLNEngine::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
  ManagedComponent::configure(_config);
  
  map<string,string>::const_iterator it;
  
  if (_config.find("-inf") != _config.end()) {
	m_inferenceString=_config.find("-inf")->second;
  } else {
	m_inferenceString="-p -testConvergence true -i subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-out.mln -e subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-test.db -q advisedBy,student -maxSteps 1000 -burnMaxSteps 100";
  }
  
  if (_config.find("--bsa") != _config.end()) {
	m_bindingSA=_config.find("--bsa")->second;
  } else {
   m_bindingSA="binder.sa";
  }

  if((it = _config.find("--display")) != _config.end())
  {
	doDisplay = true;
  }
  else
	doDisplay = false;
  
  m_id="mrf";
  
}

void MLNEngine::start()
{
  //must call super start to ensure that the reader sets up change
  //filters
//  BindingWorkingMemoryReader::start();

  const char *name = "MLNSemaphore";
  named_semaphore(open_or_create, name, 0);
  m_queuesNotEmpty = new named_semaphore(open_only, name);
  log("MLNEngine active");

  if (doDisplay)
  {
	cout << "MLNEngine active" << endl;
  }

 // filters for belief updates
  addChangeFilter(createGlobalTypeFilter<dBelief>(cdl::ADD),
	  new MemberFunctionChangeReceiver<MLNEngine>(this,
		&MLNEngine::newBelief));
		
 // filters for direct evidence updates
  addChangeFilter(createGlobalTypeFilter<Evidence>(cdl::ADD),
	  new MemberFunctionChangeReceiver<MLNEngine>(this,
		&MLNEngine::newEvidence));
  
//  addChangeFilter(createGlobalTypeFilter<Belief>(cdl::OVERWRITE),
//	  new MemberFunctionChangeReceiver<MLNEngine>(this,
//		&MLNEngine::updatedBelief));

  m_oe = new OnlineEngine(m_inferenceString);
  m_oe->init();
  
  m_resultWMId = newDataID();
  ResultPtr result = new Result();
  addToWorkingMemory(m_resultWMId, m_bindingSA, result);
  
  cout << "MRF initialized" << endl;
}

void MLNEngine::runComponent()
{

 /* 
  
  log("sumbitting a fake belief 1");
  
  BeliefPtr tb = new Belief();  
  tb->ags = new AttributedAgentStatus();
  
  ComplexFormulaPtr cf = new ComplexFormula();
  
  UnionRefPropertyPtr un =  new UnionRefProperty();
  un->beliefRef = "whatever";
  
  cf->formulae.push_back(un);
  
  ShapePropertyPtr sh = new ShapeProperty();
  sh->shapeValue = cubic;
  sh->polarity = true;
  sh->prob = 1.0f;
  sh->cstatus = assertion;
  
  cf->formulae.push_back(sh);
  
  tb->phi = cf;
  tb->id = newDataID();
  
  addToWorkingMemory(tb->id, m_bindingSA, tb);
  
  
  log("sumbitting a fake belief 2");
  
  tb = new Belief();
  tb->ags = new AttributedAgentStatus();

  cf = new ComplexFormula();
  
  un =  new UnionRefProperty();
  un->beliefRef = "whatever";
  
  cf->formulae.push_back(un);
  
  ColorPropertyPtr co = new ColorProperty();
  co->colorValue = red;
  co->polarity = true;
  co->prob = 1.0f;
  co->cstatus = proposition;
  
  cf->formulae.push_back(co);
  
  tb->phi = cf;
  tb->id = newDataID();
  
  addToWorkingMemory(tb->id, m_bindingSA, tb);
  */

  int tstep = 0;
  
  while(isRunning())
  {
  
  sleep(1);
  
  while(!m_evidenceQueue.empty())
  {
	log("Adding new evidence...");
	EvidencePtr evd = m_evidenceQueue.front();
	
	m_oe->addTrueEvidence(evd->trueEvidence);
	m_oe->addFalseEvidence(evd->falseEvidence);
	m_oe->removeEvidence(evd->oldEvidence);
	
	m_evidenceQueue.pop();
  }
  
  ResultPtr result = new Result();
  result->mrfId = m_id;
  
  m_oe->setMaxInferenceSteps(100);
  m_oe->infer(result->atoms, result->probs);
  
  if(doDisplay) {
    // Print true atoms
	cout << endl;
	cout << "Time step " << ++tstep << " non-zero atoms: " << endl;
	vector<string>::const_iterator it = result->atoms.begin();
	vector<float>::const_iterator probIt = result->probs.begin();
	for (; it != result->atoms.end(); it++)
	{
	  cout << (*it) << " " << (*probIt) << endl;
	  probIt++;
	}
  }
  
  overwriteWorkingMemory(m_resultWMId, m_bindingSA, result);
  
//	ptime t(second_clock::universal_time() + seconds(1));
/*
	if (queuesNotEmpty->timed_wait(t))"MLNEngine active"
	{
	  log("Got something in my queues");

	  if(!proxyToAdd.empty())
	  { 
		log("");
		VisualObjectData &data = VisualObjectMap[proxyToAdd.front()];

		if(data.status == OBJECT)
		{
		  data.status = PROXY;
		  try
		  {
			VisualObjectPtr objPtr = getMemoryEntry<VisionData::VisualObject>(datattributedEpiStatusa.addr);

			WorkingMemoryPointerPtr origin = createWorkingMemoryPointer(getSubarchitectureID(), data.addr.id, "VisualObject");

			FeatureValuePtr value = createUnknownValue(1.00f); //createStringValue (objPtr->label.c_str(), objPtr->labelConfidence);
			//FeatureValuePtr value = createStringValue ("thing", 1.00f);
			FeaturePtr label = createFeatureWithUniqueFeatureValue ("obj_label", value);

			FeatureValuePtr salvalue = createStringValue ("high", 1.00f);
			FeaturePtr saliency = createFeatureWithUniqueFeatureValue ("saliency", salvalue);

			ProxyPtr proxy = createNewProxy (origin, 1.0f);

			addFeatureToProxy (proxy, label);
			addFeatureToProxy (proxy, saliency);
			addFeatureListToProxy(proxy, objPtr->labels, objPtr->distribution);

			addProxyToWM(proxy);
			
			if(!first && existsOnWorkingMemory(m_salientObjID, m_bindingSA))
			{
			  
			  ProxyPtr salProxy = getMemoryEntry<Proxy>(m_salientObjID, m_bindingSA);
			  
			  WorkingMemoryPointerPtr ovrOrigin = createWorkingMemoryPointer(getSubarchitectureID(), salProxy->origin->address.id, "VisualObject");
			  ProxyPtr ovrProxy = createNewProxy (ovrOrigin, 1.0f);
			  
			  vsitbeliefsector<FeaturePtr>::iterator it;
			  
			  for(it = salProxy->features.begin(); it != salProxy->features.end(); it++)
			   if(string((*it)->featlabel) == "saliency")
			   {
				  addFeatureToProxy(ovrProxy, createFeatureWithUniqueFeatureValue ("saliency", createStringValue ("low", 1.00f)));
			   }
			   else
			   {
				  addFeatureToProxy(ovrProxy, *it);
			   }
			  
			  ovrProxy->entityID = salProxy->entityID;
			  overwriteProxyInWM(ovrProxy);		 
			}
					
			m_salientObjID = data.proxyId = proxy->entityID;
			first = false;

			log("A visual proxy ID %s added for visual object ID %s",
				proxy->entityID.c_str(), data.addr.id.c_str());

		  }
		  catch (DoesNotExistOnWMException e)
		  {
			log("VisualObject ID: %s was removed before it could be processed", data.addr.id.c_str());
		  }
		}

		proxyToAdd.pop();
	  }
	  else if(!proxyToUpdate.empty())
	  {
		log("An update proxy instruction"); 
		VisualObjectData &data = VisualObjectMap[proxyToUpdate.front()];
	
		if(data.status == PROXY)
		{
		  try
		  {
			VisualObjectPtr objPtr = getMemoryEntry<VisionData::VisualObject>(data.addr);

			WorkingMemoryPointerPtr origin = createWorkingMemoryPointer(getSubarchitectureID(), data.addr.id, "VisualObject");
			
			// check if we can reliable recognise the color
			bool known = false;
			for(int i=0; i<objPtr->distribution.size(); i++)
				if(objPtr->labels[i] <= 7 && objPtr->distribution[i] > 0.7)
				  known = true;
			
			FeatureValuePtr value;	  
			if(known)
			   value = createStringValue ("thing", 1.00f);
			else
			  value = createUnknownValue(1.00f);
			
			FeaturePtr label = createFeatureWithUniqueFeatureValue ("obj_label", value);

			ProxyPtr proxy = createNewProxy (origin, 1.0f);

			addFeatureToProxy (proxy, label);
			addFeatureListToProxy(proxy, objPtr->labels, objPtr->distribution);
			
			proxy->entityID = data.proxyId;
			
			string salval;
			if(m_salientObjID == proxy->entityID)
			  salval = "high";
			 else
			  salval = "low";
			
			FeaturePtr saliency = createFeatureWithUniqueFeatureValue ("saliency", createStringValue (salval, 1.0f));
			addFeatureToProxy (proxy, saliency);
			
			overwriteProxyInWM(proxy);
			
			log("A visual proxy ID %s was updated following the visual object ID %s",
				proxy->entityID.c_str(), data.addr.id.c_str());
				
			checkDistribution4Clarification(proxy->entityID, objPtr->labels, objPtr->distribution);

		  }
		  catch (DoesNotExistOnWMException e)
		  {
			log("VisualObject ID: %s was removed before it could be processed", data.addr.id.c_str());
		  }
		}
		else if(data.status == OBJECT)
		{
		  proxyToUpdate.push(data.addr.id);
		  queuesNotEmpty->post();
		  log("No updating, waiting for the proxy to be created");
		}
		proxyToUpdate.pop();
	  }
	  else if(!proxyToDelete.empty())
	  {
		log("A delete proxy instruction");
		VisualObjectData &obj = VisualObjectMap[proxyToDelete.front()];

		  if(obj.status == PROXY)
		  {
			obj.status == DELETED;
			try
			{  
			  deleteEntityInWM(obj.proxyId);
			  
			  log("A proxy deleted ID: %s", obj.proxyId.c_str());
//			  VisualObjectMap.erase(proxyToDelete.front());
			}
			catch (DoesNotExistOnWMException e)
			{
			  log("WARNING: Proto-object ID %s already removed", obj.proxyId);
			}
		  }

		  proxyToDelete.pop();
	  }
	}
	
*/
	//    else
	//		log("Timeout");   
  }

  log("Removing semaphore ...");
  m_queuesNotEmpty->remove("MLNSemaphore");
  delete m_queuesNotEmpty;

  if (doDisplay)
  {
  }
}

void MLNEngine::newEvidence(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new MRF evidence entry. ID: %s ", _wmc.address.id.c_str());
  
  EvidencePtr evd; 
  
  try {
	evd = getMemoryEntry<Evidence>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the entry Evidence ID %s was removed before it could be processed.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a evidence update from WM. ID: %s", _wmc.address.id.c_str());
  
  if( evd->mrfId == m_id ) {
	queueNewEvidence(evd);
	deleteFromWorkingMemory(_wmc.address);
  }
  else {
	debug("Wrong MRF.");
	return;
  }  
};


void MLNEngine::newBelief(const cdl::WorkingMemoryChange & _wmc)
{
  log("A belief was updated. ID: %s SA: %s", _wmc.address.id.c_str(), _wmc.address.subarchitecture.c_str());
  
  dBeliefPtr belief; 
  
  try {
	belief = getMemoryEntry<dBelief>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: belief ID %s was removed before it could be processed.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a belief from WM. ID: %s", _wmc.address.id.c_str());  
/*
  if(!epistemicStatus<SharedEpistemicStatus>(belief))
  {
	log("WARNING: the epistemic status is not a shared one. No learning case.");
	return;
  }
*/
  
  cdl::WorkingMemoryAddress refWmaAddr;
  
//  if(!pointedCASTAddr("about", belief, refWmaAddr))
//  {
//	log("WARNING: no reference property in the presupposed belief. No learning case.");
//	return;
//  }
}

void MLNEngine::queueNewEvidence(EvidencePtr evd)
{
  m_evidenceQueue.push(evd);
}

/*
void MLNEngine::updatedLearningTask(const cdl::WorkingMemoryChange & _wmc)
{
  log("A learning Task was updated. ID: %s SA: %s", _wmc.address.id.c_str(), _wmc.address.subarchitecture.c_str());
  
  VisualLearnerLearningTaskPtr task =
	getMemoryEntry<VisualLearnerLearningTask>(_wmc.address);
	
  debug("Got an overwritten learning task ID: %s", _wmc.address.id.c_str());
  
  BeliefPtr belief = getMemoryEntry<Belief>(task->beliefId, m_bindingSA);
  GroundedBeliefPtr grobelief = new GroundedBelief();
  grobelief->sigma = belief->sigma;
  grobelief->phi = belief->phi;
	removeLearnedAssertions(grobelief->phi, task);
	
	MutualAgentStatusPtr agstatus = new MutualAgentStatus();
	agstatus->ags.push_back(new Agent("robot"));
	agstatus->ags.push_back(new Agent("human"));
  grobelief->ags = agstatus;
  grobelief->grounding = new Ground();
  grobelief->grounding->gstatus = assertionVerified;
  grobelief->grounding->modality = getSubarchitectureID();
//  grobelief->grounding->indexSet
  grobelief->grounding->reason = new SuperFormula();
  grobelief->timeStamp = getCASTTime();
  grobelief->id = belief->id;
  
  overwriteWorkingMemory(task->beliefId, m_bindingSA, grobelief);
  log("Updated the belief ID %s with grounding", belief->id.c_str());

  deleteFromWorkingMemory(_wmc.address);
  log("Removed the learning task ID %s", _wmc.address.id.c_str());
  
  removeChangeFilter(TaskFilterMap[_wmc.address.id]);
  TaskFilterMap.erase(_wmc.address.id);
}
*/

/*
bool MLNEngine::pointedCASTAddr(std::string key, map<string,ProbDistribution> distibutions, cast::cdl::WorkingMemoryAddress &refWmaAddr)
{
  // BasicProbDistribution
  map<string,ProbDistribution>::iterator bpd = distributions.find(key);
  
  if(bpd==distributions.end())
	return false;
  else if(bpd->key==key)
	{
	  refWmaAddr=bpd->values->values.begin().val->pointer;
	  return true;
	}
	else
	{
	  log("Basic distribution key did not match the Distributions key");
	  return false;
	}	  
}


bool MLNEngine::pointedCASTAddr(std::string key, ProbDistribution distribution, cast::cdl::WorkingMemoryAddress &refWmaAddr)
{
  ProbDistribution *pd= &(*distribution);
  
  if(typeid(*pd) == typeid(CondIndependentDistribs))
	return pointedCASTAddr(key, pd->distributions, refWmaAddr)
  else
  {
	log("UNEXPECTED STRUCTURE in belief content");
	return false;
  }
}

bool MLNEngine::pointedCASTAddr(std::string key, dBelief belief, cast::cdl::WorkingMemoryAddress &refWmaAddr)
{
  return pointedCASTAddr(key, belief->content, refWmaAddr);
}
*/

}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

