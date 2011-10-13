/**
 * @author Alen Vrecko
 * @date October 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNEngine.h"

#define DEFAULT_INSTANCE_TYPE "belief"
#define DEFAULT_INFERENCE_STEPS 50
#define DEFAULT_INFERENCE_PAUSE 200
#define DEFAULT_MAX_SAMPLES 4000
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
  
  m_query.clear();
  m_defaultInfSteps = DEFAULT_INFERENCE_STEPS;
  
  if ((it = _config.find("--inf-str")) != _config.end()) {
//	istringstream str(it->second);
	m_inferenceString = it->second;
  } else {
	m_inferenceString=""; //"-ms -i subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-out.mln -e subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-test.db -q student -maxSteps 1000 -burnMaxSteps 100";
  }
  
  if ((it = _config.find("--sa")) != _config.end()) {
	m_bindingSA=it->second;
  } else {
   m_bindingSA="binder.sa";
  }

  if(_config.find("--display") != _config.end())
  {
	doDisplay = true;
  }
  else
	doDisplay = false;
		
  if ((it = _config.find("--queries")) != _config.end()) {
		stringstream ss(it->second);
		string token;
	
	  while(getline(ss, token, ',')) {
	      m_query.push_back(token);
	  }
	}
	
  if ((it = _config.find("--id")) != _config.end()) {
	m_id=it->second;
  } else {
   m_id=getComponentID() ;
  }
  
  if ((it = _config.find("--instance-type")) != _config.end()) {
	m_instanceType=it->second;
  } else {
   m_instanceType=DEFAULT_INSTANCE_TYPE;
  }
  #ifdef FEAT_VISUALIZATION
	m_display.configureDisplayClient(_config);
  #endif
}

void MLNEngine::start()
{
  //must call super start to ensure that the reader sets up change
  //filters
//  BindingWorkingMemoryReader::start();

  log("MLNEngine active");


 // filters for belief updates
//  addChangeFilter(createGlobalTypeFilter<dBelief>(cdl::ADD),
//	  new MemberFunctionChangeReceiver<MLNEngine>(this,
//		&MLNEngine::newBelief));
		
 // filters for direct evidence updates
  addChangeFilter(createGlobalTypeFilter<Evidence>(cdl::ADD),
	  new MemberFunctionChangeReceiver<MLNEngine>(this,
		&MLNEngine::newEvidence));
		
  
  addChangeFilter(createGlobalTypeFilter<Query>(cdl::ADD),
	  new MemberFunctionChangeReceiver<MLNEngine>(this,
		&MLNEngine::newQuery));
		
    addChangeFilter(createGlobalTypeFilter<LearnWts>(cdl::ADD),
	  new MemberFunctionChangeReceiver<MLNEngine>(this,
		&MLNEngine::learnWts));
		
//  addChangeFilter(createGlobalTypeFilter<Belief>(cdl::OVERWRITE),
//	  new MemberFunctionChangeReceiver<MLNEngine>(this,
//		&MLNEngine::updatedBelief));

  m_oe = new OnlineEngine(m_inferenceString);
  m_oe->init();
  m_oe->saveAllCounts(true);
   
  log("MLN engine ID '%s': MRF initialized", m_id.c_str());
  
#ifdef FEAT_VISUALIZATION
	m_display.connectIceClient(*this);
    m_display.setClientData(this);
#endif  
}

void MLNEngine::runComponent()
{ 
  sleepComponent(1000);
  
  // Initialize the result WM entry
  m_resultWMId = newDataID();
  InferredResultPtr result = new InferredResult();
  result->engId = m_id;
  result->token = "";
  result->tokenSamples = 0;
  result->overallSamples = 0;
  
  addToWorkingMemory(m_resultWMId, m_bindingSA, result);

  // Init
  int tstep = 0;
  bool first = true;
  m_infSteps = m_defaultInfSteps;
  m_oe->setMaxInferenceSteps(m_infSteps);
  m_oe->setMaxBurnIn(0); 
  
  log("MLN engine ID '%s' initialized", m_id.c_str());
 
  while(isRunning())
  {
	
	while(!m_queryQueue.empty())
	{
	  log("A new query...");
	  QueryPtr q = m_queryQueue.front().query;
	
	  m_query.clear();
	  m_query=q->atoms;
	
	  m_queryQueue.front().status=USED;
	  m_removeQueue.push(m_queryQueue.front().addr);
	  m_queryQueue.pop();
	}
	
	while(!m_evidenceQueue.empty())
	{
	  log("Adding new evidence...");
	  EvidencePtr evd = m_evidenceQueue.front().evidence;

	  vector<Instance>::iterator it;
	  for(it=evd->newInstances.begin(); it!=evd->newInstances.end(); it++) {
		string placeh = m_oe->addInstance(m_instanceType, it->type);
		if(!placeh.empty())
		  addInstConst(it->name, placeh);
		else
		  log("WARNING: No free placeholder slots for instance %s", it->name.c_str());
	  }

	  m_oe->addTrueEvidence(replaceInstWithConst(evd->trueEvidence));
	  m_oe->addFalseEvidence(replaceInstWithConst(evd->falseEvidence));
	  m_oe->removeEvidence(replaceInstWithConst(evd->noEvidence));

	  m_evidenceQueue.front().status=USED;
	  m_removeQueue.push(m_evidenceQueue.front().addr);
	  m_evidenceQueue.pop();	

	  m_oe->adaptProbs(evd->prevInfSteps);
//	  m_oe->setMaxInferenceSteps(evd->initInfSteps);
	  m_oe->setMaxBurnIn(evd->burnInSteps);
	  m_oe->setExtPriors(replaceInstWithConst(evd->extPriors), evd->priorWts);
	  m_oe->resetPriors(replaceInstWithConst(evd->resetPriors));

	  for(it=evd->removeInstances.begin(); it!=evd->removeInstances.end(); it++) {	
		if(existsInstConst(it->name)) {
		  m_oe->removeInstance(getInstConst(it->name), it->type);
		  removeInstConst(it->name);
		} else
		  log("WARNING: No instance %s found", it->name.c_str());
		
	  }

  #ifdef FEAT_VISUALIZATION
	  ostringstream v11out;
	  m_oe->printNetwork(v11out);
	  m_display.setHtml("MRFView." + m_id, "Rules", "<pre>" + v11out.str() + "</pre>");
  #endif	  
	  
	  m_infSteps = evd->initInfSteps;
	  
	  if(!evd->setToken.empty()) {
		m_token = evd->setToken;
		m_tokenSamples = 0;
	  }
		
	  if(!evd->resetToken.empty() && evd->resetToken == m_token) {
		m_token = "";
		m_tokenSamples = 0;
	  }
	  
	  m_overallSamples = 0;
	}
	
	while(!m_learnWtsQueue.empty())
	{
	  log("Executing weight learning instruction...");
	  LearnWtsPtr lw = m_learnWtsQueue.front().learnWts;


	  m_learnWtsQueue.front().status=USED;
	  m_removeQueue.push(m_learnWtsQueue.front().addr);
	  m_learnWtsQueue.pop();
	
	  m_oe->genLearnWts(lw->trueEvidence, lw->falseEvidence, lw->noEvidence, 1);
	  	  
	#ifdef FEAT_VISUALIZATION
	  ostringstream v11out;
	  m_oe->printNetwork(v11out);
	  m_display.setHtml("MRFView." + m_id, "Rules", "<pre>" + v11out.str() + "</pre>");
	#endif
	}
	
  //  InferredResultPtr result = new InferredResult();
  //  result->engId = m_id;
	
  //  m_oe->setMaxInferenceSteps(5000);
	if(m_infSteps > 0 && m_overallSamples < DEFAULT_MAX_SAMPLES)
	{
	  if(!first) m_oe->restoreCnts();
	  m_oe->setMaxInferenceSteps(m_infSteps);
	  m_oe->infer(m_query, result->atoms, result->probs);
	  m_oe->saveCnts();
	
	  first=false;
	  m_tokenSamples += m_infSteps;
	  m_overallSamples += m_infSteps;

	
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
	
	  result->atoms = replaceConstWithInst(result->atoms);
	  result->token = m_token;
	  result->tokenSamples = m_tokenSamples;
	  result->overallSamples =  m_overallSamples;
	
	  overwriteWorkingMemory(m_resultWMId, m_bindingSA, result);
	}
	
	// restore the default values for inference samples and burn in samples
	m_infSteps = m_defaultInfSteps;	
	m_oe->setMaxBurnIn(0);	
	
	while(!m_removeQueue.empty())
	{
	  debug("Removing evidence or query entry"); 
//	  deleteFromWorkingMemory(m_removeQueue.front());
	
	  m_removeQueue.pop();
	}

//	debug("Samples: %i", m_oe->getNumSamples());
//	debug("Num true cnts: %i", m_oe->getClauseTrueCnts(0));
	 
	sleepComponent(DEFAULT_INFERENCE_PAUSE);
  }
  
  delete m_oe;
}

void MLNEngine::newEvidence(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new eng evidence entry. ID: %s ", _wmc.address.id.c_str());
  
  EvidencePtr evd; 
  
  try {
	evd = getMemoryEntry<Evidence>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the Evidence entry ID %s was removed before it could be processed.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a evidence update from WM. ID: %s", _wmc.address.id.c_str());
  
  if( evd->engId == m_id ) {
	EvidenceData data;
	data.status=NEW;
	data.evidence=evd;
	data.addr= _wmc.address;
	queueNewEvidence(data);	
//	deleteFromWorkingMemory(_wmc.address);
  }
  else {
	debug("Not my evidence.");
	return;
  }  
}


void MLNEngine::newQuery(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new MRF query entry. ID: %s ", _wmc.address.id.c_str());
  
  QueryPtr q; 
  
  try {
	q = getMemoryEntry<Query>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the Query entry ID %s was removed before it could be processed.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a query update from WM. ID: %s", _wmc.address.id.c_str());
  
  if( q->engId == m_id ) {
	QueryData data;
	data.status=NEW;
	data.query=q;
	data.addr= _wmc.address;
	queueNewQuery(data);
  }
  else {
	debug("Not mine.");
	return;
  }  
}

/*
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
  
  cdl::WorkingMemoryAddress refWmaAddr;
  
}
*/

void MLNEngine::learnWts(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new weight learn entry. ID: %s ", _wmc.address.id.c_str());
  
  LearnWtsPtr lw; 
  
  try {
	lw = getMemoryEntry<LearnWts>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the LearnWts entry ID %s was removed before it could be processed.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a weight learning instruction from WM. ID: %s", _wmc.address.id.c_str());
  
  if( lw->engId == m_id ) {
	LearnWtsData data;
	data.status=NEW;
	data.learnWts=lw;
	data.addr= _wmc.address;
	queueLearnWts(data);
  }
  else {
	debug("Not mine.");
	return;
  }  
}

vector<string> MLNEngine::replaceInstWithConst(vector<string> predicates)
{
  vector<string>::iterator pred;
  map<string,string>::iterator inst;
  
  for(inst=m_instances.begin(); inst!=m_instances.end(); inst++)
	for(pred=predicates.begin(); pred!=predicates.end(); pred++) {
	  
	  size_t pos = pred->find(inst->first.c_str());
	  if(pos!=string::npos) { 
		debug("Replacing belief ID %s with constant %s in predicate %s",
				inst->first.c_str(), inst->second.c_str(), pred->c_str());
		pred->replace(pos, inst->first.size(), inst->second.c_str());
		debug("...resulting in  predicate %s", pred->c_str());
	  }
	}
  return predicates;
}

vector<string> MLNEngine::replaceConstWithInst(vector<string> predicates)
{
  vector<string>::iterator pred;
  map<string,string>::iterator inst;
  
  for(inst=m_instances.begin(); inst!=m_instances.end(); inst++)
	for(pred=predicates.begin(); pred!=predicates.end(); pred++) {
	  
	  size_t pos = pred->find(inst->second.c_str());
	  if(pos!=string::npos)
		pred->replace(pos, inst->second.size(), inst->first.c_str());
	}
  return predicates;
}

}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

