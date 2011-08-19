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
  
  if ((it = _config.find("--inf")) != _config.end()) {
//	istringstream str(it->second);
	m_inferenceString = it->second;
  } else {
	m_inferenceString=""; //"-ms -i subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-out.mln -e subarchitectures/binder.sa/src/c++/alchemy/exdata/univ-test.db -q student -maxSteps 1000 -burnMaxSteps 100";
  }
  
  if ((it = _config.find("--bsa")) != _config.end()) {
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
	
  if ((it = _config.find("--rid")) != _config.end()) {
	m_id=it->second;
  } else {
   m_id="mrf";
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

  const char *name = "MLNSemaphore";
  named_semaphore(open_or_create, name, 0);
  m_queuesNotEmpty = new named_semaphore(open_only, name);
  log("MLNEngine active");

  if (doDisplay)
  {
	cout << "MLNEngine active" << endl;
  }

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
  
  m_resultWMId = newDataID();
  InferredResultPtr result = new InferredResult();
  addToWorkingMemory(m_resultWMId, m_bindingSA, result);
  
  cout << "MRF initialized" << endl;
  
  m_query.clear();
  
#ifdef FEAT_VISUALIZATION
	m_display.connectIceClient(*this);
    m_display.setClientData(this);
#endif  
}

void MLNEngine::runComponent()
{

  int tstep = 0;
  bool first = true;
  m_infSteps = 100;
  m_oe->setMaxInferenceSteps(m_infSteps);
  m_oe->setMaxBurnIn(0);
  
  InferredResultPtr result = new InferredResult();
  result->engId = m_id;
 
  while(isRunning())
  {
  
	sleep(0.1);
	
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
	
	  m_oe->addTrueEvidence(evd->trueEvidence);
	  m_oe->addFalseEvidence(evd->falseEvidence);
	  m_oe->removeEvidence(evd->noEvidence);
	
	  m_evidenceQueue.front().status=USED;
	  m_removeQueue.push(m_evidenceQueue.front().addr);
	  m_evidenceQueue.pop();
	

	  m_oe->adaptProbs(evd->prevInfSteps);
	  m_oe->setMaxInferenceSteps(evd->initInfSteps);
	  m_oe->setMaxBurnIn(evd->burnInSteps);
	  m_oe->setExtPriors(evd->extPriors, evd->priorWts);
	  m_oe->resetPriors(evd->resetPriors);

	  
  #ifdef FEAT_VISUALIZATION
	  ostringstream v11out;
	  m_oe->printNetwork(v11out);
	  m_display.setHtml("MLNEngine", "MRF Rules", "<pre>" + v11out.str() + "</pre>");
  #endif
	}
	
	while(!m_learnWtsQueue.empty())
	{
	  log("Executing weight learning instrction...");
	  LearnWtsPtr lw = m_learnWtsQueue.front().learnWts;


	  m_learnWtsQueue.front().status=USED;
	  m_removeQueue.push(m_learnWtsQueue.front().addr);
	  m_learnWtsQueue.pop();
	
	  m_oe->genLearnWts(lw->trueEvidence, lw->falseEvidence, lw->noEvidence, 1);
	  	  
  #ifdef FEAT_VISUALIZATION
	  ostringstream v11out;
	  m_oe->printNetwork(v11out);
	  m_display.setHtml("MLNEngine", "eng Rules", "<pre>" + v11out.str() + "</pre>");
  #endif
	}
	
  //  InferredResultPtr result = new InferredResult();
  //  result->engId = m_id;
	
  //  m_oe->setMaxInferenceSteps(5000);
	if(!first) m_oe->restoreCnts();
	
	m_oe->infer(m_query, result->atoms, result->probs);
	m_oe->saveCnts();
	first=false;

	m_oe->setMaxInferenceSteps(m_infSteps);
	m_oe->setMaxBurnIn(0);
	
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
	
	while(!m_removeQueue.empty())
	{
	  debug("Removing evidence or query entry"); 
	  deleteFromWorkingMemory(m_removeQueue.front());
	
	  m_removeQueue.pop();
	}
	
	cout << "Samples: " << m_oe->getNumSamples() << endl;
	cout << "Num true cnts: " << m_oe->getClauseTrueCnts(0) << endl;
	
  //	ptime t(second_clock::universal_time() + seconds(1));

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
//	deleteFromWorkingMemory(_wmc.address);
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
//	deleteFromWorkingMemory(_wmc.address);
  }
  else {
	debug("Not mine.");
	return;
  }  
}

}
/* vim:set fileencoding=utf-8 sw=2 ts=4 noet:vim*/

