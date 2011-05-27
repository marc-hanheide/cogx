/**
 * @author Alen Vrecko
 * @date Mey 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNTester.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::MLNTester();
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

void MLNTester::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
  ManagedComponent::configure(_config);
  
  map<string,string>::const_iterator it;
  
  
  if ((it = _config.find("--bsa")) != _config.end()) {
	m_bindingSA = it->second;
  } else {
   m_bindingSA="binder.sa";
  }

  if((it = _config.find("--display")) != _config.end())
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
}

void MLNTester::start()
{
  //must call super start to ensure that the reader sets up change
  //filters  
//  BindingWorkingMemoryReader::start();


  if (doDisplay)
  {
	cout << "MLNTester active" << endl;
  }

 // filters for belief updates
  addChangeFilter(createGlobalTypeFilter<Result>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<MLNTester>(this,
		&MLNTester::newResult));
		
  
  cout << "MLNTester initialized" << endl;
}

void MLNTester::runComponent()
{
  sleep(3);
  
  // init MRF
  // leave the .db file from command line empty, because it does
  // not work online if the evidence is initialized there
  QueryPtr q = new Query();
  q->mrfId = m_id;
  q->atoms.push_back("resolution");
	
  addToWorkingMemory(newDataID(), m_bindingSA, q);

  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized
  EvidencePtr evd = new Evidence();
  evd->mrfId = m_id;
  evd->falseEvidence.push_back("percept(P1)");
  evd->falseEvidence.push_back("percept(P2)");
  evd->falseEvidence.push_back("percept(P3)");
  
  addToWorkingMemory(newDataID(), m_bindingSA, evd);
  
  int tstep = 0;
  while(isRunning())
  {
	log("Timestep: %i", tstep);  
	while(!m_resultQueue.empty()) // here we process incoming results
	{
	  log("Got a new result...");
	  ResultPtr res = m_resultQueue.front();
	
	  if(doDisplay) {
		// Print true atoms
		cout << endl;
		cout << "Time step " << tstep << " non-zero atoms: " << endl;
		vector<string>::const_iterator it = res->atoms.begin();
		vector<float>::const_iterator probIt = res->probs.begin();
		for (; it != res->atoms.end(); it++)
		{
		  cout << (*it) << " " << (*probIt) << endl;
		  probIt++;
		}
	  }
	
	  m_resultQueue.pop();
	}
	
	// now we add as evidence an object that we perceive 
	// and its property
	if(tstep == 3)
	{
	  EvidencePtr evd = new Evidence();
	  evd->mrfId = m_id;
	  evd->trueEvidence.push_back("percept(P1)");
	  evd->trueEvidence.push_back("feature(P1,VGreen)");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}	
	
	// we add attributed property as evidence
	if(tstep == 7)
	{
	  EvidencePtr evd = new Evidence();
	  evd->mrfId = m_id;
	
	  evd->trueEvidence.push_back("attribute(LRed)");	
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	

	// we now see another red object
	if(tstep == 25)
	{
	  EvidencePtr evd = new Evidence();
	  evd->mrfId = m_id;
	  evd->trueEvidence.push_back("percept(P2)");
	  evd->trueEvidence.push_back("feature(P2,VRed)");
  //	evd->removeEvidence.push_back("");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	
	// and another red object
	if(tstep == 50)
	{
	  EvidencePtr evd = new Evidence();
	  evd->mrfId = m_id;
	  evd->trueEvidence.push_back("percept(P3)");
	  evd->trueEvidence.push_back("feature(P3,VRed)");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	
	// one of the red objects is removed
	if(tstep == 75)
	{
	  EvidencePtr evd = new Evidence();
	  evd->mrfId = m_id;
	  evd->falseEvidence.push_back("percept(P2)");
	  evd->removeEvidence.push_back("feature(P2,VRed)");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	
	tstep++;
	sleep(1);
  }

  if (doDisplay)
  {
  }
}

void MLNTester::newResult(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new Result entry. ID: %s ", _wmc.address.id.c_str());
  
  ResultPtr res; 
  
  try {
	res = getMemoryEntry<Result>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the entry Result ID %s not in WM.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a result update from WM. ID: %s", _wmc.address.id.c_str());
  
  if( res->mrfId == m_id ) {
	queueNewResult(res);
  }
  else {
	debug("Wrong MRF.");
	return;
  }  
};


void MLNTester::queueNewResult(ResultPtr res)
{
  m_resultQueue.push(res);
}



}

