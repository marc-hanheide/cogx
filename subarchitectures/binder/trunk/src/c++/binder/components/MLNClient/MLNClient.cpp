/**
 * @author Alen Vrecko
 * @date Mey 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNClient.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::MLNClient();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;

using namespace boost::interprocess;
using namespace boost::posix_time;

using namespace org::cognitivesystems::binder::mln;

void MLNClient::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
//  MLNListener::configure(_config);
  MLNEvdProvider::configure(_config);
    
  map<string,string>::const_iterator it;
  
  if ((it = _config.find("--eids")) != _config.end()) {
  	stringstream ss(it->second);
  	string token;
	
	while(getline(ss, token, ',')) {
        m_evdEngIds.push_back(token);
    }
  }
}

void MLNClient::start()
{
  //must call super start to ensure that the reader sets up change
  //filters  
//  MLNListener::start();
  MLNEvdProvider::start();
  
  log("MLNClient active");
}

void MLNClient::runComponent()
{
  sleep(3);
  
  // init mrf
  // leave the .db file from command line empty, because it does
  // not work online if the evidence is initialized there
//  setQuery(m_evdEngIds[0], "resolution");

  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized
  EvidencePtr evd = new Evidence();
  evd->engId = m_evdEngIds[0];
  evd->falseEvidence.push_back("percept(P1)");
  evd->falseEvidence.push_back("percept(P2)");
  evd->falseEvidence.push_back("percept(P3)");
  evd->initInfSteps = 400;
  evd->prevInfSteps = 0;
  evd->burnInSteps = 100;
  
  addToWorkingMemory(newDataID(), getBindingSA(), evd);
  
  int tstep = 0;
  while(isRunning())
  {
/*	log("Timestep: %i", tstep);  
	while(!qInfEmpty()) // here we process incoming inference
	{
	  log("Got an inference update...");
	  InferredResultPtr inf = qInfFront();
	
	  qInfPop();
	}*/
	
	// now we add as evidence an object that we perceive 
	// and its property
	if(tstep == 3)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_evdEngIds[0];
	  evd->trueEvidence.push_back("percept(P1)");
	  evd->trueEvidence.push_back("feature(P1,VGreen)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	}	
	
	// we add attributed property as evidence
	if(tstep == 7)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_evdEngIds[0];
	
	  evd->trueEvidence.push_back("attribute(LRed)");	
	  
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
  	  evd->burnInSteps = 100;
  	  
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	}
	

	// we now see another red object
	if(tstep == 25)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_evdEngIds[0];
	  evd->trueEvidence.push_back("percept(P2)");
//	  evd->trueEvidence.push_back("feature(P2,VRed)");
	  evd->extPriors.push_back("feature(P2,VRed)");
	  evd->priorWts.push_back(1);
	  evd->extPriors.push_back("feature(P2,VBlue)");
	  evd->priorWts.push_back(-1);
  //	evd->noEvidence.push_back("");
  	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
  	  evd->burnInSteps = 100;
	
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	}
	
	if(tstep == 40)
	{
	  LearnWtsPtr lw = new LearnWts();
	  lw->engId = m_evdEngIds[0];
	  lw->trueEvidence.push_back("resolution(P2)");
	  lw->trueEvidence.push_back("feature(P2,VRed)");
	  lw->falseEvidence.push_back("resolution(P1)");
	  lw->falseEvidence.push_back("resolution(P3)");
	
	  addToWorkingMemory(newDataID(), getBindingSA(), lw);
	}
	
	// and another red object
	if(tstep == 50)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_evdEngIds[0];
	  evd->trueEvidence.push_back("percept(P3)");
	  evd->trueEvidence.push_back("feature(P3,VRed)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	}
	
	if(tstep == 60)
	{
	  LearnWtsPtr lw = new LearnWts();
	  lw->engId = m_evdEngIds[0];
	  lw->trueEvidence.push_back("resolution(P3)");
	  lw->trueEvidence.push_back("feature(P2,VRed)");
	  lw->falseEvidence.push_back("resolution(P1)");
	  lw->falseEvidence.push_back("resolution(P2)");
	
	  addToWorkingMemory(newDataID(), getBindingSA(), lw);
	}
	
	// one of the red objects is removed
	if(tstep == 75)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_evdEngIds[0];
	  evd->falseEvidence.push_back("percept(P2)");
//	  evd->noEvidence.push_back("feature(P2,VRed)");
	  evd->resetPriors.push_back("feature(P2,VRed)");
	  evd->resetPriors.push_back("feature(P2,VBlue)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), getBindingSA(), evd);
	}
	
	tstep++;
	sleep(1);
  }

}

}

