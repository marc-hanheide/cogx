/**
 * @author Alen Vrecko
 * @date Mey 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNBeliefEvdFilter.h"

#define DEFAULT_SLEEP_PERIOD 100
#define DEFAULT_INFERENCE_STEPS 800
#define DEFAULT_BURNIN_STEPS 300

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::MLNBeliefEvdFilter();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;

using namespace org::cognitivesystems::binder::mln;

void MLNBeliefEvdFilter::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
  AbsMLNEvdFilter::configure(_config);
    
  map<string,string>::const_iterator it;
/* 
  if ((it = _config.find("--eids")) != _config.end()) {
  	stringstream ss(it->second);
  	string token;
	
	while(getline(ss, token, ',')) {
        m_evdEngIds.push_back(token);
    }
  }
 */
}

void MLNBeliefEvdFilter::start()
{
  //must call super start to ensure that the reader sets up change
  //filters  
//  MLNListener::start();
  AbsMLNEvdFilter::start();
  
  log("MLNBeliefEvdFilter active");
}

void MLNBeliefEvdFilter::runComponent()
{
  sleep(3);
  
  // init mrf
  // leave the .db file from command line empty, because it does
  // not work online if the evidence is initialized there

//  distributeQuery("resolution");
  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized
  
  EvidencePtr evd = new Evidence();

 // evd->trueEvidence.push_back("color_constraint(A_red)");
  evd->falseEvidence.push_back("belief(Blf1)");
  evd->falseEvidence.push_back("belief(Blf2)");
  evd->falseEvidence.push_back("belief(Blf3)");
  evd->falseEvidence.push_back("belief(Blf4)");
  evd->falseEvidence.push_back("belief(Blf5)");
  evd->falseEvidence.push_back("belief(Blf6)");
/*  
  evd->falseEvidence.push_back("epstatus(Blf1,Attributed)");
  evd->falseEvidence.push_back("epstatus(Blf2,Attributed)");
  evd->falseEvidence.push_back("epstatus(Blf3,Attributed)");
  evd->falseEvidence.push_back("epstatus(Blf4,Attributed)");
  evd->falseEvidence.push_back("epstatus(Blf5,Attributed)");
  evd->falseEvidence.push_back("epstatus(Blf6,Attributed)");
  
  evd->falseEvidence.push_back("epstatus(Blf1,Shared)");
  evd->falseEvidence.push_back("epstatus(Blf2,Shared)");
  evd->falseEvidence.push_back("epstatus(Blf3,Shared)");
  evd->falseEvidence.push_back("epstatus(Blf4,Shared)");
  evd->falseEvidence.push_back("epstatus(Blf5,Shared)");
  evd->falseEvidence.push_back("epstatus(Blf6,Shared)");
*/  
  evd->initInfSteps = DEFAULT_INFERENCE_STEPS;
  evd->prevInfSteps = 0;
  evd->burnInSteps = DEFAULT_BURNIN_STEPS;

  distributeEvd(evd);
  
  while(isRunning())	  
  {
/*	log("Timestep: %i", tstep);  
	while(!qInfEmpty()) // here we process incoming inference
	{
	  log("Got an inference update...");
	  InferredResultPtr inf = qInfFront();
	
	  qInfPop();
	}*/
	
	
	if(pendingFactChanges()) {
		resetFactChanges();
		
		m_filtFacts = filterFacts(getRawFacts());
//		if(m_filtFacts.size()) {
			log("Processing belief changes");
			EvidencePtr evd = new Evidence();
			
			if(getEvdChanges(m_filtFacts, m_oldFacts, evd)) {
				distributeEvd(evd, "");
				m_oldFacts = m_filtFacts;
			  log("New MLN evidence provided");
			} else
				log("No relevant beliefs found");	
	}
	sleepComponent(DEFAULT_SLEEP_PERIOD);
  }

}

}

