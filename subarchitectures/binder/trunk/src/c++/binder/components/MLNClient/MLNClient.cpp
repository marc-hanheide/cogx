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

  distributeQuery("resolution");
  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized

  EvidencePtr evd = new Evidence();

  evd->trueEvidence.push_back("attribute(LRed)");
  evd->falseEvidence.push_back("percept(PH1)");
  evd->falseEvidence.push_back("percept(PH2)");
//  evd->falseEvidence.push_back("percept(PH3)");
//  evd->falseEvidence.push_back("percept(PH4)");
//  evd->falseEvidence.push_back("percept(PH5)");
//  evd->falseEvidence.push_back("percept(PH6)");
  evd->initInfSteps = 400;
  evd->prevInfSteps = 0;
  evd->burnInSteps = 100;

  distributeEvd(evd);
  
  sleep(5);
  
  evd = new Evidence();
  Instance i;
  i.name="T:T";
  i.type="perc";
  evd->newInstances.push_back(i);

  evd->trueEvidence.push_back("color(T:T,V_red)");
  evd->initInfSteps = 400;
  evd->prevInfSteps = 0;
  evd->burnInSteps = 100;
  
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
	
	
	if(pendingEvdChanges()) {
		resetEvdChanges();
		
		m_filtFacts = filterFacts(getRawFacts());
		if(m_filtFacts.size()) {
			log("Processing belief changes");
			EvidencePtr evd = new Evidence();
			
			if(getEvdChanges(m_filtFacts, m_oldFacts, evd)) {
				distributeEvd(evd);
				m_oldFacts = m_filtFacts;
			    log("New MLN evidence provided");
			}
		} else
			log("No relevant beliefs found");	
	}
	sleepComponent(200);
  }

}

}

