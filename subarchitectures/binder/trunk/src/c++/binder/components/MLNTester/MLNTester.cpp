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
  addChangeFilter(createGlobalTypeFilter<InferredResult>(cdl::OVERWRITE),
	  new MemberFunctionChangeReceiver<MLNTester>(this,
		&MLNTester::newInferredResult));
		
  
  cout << "MLNTester initialized" << endl;
}

void MLNTester::runComponent()
{
  sleep(3);
  
  // init mrf
  // leave the .db file from command line empty, because it does
  // not work online if the evidence is initialized there
  QueryPtr q = new Query();
  q->engId = m_id;
  q->atoms.push_back("resolution");
	
  addToWorkingMemory(newDataID(), m_bindingSA, q);

  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized
  EvidencePtr evd = new Evidence();
  evd->engId = m_id;

  evd->falseEvidence.push_back("percept(PH1)");
  evd->falseEvidence.push_back("percept(PH2)");
  evd->falseEvidence.push_back("percept(PH3)");
  evd->falseEvidence.push_back("percept(PH4)");
  evd->falseEvidence.push_back("percept(PH5)");
  evd->falseEvidence.push_back("percept(PH6)");
  evd->initInfSteps = 400;
  evd->prevInfSteps = 0;
  evd->burnInSteps = 100;
  
  addToWorkingMemory(newDataID(), m_bindingSA, evd);
  
  int tstep = 0;
  while(isRunning())
  {
	log("Timestep: %i", tstep);  
	while(!m_resultQueue.empty()) // here we process incoming results
	{
	  log("Got a new result...");
	  InferredResultPtr res = m_resultQueue.front();
	
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
	  Instance i;
	  i.name="P1";
	  i.type="perc";
	  evd->newInstances.push_back(i);
	  evd->engId = m_id;
//	  evd->trueEvidence.push_back("percept(P1)");
	  evd->trueEvidence.push_back("feature(P1,VGreen)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}	
	
	// we add attributed property as evidence
	if(tstep == 7)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_id;
	
	  evd->trueEvidence.push_back("attribute(LRed)");	
	  
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
  	  evd->burnInSteps = 100;
  	  
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	

	// we now see another red object
	if(tstep == 25)
	{
	  EvidencePtr evd = new Evidence();
	  Instance i;
	  i.name="P2";
	  i.type="perc";
	  evd->newInstances.push_back(i);
	  evd->engId = m_id;
//	  evd->trueEvidence.push_back("percept(P2)");
//	  evd->trueEvidence.push_back("feature(P2,VRed)");
	  evd->extPriors.push_back("feature(P2,VRed)");
	  evd->priorWts.push_back(1);
	  evd->extPriors.push_back("feature(P2,VBlue)");
	  evd->priorWts.push_back(-1);
  //	evd->noEvidence.push_back("");
  	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
  	  evd->burnInSteps = 100;
	
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
/*	
	if(tstep == 40)
	{
	  LearnWtsPtr lw = new LearnWts();
	  lw->engId = m_id;
	  lw->trueEvidence.push_back("resolution(P2)");
	  lw->trueEvidence.push_back("feature(P2,VRed)");
	  lw->falseEvidence.push_back("resolution(P1)");
	  lw->falseEvidence.push_back("resolution(P3)");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, lw);
	}
*/	
	// and another red object
	if(tstep == 50)
	{
	  EvidencePtr evd = new Evidence();
	  Instance i;
	  i.name="P3";
	  i.type="perc";
	  evd->newInstances.push_back(i);
	  evd->engId = m_id;
//	  evd->trueEvidence.push_back("percept(P3)");
	  evd->trueEvidence.push_back("feature(P3,VRed)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
/*	
	if(tstep == 60)
	{
	  LearnWtsPtr lw = new LearnWts();
	  lw->engId = m_id;
	  lw->trueEvidence.push_back("resolution(P3)");
	  lw->trueEvidence.push_back("feature(P2,VRed)");
	  lw->falseEvidence.push_back("resolution(P1)");
	  lw->falseEvidence.push_back("resolution(P2)");
	
	  addToWorkingMemory(newDataID(), m_bindingSA, lw);
	}
*/	
	// one of the red objects is removed
	if(tstep == 75)
	{
	  EvidencePtr evd = new Evidence();
	  evd->engId = m_id;
	  Instance inst;
	  inst.name="P2";
	  inst.type="perc";
	  evd->removeInstances.push_back(inst);
//	  evd->falseEvidence.push_back("percept(P2)");
//	  evd->noEvidence.push_back("feature(P2,VRed)");
	  evd->resetPriors.push_back("feature(P2,VRed)");
	  evd->resetPriors.push_back("feature(P2,VBlue)");
	  evd->initInfSteps = 400;
	  evd->prevInfSteps = 0;
	  evd->burnInSteps = 100;
  
	  addToWorkingMemory(newDataID(), m_bindingSA, evd);
	}
	
	tstep++;
	sleep(1);
  }

  if (doDisplay)
  {
  }
}

void MLNTester::newInferredResult(const cdl::WorkingMemoryChange & _wmc)
{
  log("A new InferredResult entry. ID: %s ", _wmc.address.id.c_str());
  
  InferredResultPtr res; 
  
  try {
	res = getMemoryEntry<InferredResult>(_wmc.address);
  }
  catch (DoesNotExistOnWMException e) {
	log("WARNING: the entry InferredResult ID %s not in WM.", _wmc.address.id.c_str());
	return;
  }
  		
  debug("Got a result update from WM. ID: %s", _wmc.address.id.c_str());
  
  if( res->engId == m_id ) {
	queueNewInferredResult(res);
  }
  else {
	debug("Wrong MRF.");
	return;
  }  
};


void MLNTester::queueNewInferredResult(InferredResultPtr res)
{
  m_resultQueue.push(res);
}



}

