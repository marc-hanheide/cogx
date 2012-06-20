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
 /* 
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
	};
 */
 
  int tstep = 0;
  while(isRunning())
  {
		log("Timestep: %i", tstep);  
	
/*	
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
*/
/*
	  class ReferenceResolutionRequest {
		string nom;
		string sort;
		ConstraintSeq constraints;
		de::dfki::lt::tr::dialogue::slice::time::Interval ival;
		
		class Constraint {
		string feature;
		string value;
	};
*/
	
	// now we add as evidence an object that we perceive 
	// and its property
		if(tstep == 30)
		{
			ReferenceResolutionRequestPtr rrr = new ReferenceResolutionRequest();
			rrr->nom = "";
			rrr->sort = "";
		
			ConstraintPtr con = new Constraint();
			con->feature = "color";
			con->value = "red";
		
			rrr->constraints.push_back(con);
		
			addToWorkingMemory(newDataID(), m_bindingSA, rrr);
		}
		
		if(tstep == 60)
		{
			ReferenceResolutionRequestPtr rrr = new ReferenceResolutionRequest();
			rrr->nom = "";
			rrr->sort = "";
		
			ConstraintPtr con = new Constraint();
			con->feature = "color";
			con->value = "blue";
		
			rrr->constraints.push_back(con);
		
			addToWorkingMemory(newDataID(), m_bindingSA, rrr);
		}
	
		tstep++;
		sleep(1);
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

