/**
 * @author Alen Vrecko
 * @date Mey 2011
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "MLNRefResolutionClient.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::MLNRefResolutionClient();
  }
}

namespace cast
{

using namespace std;
using namespace cdl;

using namespace org::cognitivesystems::binder::mln;

void MLNRefResolutionClient::newConstraints(const cdl::WorkingMemoryChange & _wmc)
{
	log("A new ReferenceResolutionRequest WM entry ID %s ", _wmc.address.id.c_str());
	
	ReferenceResolutionRequestPtr req; 
	
	try {
	  req = getMemoryEntry<ReferenceResolutionRequest>(_wmc.address);
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: the entry ReferenceResolutionRequest ID %s not in WM.", _wmc.address.id.c_str());
	  return;
	}
	
	EvidencePtr evd = new Evidence();
	evd->noEvidence = m_currentConstraints;
	m_currentConstraints.clear();
	
	vector<ConstraintPtr>::iterator it;
	vector<ConstraintPtr> constraints = req->constraints;
	for(it=constraints.begin(); it != constraints.end(); it++) {
		if(m_supportedConstraintTypes.find((*it)->feature) != m_supportedConstraintTypes.end())
			m_currentConstraints.push_back((*it)->feature + "_constraint(A_" + (*it)->value + ")");
	}
	
	evd->trueEvidence = m_currentConstraints;
	m_constraintAddr = _wmc.address;
	distributeEvd(evd, m_constraintAddr.id,"",1000,200);
}

void MLNRefResolutionClient::removeConstraints(const cdl::WorkingMemoryChange & _wmc)
{
	debug("Reference resolution request deleted %s %s", _wmc.address.id.c_str(), m_constraintAddr.id.c_str());
	if(_wmc.address == m_constraintAddr) {
		debug("Adding evidence for constraint removal");
		EvidencePtr evd = new Evidence();
		evd->noEvidence = m_currentConstraints;
		m_currentConstraints.clear();
		
		distributeEvd(evd, "", m_constraintAddr.id,1000,200);
	}
}

void MLNRefResolutionClient::configure(const map<string,string> & _config)
{
//  BindingWorkingMemoryWriter::configure(_config);
  AbsMLNClient::configure(_config);
    
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

void MLNRefResolutionClient::start()
{
  // must call super start to ensure that the reader sets up change
  // filters 
  AbsMLNClient::start();
  m_constraintAddr.id = "no constraint";
  
  string constraints[] = {"color","shape","type"};
  
  m_supportedConstraintTypes = set<string>(constraints, constraints + 3); 
  
  addChangeFilter(createGlobalTypeFilter<ReferenceResolutionRequest>(cdl::ADD),
		new MemberFunctionChangeReceiver<MLNRefResolutionClient>(this,
		  &MLNRefResolutionClient::newConstraints));
		  
	addChangeFilter(createGlobalTypeFilter<ReferenceResolutionRequest>(cdl::DELETE),
		new MemberFunctionChangeReceiver<MLNRefResolutionClient>(this,
		  &MLNRefResolutionClient::removeConstraints));
  
  log("MLNRefResolutionClient active");
}

void MLNRefResolutionClient::runComponent()
{
  sleep(3);
  
  // init mrf
  // leave the .db file from command line empty, because it does
  // not work online if the evidence is initialized there
  // distributeQuery("resolution");
  
  // initial evidence - we treat percepts as close-world predicates;
  // again "-cw" option in command line doesn't help, beacuse the
  // evidence from .db is not correctly initialized
/*
  EvidencePtr evd = new Evidence();

  evd->trueEvidence.push_back("constraint(A_red)");
  evd->falseEvidence.push_back("belief(PH1)");
  evd->falseEvidence.push_back("belief(PH2)");
  evd->falseEvidence.push_back("belief(PH3)");
  evd->falseEvidence.push_back("belief(PH4)");
  evd->falseEvidence.push_back("belief(PH5)");
  evd->falseEvidence.push_back("belief(PH6)");

  evd->initInfSteps = 400;
  evd->prevInfSteps = 0;
  evd->burnInSteps = 100;

  distributeEvd(evd);
*/  
  while(isRunning())	  
  {
    //nah: replaced sleepComponent(100) with waitForChanges() which fires on every WM change as this seems more like what was wanted
    waitForChanges();
		if( newResultReady()) {
			InferredResultPtr infRes = getInfResult();

			//nah: this seems to be continually true...
			//log("New inference result available from MLN engine ID %s", getInfEngineID().c_str());
		
			if(infRes->token == m_constraintAddr.id && infRes->tokenSamples >= 400) {
				vector<EpistemicReferenceHypothesisPtr> hypos = getHypothesisList(infRes);
		
				if(!hypos.empty()) {
					ReferenceResolutionResultPtr rrResult = new ReferenceResolutionResult();
					rrResult->nom = "";
					rrResult->method = "MLN";
					rrResult->requestAddress = m_constraintAddr;
					rrResult->hypos = hypos;
					addToWorkingMemory(newDataID(), getBindingSA(), rrResult);
					
					log("Reference resolution for request ID '%s' added", m_constraintAddr.id.c_str());
//					m_constraintAddr.id = "no constraint";				
				}
			}
		}
		
  }
}



vector<EpistemicReferenceHypothesisPtr> MLNRefResolutionClient::getHypothesisList(InferredResultPtr infRes)
{
	vector<string> atoms = infRes->atoms;
	vector<EpistemicReferenceHypothesisPtr> hypos;
	hypos.clear();
	
	for(int i=0; i < atoms.size(); i++) {
			string id = getBeliefId(atoms[i]);
			if(!id.empty()) {
				EpistemicReferenceHypothesisPtr h;
				if(makeHypothesis(id, infRes->probs[i], h))
					hypos.push_back(h);
			}
	}
	return hypos;
}


string MLNRefResolutionClient::getBeliefId(string atom)
{
	int p1 = atom.find_first_of('(') + 1;
	int lnt = atom.find_first_of(',') - p1;
	if(lnt < 1)
		lnt = atom.find_first_of(')') - p1;
		
	string id = atom.substr(p1, lnt);
	debug("Belief Id '%s'", id.c_str());
		
	if(id.find(':') != string::npos)
		return id;
	else
		return "";
	
}


bool MLNRefResolutionClient::makeHypothesis(string id, double prob, EpistemicReferenceHypothesisPtr &h)
{
	WorkingMemoryAddress wma;
	wma.id = id;
	wma.subarchitecture = getBindingSA();
	
	dBeliefPtr b;
	try {
	  b = getMemoryEntry<dBelief>(wma);	  
	}
	catch (DoesNotExistOnWMException e) {
	  log("WARNING: belief ID %s not in WM.", id.c_str());
	  return false;
	}
	
	PointerFormulaPtr f = new PointerFormula();
	f->id = rand();

	//Y3 hack to return first ancestor
	//history::CASTBeliefHistory* hist = dynamic_cast<history::CASTBeliefHistory*>(b->hist.get());
	//f->pointer = hist->ancestors[0]->address;
	//f->type = "dBelief";
	//end Y3 hack

	f->pointer = wma;
	f->type = b->ice_staticId();

	h = new EpistemicReferenceHypothesis();
	h->epst = b->estatus;
	h->referent = f;
	h->score = prob;
	
	return true;
}

}

