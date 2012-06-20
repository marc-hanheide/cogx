/*
 * All of the documentation and software included in the
 * Alchemy Software is copyrighted by Stanley Kok, Parag
 * Singla, Matthew Richardson, Pedro Domingos, Marc
 * Sumner, Hoifung Poon, and Daniel Lowd.
 * 
 * Copyright [2004-07] Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner, Hoifung
 * Poon, and Daniel Lowd. All rights reserved.
 * 
 * Contact: Pedro Domingos, University of Washington
 * (pedrod@cs.washington.edu).
 * 
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above
 * copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the
 * above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use
 * of this software must display the following
 * acknowledgment: "This product includes software
 * developed by Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner, Hoifung
 * Poon, and Daniel Lowd in the Department of Computer Science and
 * Engineering at the University of Washington".
 * 
 * 4. Your publications acknowledge the use or
 * contribution made by the Software to your research
 * using the following citation(s): 
 * Stanley Kok, Parag Singla, Matthew Richardson and
 * Pedro Domingos (2005). "The Alchemy System for
 * Statistical Relational AI", Technical Report,
 * Department of Computer Science and Engineering,
 * University of Washington, Seattle, WA.
 * http://www.cs.washington.edu/ai/alchemy.
 * 
 * 5. Neither the name of the University of Washington nor
 * the names of its contributors may be used to endorse or
 * promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY OF WASHINGTON
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE UNIVERSITY
 * OF WASHINGTON OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef ONLINEENGINE_H_
#define ONLINEENGINE_H_

#include "inference.h"
#include "infer.h"
#include "variablestate.h"
#include "arguments.h"
#include "util.h"
#include "learnwts.h"
#include "lbfgsb.h"

const double GEN_DEFAULT_STD_DEV = 100;
const bool PRINT_CLAUSE_DURING_COUNT = true;

  // Set to true for more output
const bool oedebug = false;


  // TODO: List the arguments common to learnwts and inference in
  // inferenceargs.h. This can't be done with a static array.
ARGS ARGS::Args[] = 
{
    // BEGIN: Common arguments
  ARGS("i", ARGS::Req, ainMLNFiles, 
       "Comma-separated input .mln files."),

  ARGS("cw", ARGS::Opt, aClosedWorldPredsStr,
       "Specified non-evidence atoms (comma-separated with no space) are "
       "closed world, otherwise, all non-evidence atoms are open world. Atoms "
       "appearing here cannot be query atoms and cannot appear in the -o "
       "option."),

  ARGS("ow", ARGS::Opt, aOpenWorldPredsStr,
       "Specified evidence atoms (comma-separated with no space) are open "
       "world, while other evidence atoms are closed-world. "
       "Atoms appearing here cannot appear in the -c option."),
    // END: Common arguments

    // BEGIN: Common inference arguments
  ARGS("m", ARGS::Tog, amapPos, 
       "Run MAP inference and return only positive query atoms."),

  ARGS("a", ARGS::Tog, amapAll, 
       "Run MAP inference and show 0/1 results for all query atoms."),

  ARGS("p", ARGS::Tog, agibbsInfer, 
       "Run inference using MCMC (Gibbs sampling) and return probabilities "
       "for all query atoms."),
  
  ARGS("ms", ARGS::Tog, amcsatInfer,
       "Run inference using MC-SAT and return probabilities "
       "for all query atoms"),

  ARGS("simtp", ARGS::Tog, asimtpInfer,
       "Run inference using simulated tempering and return probabilities "
       "for all query atoms"),

  ARGS("seed", ARGS::Opt, aSeed,
       "[random] Seed used to initialize the randomizer in the inference "
       "algorithm. If not set, seed is initialized from the current date and "
       "time."),

  ARGS("lazy", ARGS::Opt, aLazy, 
       "[false] Run lazy version of inference if this flag is set."),
  
  ARGS("lazyNoApprox", ARGS::Opt, aLazyNoApprox, 
       "[false] Lazy version of inference will not approximate by deactivating "
       "atoms to save memory. This flag is ignored if -lazy is not set."),
  
  ARGS("memLimit", ARGS::Opt, aMemLimit, 
       "[-1] Maximum limit in kbytes which should be used for inference. "
       "-1 means main memory available on system is used."),
    // END: Common inference arguments

    // BEGIN: MaxWalkSat args
  ARGS("mwsMaxSteps", ARGS::Opt, amwsMaxSteps,
       "[1000000] (MaxWalkSat) The max number of steps taken."),

  ARGS("tries", ARGS::Opt, amwsTries, 
       "[1] (MaxWalkSat) The max number of attempts taken to find a solution."),

  ARGS("targetWt", ARGS::Opt, amwsTargetWt,
       "[the best possible] (MaxWalkSat) MaxWalkSat tries to find a solution "
       "with weight <= specified weight."),

  ARGS("hard", ARGS::Opt, amwsHard, 
       "[false] (MaxWalkSat) MaxWalkSat never breaks a hard clause in order to "
       "satisfy a soft one."),
  
  ARGS("heuristic", ARGS::Opt, amwsHeuristic,
       "[1] (MaxWalkSat) Heuristic used in MaxWalkSat (0 = RANDOM, 1 = BEST, "
       "2 = TABU, 3 = SAMPLESAT)."),
  
  ARGS("tabuLength", ARGS::Opt, amwsTabuLength,
       "[5] (MaxWalkSat) Minimum number of flips between flipping the same "
       "atom when using the tabu heuristic in MaxWalkSat." ),

  ARGS("lazyLowState", ARGS::Opt, amwsLazyLowState, 
       "[false] (MaxWalkSat) If false, the naive way of saving low states "
       "(each time a low state is found, the whole state is saved) is used; "
       "otherwise, a list of variables flipped since the last low state is "
       "kept and the low state is reconstructed. This can be much faster for "
       "very large data sets."),  
    // END: MaxWalkSat args

    // BEGIN: MCMC args
  ARGS("burnMinSteps", ARGS::Opt, amcmcBurnMinSteps,
       "[100] (MCMC) Minimun number of burn in steps (-1: no minimum)."),

  ARGS("burnMaxSteps", ARGS::Opt, amcmcBurnMaxSteps,
       "[100] (MCMC) Maximum number of burn-in steps (-1: no maximum)."),

  ARGS("minSteps", ARGS::Opt, amcmcMinSteps, 
       "[-1] (MCMC) Minimum number of Gibbs sampling steps."),

  ARGS("maxSteps", ARGS::Opt, amcmcMaxSteps, 
       "[1000] (MCMC) Maximum number of Gibbs sampling steps."),

  ARGS("maxSeconds", ARGS::Opt, amcmcMaxSeconds, 
       "[-1] (MCMC) Max number of seconds to run MCMC (-1: no maximum)."),
    // END: MCMC args
  
    // BEGIN: Simulated tempering args
  ARGS("subInterval", ARGS::Opt, asimtpSubInterval,
        "[2] (Simulated Tempering) Selection interval between swap attempts"),

  ARGS("numRuns", ARGS::Opt, asimtpNumST,
        "[3] (Simulated Tempering) Number of simulated tempering runs"),

  ARGS("numSwap", ARGS::Opt, asimtpNumSwap,
        "[10] (Simulated Tempering) Number of swapping chains"),
    // END: Simulated tempering args

    // BEGIN: MC-SAT args
  //ARGS("numStepsEveryMCSat", ARGS::Opt, amcsatNumStepsEveryMCSat,
  //     "[1] (MC-SAT) Number of total steps (mcsat + gibbs) for every mcsat "
  //     "step"),
    // END: MC-SAT args

    // BEGIN: SampleSat args
  ARGS("numSolutions", ARGS::Opt, amwsNumSolutions,
       "[10] (MC-SAT) Return nth SAT solution in SampleSat"),

  ARGS("saRatio", ARGS::Opt, assSaRatio,
       "[50] (MC-SAT) Ratio of sim. annealing steps mixed with WalkSAT in "
       "MC-SAT"),

  ARGS("saTemperature", ARGS::Opt, assSaTemp,
        "[10] (MC-SAT) Temperature (/100) for sim. annealing step in "
        "SampleSat"),

  ARGS("lateSa", ARGS::Tog, assLateSa,
       "[false] Run simulated annealing from the start in SampleSat"),
    // END: SampleSat args

    // BEGIN: Gibbs sampling args
  ARGS("numChains", ARGS::Opt, amcmcNumChains, 
       "[10] (Gibbs) Number of MCMC chains for Gibbs sampling (there must be "
       "at least 2)."),

  ARGS("delta", ARGS::Opt, agibbsDelta,
       "[0.05] (Gibbs) During Gibbs sampling, probabilty that epsilon error is "
       "exceeded is less than this value."),

  ARGS("epsilonError", ARGS::Opt, agibbsEpsilonError,
       "[0.01] (Gibbs) Fractional error from true probability."),

  ARGS("fracConverged", ARGS::Opt, agibbsFracConverged, 
       "[0.95] (Gibbs) Fraction of ground atoms with probabilities that "
       "have converged."),

  ARGS("walksatType", ARGS::Opt, agibbsWalksatType, 
       "[1] (Gibbs) Use Max Walksat to initialize ground atoms' truth values "
       "in Gibbs sampling (1: use Max Walksat, 0: random initialization)."),

  ARGS("samplesPerTest", ARGS::Opt, agibbsSamplesPerTest, 
       "[100] Perform convergence test once after this many number of samples "
       "per chain."),
    // END: Gibbs sampling args

    // BEGIN: Args specific to stand-alone inference
  ARGS("e", ARGS::Req, aevidenceFiles, 
       "Comma-separated .db files containing known ground atoms (evidence), "
       "including function definitions."),

  ARGS("q", ARGS::Opt, aqueryPredsStr, 
       "Query atoms (comma-separated with no space)  "
       ",e.g., cancer,smokes(x),friends(Stan,x). Query atoms are always "
       "open world."),

  ARGS("f", ARGS::Opt, aqueryFile,
       "A .db file containing ground query atoms, "
       "which are are always open world."),
    // END: Args specific to stand-alone inference

  ARGS()
};


class OnlineEngine
{
 public:
 
  OnlineEngine(const string& inferString)
  {
    Inference* inference = NULL;
    parseInferString(inferString, inference);
    setInference(inference);
  }
  
  OnlineEngine(Inference* inference)
  {
    setInference(inference);
  }

  ~OnlineEngine()
  {
  	PowerSet::deletePowerSet();
    delete inference_;
  }
     
  void init()
  {
    inference_->init();
  }
  
  void infer(vector<string> query, vector<string>& nonZeroAtoms, vector<float>& probs )
  {
 // 	vector<float> oldProbs=probs;
    nonZeroAtoms.clear();
    probs.clear();
    
    inference_->infer();
      // Fill in vectors

//    inference_->getChangedPreds(nonZeroAtoms, probs, oldProbs, 0.0f);
	((MCMC*) inference_)->getPredProbs(query, nonZeroAtoms, probs);
    assert(nonZeroAtoms.size() == probs.size());
  }

  void addTrueEvidence(const vector<string>& evidence)
  {
    addRemoveEvidenceHelper(evidence, true, true);
  }
  
  void addFalseEvidence(const vector<string>& evidence)
  {
    addRemoveEvidenceHelper(evidence, true, false);
  }
  
  void removeEvidence(const vector<string>& oldEvidence)
  {
    addRemoveEvidenceHelper(oldEvidence, false, false);
  }

  void setInference(Inference* inference)
  {
    inference_ = inference;
  }
  
  void setMaxInferenceSteps(const int& inferenceSteps)
  {
      // Check if using MWS
    if (MaxWalkSat* mws = dynamic_cast<MaxWalkSat*>(inference_))
    {
      mws->setMaxSteps(inferenceSteps);
    }  
    else if (MCSAT* ms = dynamic_cast<MCSAT*>(inference_))
    {
      ms->setMaxSteps(inferenceSteps);
    }
    else if (GibbsSampler* p = dynamic_cast<GibbsSampler*>(inference_))
    {
      p->setMaxSteps(inferenceSteps);
    }
  }
  
  void setMaxBurnIn(const int& maxBurnIn)
  {
    if (MCSAT* ms = dynamic_cast<MCSAT*>(inference_))
    {
      ms->setMaxBurnIn(maxBurnIn);
    }
    else if (GibbsSampler* p = dynamic_cast<GibbsSampler*>(inference_))
    {
      p->setMaxBurnIn(maxBurnIn);
    }
  }
  
  void saveAllCounts(bool saveCounts=true)
  {
    inference_->saveAllCounts(saveCounts);
  }
  
  void tallyCntsFromState()
  {
    inference_->tallyCntsFromState();
  }
  
  void resetCnts()
  {
    inference_->resetCnts();
  }
  
  void saveCnts()
  {
    inference_->saveCnts();
  }
  
  void restoreCnts()
  {
    inference_->restoreCnts();
  }
  
  int getNumSamples()
  {
    return inference_->getNumSamples();
  }
  
  double getClauseTrueCnts(int i)
  {
    return (*inference_->getClauseTrueCnts())[i];
  }
  
  void adaptProbs(int maxn)
  {
  	if (MCSAT* ms = dynamic_cast<MCSAT*>(inference_))
  		ms->adaptProbs(maxn);
  }
  
  void setExtPriors(const vector<string>& preds, const vector<float>& wts)
  {
  	assert(preds.size() == wts.size());
  	
    for (int i=0; i < preds.size(); i++)
    {
      GroundPredicate* p = NULL;
      parseGroundPredicate(preds[i], p);
      inference_->getState()->setClausePrior(p, wts[i]);
    }
  }
  
  void resetPriors(const vector<string>& preds)
  {
    for (int i=0; i < preds.size(); i++)
    {
      GroundPredicate* p = NULL;
      parseGroundPredicate(preds[i], p);
      inference_->getState()->resetClausePrior(p);
    }
  }
  
  string addInstance(string type, string pred)
  {
  	const Domain* domain = inference_->getState()->getDomain();
      
    const PredicateTemplate* pt = domain->getPredicateTemplate(pred.c_str());
    if (pt) {// && pt->getNumTerms() == 1 && pt->getTermTypeAsStr(0) == type.c_str()) {
    	
    	const Array<int>* inst = domain->getConstantsByType(type.c_str());
    	
    	for(int i=0; i<inst->size(); i++) {
    		Predicate* p = new Predicate(pt);
    		p->appendTerm(new Term(inst->item(i), (void*)p, true));
    		assert(p->isGrounded());
    		
      		if(domain->getDB()->getValue(p) == FALSE ) {

      			string placeh = string(domain->getConstantName(inst->item(i)));
      			cout << "Activating constant placeholder '" << placeh << "." << endl;
      			
      			GroundPredicate* gp = NULL;
      			parseGroundPredicate(pred + "(" + placeh + ")", gp);
      			
     				inference_->getState()->setAsEvidence(gp, true);		
						delete p;
      			return placeh;
      		}
      		delete p;
      	}
      	   	
    	cout << "WARNING: Too many instances. Could not add a new one." << endl;
    	return "";
    }
    cout << "WARNING: Cannot form the predicate '" << pred <<"'." << endl;
    return "";
  }
  
  void removeInstance(string placeh, string pred)
  {
  	const Domain* domain = inference_->getState()->getDomain();
      
    const PredicateTemplate* pt = domain->getPredicateTemplate(pred.c_str());
    if (pt) {// && pt->getNumTerms() == 1 && pt->getTermTypeAsStr(0) == type.c_str()) {
/*    	
    	Predicate* p = new Predicate(pt);
    	int cid = domain->getConstantId(placeh.c_str());
    	p->appendTerm(new Term(cid, (void*)p, true));
    	assert(p->isGrounded());
*/
    		GroundPredicate* gp = NULL;
      	parseGroundPredicate(pred + "(" + placeh + ")", gp);
      	
      	if(domain->getDB()->getValue(gp) == TRUE ) {
      		cout << "Deactivating constant placeholder '" << placeh << "." << endl;
     			inference_->getState()->setAsQuery(gp);
     			parseGroundPredicate(pred + "(" + placeh + ")", gp);
     			inference_->getState()->setAsEvidence(gp, false); //new GroundPredicate(p), false);		

      		return;
      	}
      	cout << "WARNING: Placeholder costant '" << placeh << "' was not active." << endl;
    	return;
    }
    cout << "WARNING: Cannot form the predicate '" << pred <<"'." << endl;
  }
/*  
  void addInstance(string name, string type, string pred)
  {
  	const Domain* domain = inference_->getState()->getDomain();
      
    const PredicateTemplate* pt = domain->getPredicateTemplate(pred.c_str());
    if (pt) {// && pt->getNumTerms() == 1 && pt->getTermTypeAsStr(0) == type.c_str()) {
    	
    	const Array<int>* inst = domain->getConstantsByType(type.c_str());
    	ConstDualMap* cm = new ConstDualMap();
    	bool inserted = false;
    	
    	for(int i=0; i<inst->size(); i++) {
    		Predicate* p = new Predicate(pt);
    		p->appendTerm(new Term(inst->item(i), (void*)p, true));
    		assert(p->isGrounded());
    		
      		if(domain->getDB()->getValue(p) == FALSE && !inserted) {
      			cout << "Replacing '" << string(domain->getConstantName(inst->item(i)))
      				<< "' with '" << name << ";" << endl;
      			cm->insert(name.c_str(), inst->item(i));
      			inserted = true;
      		}
      		else
      			cm->insert(domain->getConstantName(inst->item(i)), inst->item(i));
      	}   	

      	if(inserted) {
      		inference_->getState()->setConstMap(cm);
      		
      		cout << "New instance successfully added." << endl;
      	} else
    		cout << "WARNING: Too many instances. Could not add a new one." << endl;
    }
    
//  	inference_->getState()->addConstant(name, type);
  }
*/  
  void printNetwork(ostream& out)
  {
  	inference_->printNetwork(out);
  }
  
  int genLearnWts(vector<string> tevd, // true evidence
  				  vector<string> fevd, // false evidence
  				  vector<string> nevd, // no evidence
  				  double priorStdDev=GEN_DEFAULT_STD_DEV,
  				  char* nonEvidPredsStr=NULL,
  				  int maxIter=10000,
  				  double convThresh=1e-5)
  {
  	bool noPrior = false;
  	bool noEqualPredWt = false;
  	
	Timer timer;
	double startSec = timer.time();
	double begSec;

	if (priorStdDev < 0)
	{
	  cout << "priorStdDev set to (generative learning's) default of "
		   << GEN_DEFAULT_STD_DEV << endl;
	  priorStdDev = GEN_DEFAULT_STD_DEV;
	}
	
	if (maxIter <= 0)  { cout << "maxIter must be > 0" << endl; return -1; }
	if (convThresh <= 0 || convThresh > 1)
	  { cout << "convThresh must be > 0 and <= 1" << endl; return -1;  }
	if (priorStdDev <= 0) { cout << "priorStdDev must be > 0" << endl; return -1;}

// ?????
	  StringHashArray nonEvidPredNames;
	  if (nonEvidPredsStr)
	  {
		if (!extractPredNames(nonEvidPredsStr, NULL, nonEvidPredNames))
		{
		  cout << "ERROR: failed to extract non-evidence predicate names." << endl;
		  return -1;
		}
	  }

	  StringHashArray owPredNames;
	  StringHashArray cwPredNames;

	  ////////////////////////// create domains and mlns //////////////////////////

	  Array<Domain*> domains;
	  Array<MLN*> mlns;

	  
	  Database* infdb;
	  
	  Domain* domain = (Domain*) inference_->getState()->getDomain();
	  MLN* mln = (MLN*) inference_->getState()->getMLN();
	  
	  infdb = domain->getDB();
	  Database* db = new Database(*infdb);
	  
	  domain->setDB(db);
	  
	  for(int i=0; i<db->getClosedWorld().size(); i++)
	  	if(db->isClosedWorld((const int) i)) cout << "pred " << i << "closed" << endl;
	  	else cout << "pred " << i << "open" << endl;

	  
	  dbEvidenceHelper(nevd, false, false);
	  dbEvidenceHelper(tevd, true, true);
	  dbEvidenceHelper(fevd, true, false);
	  
//	  db->printInfo();
/*	  
	  Array<bool> cw = db->getClosedWorld();
	  for(int i=0; i<cw.size(); i++)
	  	if(db->isClosedWorld((const int) i)) cout << "pred " << i << "closed" << endl;
	  	else cout << "pred " << i << "open" << endl;
	  
	  		
	  for(int i=0; i<db->getClosedWorld().size(); i++)
	  	db->setClosedWorld (i, true);
	  	
	  for(int i=0; i<cw.size(); i++)
	  	if(db->isClosedWorld((const int) i)) cout << "pred " << i << "closed" << endl;
	  	else cout << "pred " << i << "open" << endl;
*/	  	
//	  db->printInfo();	  
	  
	  domains.append(domain);
  	  mlns.append(mln);

	// start here
	  const FormulaAndClausesArray* fca = mlns[0]->getFormulaAndClausesArray();
	  for (int i = 0; i < fca->size(); i++)
	  {
		IndexClauseHashArray* indexClauses = (*fca)[i]->indexClauses;
		for (int j = 0; j < indexClauses->size(); j++)
		{
		  int idx = (*indexClauses)[j]->index;
		  Clause* c = (*indexClauses)[j]->clause;
		}
	  }

	  /*
	  cout << "Clause prior means:" << endl;
	  cout << "_________________________________" << endl;
	  mlns[0]->printClausePriorMeans(cout, domains[0]);
	  cout << "_________________________________" << endl;
	  cout << endl;

	  cout << "Formula prior means:" << endl;
	  cout << "_________________________________" << endl;
	  mlns[0]->printFormulaPriorMeans(cout);
	  cout << "_________________________________" << endl;
	  cout << endl;
	  */

	  //////////////////// set the prior means & standard deviations //////////////

		//we need an index translator if clauses do not line up across multiple DBs
	  IndexTranslator* indexTrans
		= (IndexTranslator::needIndexTranslator(mlns, domains)) ?
		   new IndexTranslator(&mlns, &domains) : NULL;

	  if (indexTrans)
		cout << endl << "the weights of clauses in the CNFs of existential"
			 << " formulas will be tied" << endl;

	  Array<double> priorMeans, priorStdDevs;
	  Array<bool> lockedWts;
	  if (!noPrior)
	  {
		if (indexTrans)
		{
		  indexTrans->setPriorMeans(priorMeans);
		  priorStdDevs.growToSize(priorMeans.size());
		  lockedWts.growToSize(priorMeans.size());
		  for (int i = 0; i < priorMeans.size(); i++)
		  {
			priorStdDevs[i] = priorStdDev;
			lockedWts[i] = false;
		  }
		}
		else
		{
		  const ClauseHashArray* clauses = mlns[0]->getClauses();
		  int numClauses = clauses->size();
		  for (int i = 0; i < numClauses; i++)
		  {
			priorMeans.append((*clauses)[i]->getWt());
			priorStdDevs.append(priorStdDev);
			lockedWts.append((*clauses)[i]->isStaticWt());
		  }
		}
	  }

	  int numClausesFormulas;
	  if (indexTrans)
		numClausesFormulas = indexTrans->getNumClausesAndExistFormulas();
	  else
		numClausesFormulas = mlns[0]->getClauses()->size();


	  Array<double> wts;

	  for (int j = 0; j < mlns[0]->getNumClauses(); j++)
	  {
		Clause* c = (Clause*) mlns[0]->getClause(j);
		  // If the weight was set to non-zero in the source MLN,
		  // don't modify it while learning.
		if (c->getWt() != 0 && c->isStaticWt())
		  c->lock();
	  }
	    
	  Array<bool> areNonEvidPreds;
	  if (nonEvidPredNames.empty())
	  {
	    areNonEvidPreds.growToSize(domains[0]->getNumPredicates(), true);
	    for (int i = 0; i < domains[0]->getNumPredicates(); i++)
	    {
		    //prevent equal pred from being non-evidence preds
		  if (domains[0]->getPredicateTemplate(i)->isEqualPred())
		  {
		    const char* pname = domains[0]->getPredicateTemplate(i)->getName();
		    int predId = domains[0]->getPredicateId(pname);
		    areNonEvidPreds[predId] = false;
		  }
		    //prevent internal preds from being non-evidence preds
		  if (domains[0]->getPredicateTemplate(i)->isInternalPredicateTemplate())
		  {
		    const char* pname = domains[0]->getPredicateTemplate(i)->getName();
		    int predId = domains[0]->getPredicateId(pname);
		    areNonEvidPreds[predId] = false;
		  }
	    }
	  }
	  else
	  {
	    areNonEvidPreds.growToSize(domains[0]->getNumPredicates(), false);
	    for (int i = 0; i < nonEvidPredNames.size(); i++)
	    {
		  int predId = domains[0]->getPredicateId(nonEvidPredNames[i].c_str());
		  if (predId < 0)
		  {
		    cout << "ERROR: Predicate " << nonEvidPredNames[i] << " undefined."
			     << endl;
		    exit(-1);
		  }
		  areNonEvidPreds[predId] = true;
	    }
	  }

	  Array<bool>* nePreds = &areNonEvidPreds;
	  PseudoLogLikelihood pll(nePreds, &domains, !noEqualPredWt, false,-1,-1,-1);
	  pll.setIndexTranslator(indexTrans);

	  if (!noPrior)
	    pll.setMeansStdDevs(numClausesFormulas, priorMeans.getItems(),
		  		            priorStdDevs.getItems());
	  else
	    pll.setMeansStdDevs(-1, NULL, NULL);

	  pll.setLockedWts(lockedWts.getItems());

	  ////////////// compute the counts for the clauses

	  begSec = timer.time();
	  Array<bool> wtsWithTiedClauses;
	  wtsWithTiedClauses.growToSize(numClausesFormulas + 1, false);
	  int numWts = 0;
	  for (int m = 0; m < mlns.size(); m++)
	  {
	    cout << "Computing counts for clauses in domain " << m << "..." << endl;
/*
	  const ClauseHashArray* clauses = mlns[m]->getClauses();
	  for (int i = 0; i < clauses->size(); i++)
	  {
		if (PRINT_CLAUSE_DURING_COUNT)
		{
		  cout << "clause " << i << ": ";
		  (*clauses)[i]->printWithoutWt(cout, domains[m]);
		  cout << endl; cout.flush();
		}
		MLNClauseInfo* ci = (MLNClauseInfo*) mlns[m]->getMLNClauseInfo(i);
		pll.computeCountsForNewAppendedClause((*clauses)[i], &(ci->index), m,
				                              NULL, false, NULL);
	  }
*/		

	    const ClauseHashArray* clauses = mlns[m]->getClauses();
	    const Array<MLNClauseInfo*>* clauseInfos = mlns[m]->getMLNClauseInfos();
	    const FormulaAndClausesArray* facs = mlns[m]->getFormulaAndClausesArray();
	    for (int i = 0; i < clauseInfos->size(); i++)
	    {
		  MLNClauseInfo* ci = (*clauseInfos)[i];
		  if (PRINT_CLAUSE_DURING_COUNT)
		  {
		    cout << "clause " << i << ": ";
		    (*clauses)[i]->printWithoutWt(cout, domains[m]);
		    cout << endl; cout.flush();
		  }

		  Array<FormulaClauseIndexes*>& fciArr = ci->formulaClauseIndexes;
		  int fidx = *(fciArr[0]->formulaIndex);
		  int cidx = *(fciArr[0]->clauseIndex);
		  FormulaAndClauses* fac = (*facs)[fidx];

		  // If clauses are tied together, compute counts for the conjunction
		  // of clauses
		  if (fac->tiedClauses)
		  {
		    if (cidx == 0)
		    {
			  if (m == 0)
			  {
			    numWts++;
			    wtsWithTiedClauses[i+1] = true;
			  }
			  Array<Clause*>* tiedClauses = new Array<Clause*>;
			  IndexClauseHashArray* indexClauses = fac->indexClauses;
			  for (int c = 1; c < indexClauses->size(); c++)
			    tiedClauses->append((*indexClauses)[c]->clause);
			  
			  pll.computeCountsForNewAppendedClause((*clauses)[i], &(ci->index),
				                                     m, NULL, false, NULL,
				                                     tiedClauses);
			  delete tiedClauses;
		    }
		  }
		  else
		  {
		    if (m == 0)
		    {
			  numWts++;
			  wtsWithTiedClauses[i+1] = true;
		    }
		    pll.computeCountsForNewAppendedClause((*clauses)[i], &(ci->index), m,
				                                  NULL, false, NULL, NULL);
		  }
	    }
	  }


	pll.compress();
	cout <<"Computing counts took ";
	Timer::printTime(cout, timer.time() - begSec); cout << endl;

	////////////// learn clause wts

	  // initialize the clause weights
	wts.growToSize(1);
	wts.append(0.0);
	int numNonLockedWts = numWts;
	for (int i = 0; i < numWts; i++)
	{
	  if (noPrior)
	  {
		wts.growToSize(wts.size() + 1);
		wts[i+1] = 0;
	  }
	  else
	  {
		if (lockedWts[i]) numNonLockedWts--;
		else
		{
		  wts.growToSize(wts.size() + 1);
		  wts[i+1] = 0;
		}
	  }
	}

	  // optimize the clause weights
	cout << "L-BFGS-B is finding optimal weights......" << endl;
	begSec = timer.time();
	//LBFGSB lbfgsb(maxIter, convThresh, &pll, numClausesFormulas);
	LBFGSB lbfgsb(maxIter, convThresh, &pll, numNonLockedWts);
	int iter;
	bool error;
	double pllValue = lbfgsb.minimize((double*)wts.getItems(), iter, error);

	if (error) cout << "LBFGSB returned with an error!" << endl;
	cout << "num iterations        = " << iter << endl;
	cout << "time taken            = ";
	Timer::printTime(cout, timer.time() - begSec);
	cout << endl;
	cout << "pseudo-log-likelihood = " << -pllValue << endl;

	  // If tied clauses, then fill out the wts array with zeros
	wts.growToSize(numClausesFormulas + 1);
	Array<double> wtsAfterTiedClauses;
	wtsAfterTiedClauses.growToSize(numClausesFormulas + 1);
	int idxOffset = 0;
	for (int w = 1; w < wtsAfterTiedClauses.size(); w++)
	{
	  if (wtsWithTiedClauses[w])
	  {
		wtsAfterTiedClauses[w] = wts[w - idxOffset];
		idxOffset = 0;
	  }
	  else
	  {
		wtsAfterTiedClauses[w] = 0.0;
		idxOffset++;
	  }
	}
	for (int w = 1; w < wtsAfterTiedClauses.size(); w++)
	{
	  wts[w] = wtsAfterTiedClauses[w];
	}

	//////////////////////////// output results ////////////////////////////////
	if (indexTrans) assignWtsAndOutputMLN(cout, mlns, domains, wts, indexTrans);
	else            assignWtsAndOutputMLN(cout, mlns, domains, wts);

	// Update MRF
	inference_->getState()->setGndClausesWtsToSumOfParentWts();
	
	/////////////////////////////// clean up ///////////////////////////////////
	
	domain->setDB(infdb);
	delete(db);
	
	/*
	deleteDomains(domains);

	for (int i = 0; i < mlns.size(); i++)
	{
	  if (DOMAINS_SHARE_DATA_STRUCT && i > 0)
	  {
		mlns[i]->setClauses(NULL);
		mlns[i]->setMLNClauseInfos(NULL);
		mlns[i]->setPredIdToClausesMap(NULL);
		mlns[i]->setFormulaAndClausesArray(NULL);
		mlns[i]->setExternalClause(NULL);
	  }
	  
	  delete mlns[i];
	}
	*/

//	PowerSet::deletePowerSet();
	if (indexTrans) delete indexTrans;

	cout << "Total time = ";
	Timer::printTime(cout, timer.time() - startSec); cout << endl;
  }
  
 private:

  void addRemoveEvidenceHelper(const vector<string>& evidence,
                               const bool& addEvidence,
                               const bool& trueEvidence)
  {
    vector<string>::const_iterator it = evidence.begin();
    for (; it != evidence.end(); it++)
    {
      GroundPredicate* p = NULL;
      parseGroundPredicate((*it), p);
      if (addEvidence) {
        inference_->getState()->setAsEvidence(p, trueEvidence); }
      else {
        inference_->getState()->setAsQuery(p);}
    }
//HACK
//   inference_->getState()->init();
  }
 
  void dbEvidenceHelper(const vector<string>& evidence,
                               const bool& addEvidence,
                               const bool& trueEvidence)
  {
    vector<string>::const_iterator it = evidence.begin();
    for (; it != evidence.end(); it++)
    {
      GroundPredicate* p = NULL;
      parseGroundPredicate((*it), p);
      
      inference_->getState()->getDomain()
        	->getDB()->setEvidenceStatus(p, addEvidence);
      
      if (trueEvidence)
        inference_->getState()->getDomain()
        	->getDB()->setValue(p, TRUE);
      else
		inference_->getState()->getDomain()
        	->getDB()->setValue(p, FALSE);
    }
  }
 
  void parseGroundPredicate(const string& predicateAsString,
                            GroundPredicate*& predicate)
  {
    const Domain* domain = inference_->getState()->getDomain();
      
      // Parse left to right predname ( constant1 , constant2 , ... )
    string rest = string(predicateAsString);
    string::size_type leftPar = rest.find("(", 0);
    if (leftPar == string::npos)
    {
      cout << predicateAsString
           << " was given as a predicate but it is not well-formed" << endl;
      exit(-1);
    }
    string name = Util::trim(rest.substr(0, leftPar));
      // rest is constant1 , constant2 , ... )
    rest = rest.substr(leftPar+1);
    const PredicateTemplate* pt = domain->getPredicateTemplate(name.c_str());
    
    if (pt)
    {
      Predicate* p = new Predicate(pt);
      string::size_type comma = rest.find(",", 0);
      while (comma != string::npos)
      {
        string constant = Util::trim(rest.substr(0, comma));
        appendConstantToPredicate(p, constant);
        rest = Util::trim(rest.substr(comma+1));
        comma = rest.find(",", 0);
      }
      string::size_type rightPar = rest.find(")", 0);
      if (rightPar == string::npos)
      {
        cout << predicateAsString
             << " was given as a predicate but it is not well-formed" << endl;
        exit(-1);
      }
      string constant = Util::trim(rest.substr(0, rightPar));
      appendConstantToPredicate(p, constant);
      assert(p->isGrounded());
      predicate = new GroundPredicate(p);
    }
    else
    {
      cout << "Predicate " << name << " is not known. Exiting..." << endl;
      exit(-1);
    }
  }

  void appendConstantToPredicate(Predicate*& pred, const string& constant)
  {
    const char* name = constant.c_str();
    if (isupper(name[0]) || name[0] == '"' || isdigit(name[0]) )  // if is a constant
    {
      const Domain* domain = inference_->getState()->getDomain();
      int constId = domain->getConstantId(name);
      if (constId < 0)
      {
        cout << "appendConstantToPredicate(): failed to find constant " << name;
        exit(-1);
      }
      
        // if exceeded the number of terms
      int exp, unexp;
      if ((unexp = pred->getNumTerms()) ==
          (exp = pred->getTemplate()->getNumTerms()))
      {
        cout << "Wrong number of terms for predicate " << pred->getName()
             << ". Expected " << exp << " but given " << unexp; 
        exit(-1);
      }

        // Check that constant has same type as that of predicate term
      int typeId = pred->getTermTypeAsInt(pred->getNumTerms());
      int unexpId;
// HACK
      if (typeId != (unexpId = domain->getConstantTypeIds(constId)->item(0)))
      {
        const char* expName = domain->getTypeName(typeId);
        const char* unexpName = domain->getTypeName(unexpId);
        cout << "Constant " << name
             << " is of the wrong type. Expected " << expName
             << " but given " << unexpName;
        exit(-1);
      }
  
        // At this point, we have the right num of terms and right types
      if (pred != NULL) pred->appendTerm(new Term(constId, (void*)pred, true));
    }
    else  // is a variable
    {
      cout << constant
           << " does not appear to be a constant." << endl;
      exit(-1);  
    }
  }
  
  
  void parseInferString(const string& inferString, Inference*& inference)
  {
    int inferArgc = 0;
    char **inferArgv = new char*[200];
    for (int i = 0; i < 200; i++)
    {
      inferArgv[i] = new char[500];
    }

    extractArgs(inferString.c_str(), inferArgc, inferArgv);
    cout << "extractArgs " << inferArgc << endl;
    for (int i = 0; i < inferArgc; i++)
    {
      cout << i << ": " << inferArgv[i] << endl;
    }

    ARGS::parse(inferArgc, inferArgv, &cout);

      // HACK: Argument parser doesn't parse the ARGS::Tog right, so do
      // it here by hand
    for (int i = 0; i < inferArgc; i++)
    {
      if (string(inferArgv[i]) == "-m") amapPos = true;
      else if (string(inferArgv[i]) == "-a") amapAll = true;
      else if (string(inferArgv[i]) == "-p") agibbsInfer = true;
      else if (string(inferArgv[i]) == "-ms") amcsatInfer = true;
      else if (string(inferArgv[i]) == "-simtp") asimtpInfer = true;
      else if (string(inferArgv[i]) == "-bp") abpInfer = true;
      else if (string(inferArgv[i]) == "-efbp") aefbpInfer = true;
    }
      // Delete memory allocated for args
    for (int i = 0; i < 200; i++)
    {
      delete[] inferArgv[i];
    }
    delete[] inferArgv;
    
    Domain* domain = NULL;
    Array<Predicate *> querryPreds = NULL;
    Array<TruthValue> queryPredValues = NULL;
    if (!buildInference(inference, domain, false, querryPreds, queryPredValues))
      exit(-1);
 }

 private:
 
  Inference* inference_;

};

#endif /*ONLINEENGINE_H_*/

