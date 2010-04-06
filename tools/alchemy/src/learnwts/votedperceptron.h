/*
 * All of the documentation and software included in the
 * Alchemy Software is copyrighted by Stanley Kok, Parag
 * Singla, Matthew Richardson, Pedro Domingos, Marc
 * Sumner and Hoifung Poon.
 * 
 * Copyright [2004-07] Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner and Hoifung
 * Poon. All rights reserved.
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
 * Richardson, Pedro Domingos, Marc Sumner and Hoifung
 * Poon in the Department of Computer Science and
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
#ifndef VOTED_PERCEPTRON_H_OCT_30_2005
#define VOTED_PERCEPTRON_H_OCT_30_2005

#include "infer.h"
#include "clause.h"
#include "timer.h"
#include "indextranslator.h"
#include "maxwalksat.h"

const bool vpdebug = false;
const double EPSILON=.00001;

/**
 * VotedPerceptron algorithm (see "Discriminative Training of Markov Logic
 * Networks", Singla and Domingos, 2005).
 */
class VotedPerceptron 
{
 public:

  /**
   * Constructor. Various variables are initialized, relevant clauses are
   * determined and weights and inference procedures are initialized.
   * 
   * @param inferences Array of inference procedures to be used for inference
   * in each domain.
   * @param nonEvidPredNames Names of non-evidence predicates. This is used to
   * determine the relevant clauses.
   * @param idxTrans IndexTranslator needed when multiple dbs are used and they
   * don't line up.
   * @param lazyInference If true, lazy inference is used.
   * @param rescaleGradient If true, gradient is rescaled with each iteration.
   * @param withEM If true, EM is used to fill in missing values.
   */
  VotedPerceptron(const Array<Inference*>& inferences,
                  const StringHashArray& nonEvidPredNames,
                  IndexTranslator* const & idxTrans, const bool& lazyInference,
                  const bool& rescaleGradient, const bool& withEM)
    : domainCnt_(inferences.size()), idxTrans_(idxTrans),
      lazyInference_(lazyInference), rescaleGradient_(rescaleGradient),
      withEM_(withEM)
  { 
    cout << endl << "Constructing voted perceptron..." << endl << endl;

    inferences_.append(inferences);
    logOddsPerDomain_.growToSize(domainCnt_);
    clauseCntPerDomain_.growToSize(domainCnt_);
    
    for (int i = 0; i < domainCnt_; i++)
    {
      clauseCntPerDomain_[i] =
        inferences_[i]->getState()->getMLN()->getNumClauses();
      logOddsPerDomain_[i].growToSize(clauseCntPerDomain_[i], 0);
    }

    totalTrueCnts_.growToSize(domainCnt_);
    defaultTrueCnts_.growToSize(domainCnt_);
    relevantClausesPerDomain_.growToSize(domainCnt_);
    //relevantClausesFormulas_ is set in findRelevantClausesFormulas()

    findRelevantClauses(nonEvidPredNames);
    findRelevantClausesFormulas();

      // Initialize the clause wts for lazy version
    if (lazyInference_)
    {
      findCountsInitializeWtsAndSetNonEvidPredsToUnknownInDB(nonEvidPredNames);
    
      for (int i = 0; i < domainCnt_; i++)
      {
        const MLN* mln = inferences_[i]->getState()->getMLN();
        Array<double>& logOdds = logOddsPerDomain_[i];
        assert(mln->getNumClauses() == logOdds.size());
        for (int j = 0; j < mln->getNumClauses(); j++)
          ((Clause*) mln->getClause(j))->setWt(logOdds[j]);
      }
    }
      // Initialize the clause wts for eager version
    else
    {      
      initializeWts();
    }
    
      // Initialize the inference / state
    for (int i = 0; i < inferences_.size(); i++)
      inferences_[i]->init();
  }


  ~VotedPerceptron() 
  {
    for (int i = 0; i < trainTrueCnts_.size(); i++)
      delete[] trainTrueCnts_[i];
  }


    // set the prior means and std devs.
  void setMeansStdDevs(const int& arrSize, const double* const & priorMeans, 
                       const double* const & priorStdDevs) 
  {
    if (arrSize < 0) 
    {
      usePrior_ = false;
      priorMeans_ = NULL;
      priorStdDevs_ = NULL;
    } 
    else 
    {
      //cout << "arr size = " << arrSize<<", clause count = "<<clauseCnt_<<endl;
      usePrior_ = true;
      priorMeans_ = priorMeans;
      priorStdDevs_ = priorStdDevs;

      //cout << "\t\t Mean \t\t Std Deviation" << endl;
      //for (int i = 0; i < arrSize; i++) 
      //  cout << i << "\t\t" << priorMeans_[i]<<"\t\t"<<priorStdDevs_[i]<<endl;
    }
  }


    // learn the weights
  void learnWeights(double* const & weights, const int& numWeights,
                    const int& maxIter, const double& learningRate,
                    const double& momentum, bool initWithLogOdds,
                    const int& mwsMaxSubsequentSteps) 
  {
    //cout << "Learning weights discriminatively... " << endl;
    memset(weights, 0, numWeights*sizeof(double));

    double* averageWeights = new double[numWeights];
    double* gradient = new double[numWeights];
    double* lastchange = new double[numWeights];

      // Set the initial weight to the average log odds across domains/databases
    if (initWithLogOdds)
    {
        // If there is one db or the clauses for multiple databases line up
      if (idxTrans_ == NULL)
      {
        for (int i = 0; i < domainCnt_; i++)
        {
          Array<double>& logOdds = logOddsPerDomain_[i];
          assert(numWeights == logOdds.size());
          for (int j = 0; j < logOdds.size(); j++) weights[j] += logOdds[j];
        }
      }
      else
      { //the clauses for multiple databases do not line up
        const Array<Array<Array<IdxDiv>*> >* cIdxToCFIdxsPerDomain 
          = idxTrans_->getClauseIdxToClauseFormulaIdxsPerDomain();

        Array<int> numLogOdds; 
        Array<double> wtsForDomain;
        numLogOdds.growToSize(numWeights);
        wtsForDomain.growToSize(numWeights);
      
        for (int i = 0; i < domainCnt_; i++)
        {
          memset((int*)numLogOdds.getItems(), 0, numLogOdds.size()*sizeof(int));
          memset((double*)wtsForDomain.getItems(), 0,
                 wtsForDomain.size()*sizeof(double));

          Array<double>& logOdds = logOddsPerDomain_[i];
        
            // Map the each log odds of a clause to the weight of a
            // clause/formula
          for (int j = 0; j < logOdds.size(); j++)
          {
            Array<IdxDiv>* idxDivs =(*cIdxToCFIdxsPerDomain)[i][j];          
            for (int k = 0; k < idxDivs->size(); k++)
            {
              wtsForDomain[ (*idxDivs)[k].idx ] += logOdds[j];
              numLogOdds[ (*idxDivs)[k].idx ]++;
            }
          }

          for (int j = 0; j < numWeights; j++)
            if (numLogOdds[j] > 0) weights[j] += wtsForDomain[j]/numLogOdds[j];  
        }
      }
    }

      // Initialize weights, averageWeights, lastchange
    for (int i = 0; i < numWeights; i++) 
    {      
      weights[i] /= domainCnt_;
      averageWeights[i] = weights[i];
      lastchange[i] = 0.0;
    }

    for (int iter = 1; iter <= maxIter; iter++) 
    {
      cout << endl << "Iteration " << iter << " : " << endl << endl;

        // In 3rd iteration, we want to tell MWS to perform subsequentSteps
        // (Iter. 1 is random assigment if initial weights are 0)
      if (iter == 3)
      {
        for (int i = 0; i < inferences_.size(); i++)
        {
            // Check if using MWS
          if (MaxWalkSat* mws = dynamic_cast<MaxWalkSat*>(inferences_[i]))
          {
            mws->setMaxSteps(mwsMaxSubsequentSteps);
          }
        }
      }

      cout << "Getting the gradient.. " << endl;
      getGradient(weights, gradient, numWeights);
      cout << endl; 

        // Add gradient to weights
      for (int w = 0; w < numWeights; w++) 
      {
        double wchange = gradient[w] * learningRate + lastchange[w] * momentum;
        cout << "clause/formula " << w << ": wtChange = " << wchange;
        cout << "  oldWt = " << weights[w];
        weights[w] += wchange;
        lastchange[w] = wchange;
        cout << "  newWt = " << weights[w];
        averageWeights[w] = (iter * averageWeights[w] + weights[w])/(iter + 1);
        cout << "  averageWt = " << averageWeights[w] << endl;
      }
      // done with an iteration
    }
    
    cout << endl << "Learned Weights : " << endl;
    for (int w = 0; w < numWeights; w++) 
    {
      weights[w] = averageWeights[w];
      cout << w << ":" << weights[w] << endl;
    }

    delete [] averageWeights;
    delete [] gradient;
    delete [] lastchange;
    
    resetDBs();
  }
 
 
 private:
 
  /**
   * Resets the values of non-evidence predicates as they were before learning.
   */
  void resetDBs() 
  {
    if (!lazyInference_)
    {
      for (int i = 0; i < domainCnt_; i++) 
      {
        VariableState* state = inferences_[i]->getState();
        Database* db = state->getDomain()->getDB();
          // Change known NE to original values
        const GroundPredicateHashArray* knePreds = state->getKnePreds();
        const Array<TruthValue>* knePredValues = state->getKnePredValues();      
        db->setValuesToGivenValues(knePreds, knePredValues);
          // Set unknown NE back to UKNOWN
        const GroundPredicateHashArray* unePreds = state->getUnePreds();
        for (int predno = 0; predno < unePreds->size(); predno++) 
          db->setValue((*unePreds)[predno], UNKNOWN);
      }
    }
  }

  /**
   * Assign true to the elements in the relevantClauses_ bool array 
   * corresponding to indices of clauses which would be relevant for list of 
   * non-evidence predicates.
   */
  void findRelevantClauses(const StringHashArray& nonEvidPredNames) 
  {
    for (int d = 0; d < domainCnt_; d++)
    {
      int clauseCnt = clauseCntPerDomain_[d];
      Array<bool>& relevantClauses = relevantClausesPerDomain_[d];
      relevantClauses.growToSize(clauseCnt);
      memset((bool*)relevantClauses.getItems(), false, 
             relevantClauses.size()*sizeof(bool));
      const Domain* domain = inferences_[d]->getState()->getDomain();
      const MLN* mln = inferences_[d]->getState()->getMLN();
    
      const Array<IndexClause*>* indclauses;
      const Clause* clause;
      int predid, clauseid;
      for (int i = 0; i < nonEvidPredNames.size(); i++)
      {
        predid = domain->getPredicateId(nonEvidPredNames[i].c_str());
        //cout << "finding the relevant clauses for predid = " << predid 
        //     << " in domain " << d << endl;
        indclauses = mln->getClausesContainingPred(predid);
        if (indclauses) 
        {
          for (int j = 0; j < indclauses->size(); j++) 
          {
            clause = (*indclauses)[j]->clause;			
            clauseid = mln->findClauseIdx(clause);
            relevantClauses[clauseid] = true;
            //cout << clauseid << " ";
          }
          //cout<<endl;
        }
      }    
    }
  }

  
  void findRelevantClausesFormulas()
  {
    if (idxTrans_ == NULL)
    {
      Array<bool>& relevantClauses = relevantClausesPerDomain_[0];
      relevantClausesFormulas_.growToSize(relevantClauses.size());
      for (int i = 0; i < relevantClauses.size(); i++)
        relevantClausesFormulas_[i] = relevantClauses[i];
    }
    else
    {
      idxTrans_->setRelevantClausesFormulas(relevantClausesFormulas_,
                                            relevantClausesPerDomain_[0]);
      cout << "Relevant clauses/formulas:" << endl;
      idxTrans_->printRelevantClausesFormulas(cout, relevantClausesFormulas_);
      cout << endl;
    }
  }


  /**
   * Calculate true/false/unknown counts for all clauses for the given domain.
   * 
   * @param trueCnt Number of true groundings for each clause is stored here.
   * @param falseCnt Number of false groundings for each clause is stored here.
   * @param domainIdx Index of domain where the groundings are counted.
   * @param hasUnknownPreds If true, the domain has predicates with unknown
   * truth values. Otherwise it contains only predicates with known values.
   */
  void calculateCounts(Array<double>& trueCnt, Array<double>& falseCnt,
                       const int& domainIdx, const bool& hasUnknownPreds) 
  {
    Clause* clause;
    double tmpUnknownCnt;
    int clauseCnt = clauseCntPerDomain_[domainIdx];
    Array<bool>& relevantClauses = relevantClausesPerDomain_[domainIdx];
    const MLN* mln = inferences_[domainIdx]->getState()->getMLN();
    const Domain* domain = inferences_[domainIdx]->getState()->getDomain();

    for (int clauseno = 0; clauseno < clauseCnt; clauseno++) 
    {
      if (!relevantClauses[clauseno]) 
      {
        continue;
        //cout << "\n\nthis is an irrelevant clause.." << endl;
      }
      clause = (Clause*) mln->getClause(clauseno);
      clause->getNumTrueFalseUnknownGroundings(domain, domain->getDB(), 
                                               hasUnknownPreds,
                                               trueCnt[clauseno],
                                               falseCnt[clauseno],
                                               tmpUnknownCnt);
      assert(hasUnknownPreds || (tmpUnknownCnt==0));
    }
  }


  void initializeWts()
  {
    cout << "Initializing weights ..." << endl;
    Array<double *> trainFalseCnts;
    trainTrueCnts_.growToSize(domainCnt_);
    trainFalseCnts.growToSize(domainCnt_);
  
    for (int i = 0; i < domainCnt_; i++)
    {
      int clauseCnt = clauseCntPerDomain_[i];
      VariableState* state = inferences_[i]->getState();
      const GroundPredicateHashArray* unePreds = state->getUnePreds();
      const GroundPredicateHashArray* knePreds = state->getKnePreds();

      trainTrueCnts_[i] = new double[clauseCnt];
      trainFalseCnts[i] = new double[clauseCnt];

      int totalPreds = unePreds->size() + knePreds->size();
        // Used to store gnd preds to be ignored in the count because they are
        // UNKNOWN
      Array<bool>* unknownPred = new Array<bool>;
      unknownPred->growToSize(totalPreds, false);
      for (int predno = 0; predno < totalPreds; predno++) 
      {
        GroundPredicate* p;
        if (predno < unePreds->size())
          p = (*unePreds)[predno];
        else
          p = (*knePreds)[predno - unePreds->size()];
        TruthValue tv = state->getDomain()->getDB()->getValue(p);

        //assert(tv != UNKNOWN);
        if (tv == TRUE)
        {
          state->setValueOfAtom(predno + 1, true);
          p->setTruthValue(true);
        }
        else
        {
          state->setValueOfAtom(predno + 1, false);
          p->setTruthValue(false);
            // Can have unknown truth values when using EM. We want to ignore
            // these when performing the counts
          if (tv == UNKNOWN)
          {
            (*unknownPred)[predno] = true;
          }
        }
      }

      state->initMakeBreakCostWatch();
      //cout<<"getting true cnts => "<<endl;
      state->getNumClauseGndingsWithUnknown(trainTrueCnts_[i], clauseCnt, true,
                                            unknownPred);
      //cout<<endl;
      //cout<<"getting false cnts => "<<endl;
      state->getNumClauseGndingsWithUnknown(trainFalseCnts[i], clauseCnt, false,
                                            unknownPred);
      delete unknownPred;
      if (vpdebug)
      {
        for (int clauseno = 0; clauseno < clauseCnt; clauseno++)
        {
          cout << clauseno << " : tc = " << trainTrueCnts_[i][clauseno]
               << " ** fc = " << trainFalseCnts[i][clauseno] << endl;
        }
      }
    }

    double tc,fc;
    cout << "List of CNF Clauses : " << endl;
    for (int clauseno = 0; clauseno < clauseCntPerDomain_[0]; clauseno++)
    {
      if (!relevantClausesPerDomain_[0][clauseno])
      {
        for (int i = 0; i < domainCnt_; i++)
        {
          Array<double>& logOdds = logOddsPerDomain_[i];
          logOdds[clauseno] = 0.0;
        }
        continue;
      }
      //cout << endl << endl;
      cout << clauseno << ":";
      const Clause* clause =
        inferences_[0]->getState()->getMLN()->getClause(clauseno);
      //cout << (*fncArr)[clauseno]->formula <<endl;
      clause->print(cout, inferences_[0]->getState()->getDomain());
      cout << endl;
      
      tc = 0.0; fc = 0.0;
      for (int i = 0; i < domainCnt_;i++)
      {
        tc += trainTrueCnts_[i][clauseno];
        fc += trainFalseCnts[i][clauseno];
      }
	
      //cout << "true count  = " << tc << endl;
      //cout << "false count = " << fc << endl;
	
      double weight = 0.0;
      double totalCnt = tc + fc;
		
      if (totalCnt == 0) 
      {
        //cout << "NOTE: Total count is 0 for clause " << clauseno << endl;
        weight = EPSILON;
      } 
      else 
      {
        double prob =  tc / (tc+fc);
        if (prob == 0) prob = 0.00001;
        if (prob == 1) prob = 0.99999;
        weight = log(prob/(1-prob));
          //if weight exactly equals 0, make it small non zero, so that clause  
          //is not ignored during the construction of the MRF
        //if(weight == 0) weight = 0.0001;
          //commented above - make sure all weights are positive in the
          //beginning
        //if(weight < EPSILON) weight = EPSILON;
        if (abs(weight) < EPSILON) weight = EPSILON;
          //cout << "Prob " << prob << " becomes weight of " << weight << endl;
      }
      for (int i = 0; i < domainCnt_; i++) 
      {
      	Array<double>& logOdds = logOddsPerDomain_[i];
        logOdds[clauseno] = weight;
      }
    }
    cout << endl;
    
    for (int i = 0; i < trainFalseCnts.size(); i++)
      delete[] trainFalseCnts[i];
  }

  /**
   * Finds the training counts and intialize the weights for the lazy version.
   * True and false groundings have to be counted for each first-order clause
   * (this is stored in each grounding while building the mrf in the eager
   * version).
   * 
   * @param nonEvidPredNames List of non-evidence predicates.
   */
  void findCountsInitializeWtsAndSetNonEvidPredsToUnknownInDB(
                                       const StringHashArray& nonEvidPredNames)
  {
    bool hasUnknownPreds;
    Array<Array<double> > totalFalseCnts; 
    Array<Array<double> > defaultFalseCnts;
    totalFalseCnts.growToSize(domainCnt_);
    defaultFalseCnts.growToSize(domainCnt_);
    
    Array<Predicate*> gpreds;
    Array<Predicate*> ppreds;
    Array<TruthValue> gpredValues;
    Array<TruthValue> tmpValues;

    for (int i = 0; i < domainCnt_; i++) 
    {
      const Domain* domain = inferences_[i]->getState()->getDomain();
      int clauseCnt = clauseCntPerDomain_[i];
      domain->getDB()->setPerformingInference(false);

      //cout << endl << "Getting the counts for the domain " << i << endl;
      gpreds.clear();
      gpredValues.clear();
      tmpValues.clear();
      for (int predno = 0; predno < nonEvidPredNames.size(); predno++) 
      {
        ppreds.clear();
        int predid = domain->getPredicateId(nonEvidPredNames[predno].c_str());
        Predicate::createAllGroundings(predid, domain, ppreds);
        //cout<<"size of gnd for pred " << predid << " = "<<ppreds.size()<<endl;
        gpreds.append(ppreds);
      }
      
      domain->getDB()->alterTruthValue(&gpreds, UNKNOWN, FALSE, &gpredValues);
	  
      //cout <<"size of unknown set for domain "<<i<<" = "<<gpreds.size()<<endl;
      //cout << "size of the values " << i << " = " << gpredValues.size()<<endl;
	
      hasUnknownPreds = false;
      
      Array<double>& trueCnt = totalTrueCnts_[i];
      Array<double>& falseCnt = totalFalseCnts[i];
      trueCnt.growToSize(clauseCnt);
      falseCnt.growToSize(clauseCnt);
      calculateCounts(trueCnt, falseCnt, i, hasUnknownPreds);

      //cout << "got the total counts..\n\n\n" << endl;
      
      hasUnknownPreds = true;

      domain->getDB()->setValuesToUnknown(&gpreds, &tmpValues);

      Array<double>& dTrueCnt = defaultTrueCnts_[i];
      Array<double>& dFalseCnt = defaultFalseCnts[i];
      dTrueCnt.growToSize(clauseCnt);
      dFalseCnt.growToSize(clauseCnt);
      calculateCounts(dTrueCnt, dFalseCnt, i, hasUnknownPreds);

      //commented out: no need to revert the grounded non-evidence predicates
      //               to their initial values because we want to set ALL of
      //               them to UNKNOWN
      //assert(gpreds.size() == gpredValues.size());
      //domain->getDB()->setValuesToGivenValues(&gpreds, &gpredValues);
	  
      //cout << "the ground predicates are :" << endl;
      for (int predno = 0; predno < gpreds.size(); predno++) 
        delete gpreds[predno];

      domain->getDB()->setPerformingInference(true);
    }
    //cout << endl << endl;
    //cout << "got the default counts..." << endl;     
    for (int clauseno = 0; clauseno < clauseCntPerDomain_[0]; clauseno++) 
    {
      double tc = 0;
      double fc = 0;
      for (int i = 0; i < domainCnt_; i++) 
      {
      	Array<bool>& relevantClauses = relevantClausesPerDomain_[i];
      	Array<double>& logOdds = logOddsPerDomain_[i];
      
        if (!relevantClauses[clauseno]) { logOdds[clauseno] = 0; continue; }
        tc += totalTrueCnts_[i][clauseno] - defaultTrueCnts_[i][clauseno];
        fc += totalFalseCnts[i][clauseno] - defaultFalseCnts[i][clauseno];

        if (vpdebug)
          cout << clauseno << " : tc = " << tc << " ** fc = "<< fc <<endl;      
      }
      
      double weight = 0.0;

      if ((tc + fc) == 0) 
      {
        //cout << "NOTE: Total count is 0 for clause " << clauseno << endl;
      } 
      else 
      {
        double prob = tc / (tc+fc);
        if (prob == 0) prob = 0.00001;
        if (prob == 1) prob = 0.99999;
        weight = log(prob / (1-prob));
            //if weight exactly equals 0, make it small non zero, so that clause
            //is not ignored during the construction of the MRF
        //if (weight == 0) weight = 0.0001;
        if (abs(weight) < EPSILON) weight = EPSILON;
          //cout << "Prob " << prob << " becomes weight of " << weight << endl;
      }
      
      	// Set logOdds in all domains to the weight calculated
      for(int i = 0; i < domainCnt_; i++) 
      { 
      	Array<double>& logOdds = logOddsPerDomain_[i];
        logOdds[clauseno] = weight;
      }
    }
  }
 
  
  /**
   * Runs inference using the current set of parameters.
   */
  void infer() 
  {
    for (int i = 0; i < domainCnt_; i++) 
    {
      VariableState* state = inferences_[i]->getState();
      state->setGndClausesWtsToSumOfParentWts();
      //inferences_[i]->init();
        // MWS: Search is started from state at end of last iteration
      state->init();
      inferences_[i]->infer();
      state->saveLowStateToGndPreds();
    }
  }

  /**
   * Infers values for predicates with unknown truth values and uses these
   * values to compute the training counts.
   */
  void fillInMissingValues()
  {
    assert(withEM_);
    cout << "Filling in missing data ..." << endl;
      // Get values of initial unknown preds by producing MAP state of
      // unknown preds given known evidence and non-evidence preds (VPEM)
    Array<Array<TruthValue> > ueValues;
    ueValues.growToSize(domainCnt_);
    for (int i = 0; i < domainCnt_; i++)
    {
      VariableState* state = inferences_[i]->getState();
      const Domain* domain = state->getDomain();
      const GroundPredicateHashArray* knePreds = state->getKnePreds();
      const Array<TruthValue>* knePredValues = state->getKnePredValues();

        // Mark known non-evidence preds as evidence
      domain->getDB()->setValuesToGivenValues(knePreds, knePredValues);

        // Infer missing values
      state->setGndClausesWtsToSumOfParentWts();
        // MWS: Search is started from state at end of last iteration
      state->init();
      inferences_[i]->infer();
      state->saveLowStateToGndPreds();

      if (vpdebug)
      {
        cout << "Inferred following values: " << endl;
        inferences_[i]->printProbabilities(cout);
      }

        // Compute counts
      if (lazyInference_)
      {
        Array<double>& trueCnt = totalTrueCnts_[i];
        Array<double> falseCnt;
        bool hasUnknownPreds = false;
        falseCnt.growToSize(trueCnt.size());
        calculateCounts(trueCnt, falseCnt, i, hasUnknownPreds);
      }
      else
      {
        int clauseCnt = clauseCntPerDomain_[i];
        state->initMakeBreakCostWatch();
        //cout<<"getting true cnts => "<<endl;
        const Array<double>* clauseTrueCnts =
          inferences_[i]->getClauseTrueCnts();
        assert(clauseTrueCnts->size() == clauseCnt);
        for (int j = 0; j < clauseCnt; j++)
          trainTrueCnts_[i][j] = (*clauseTrueCnts)[j];
      }

        // Set evidence values back
      //assert(uePreds.size() == ueValues[i].size());
      //domain->getDB()->setValuesToGivenValues(&uePreds, &ueValues[i]);
        // Set non-evidence values to unknown
      Array<TruthValue> tmpValues;
      tmpValues.growToSize(knePreds->size());
      domain->getDB()->setValuesToUnknown(knePreds, &tmpValues);
    }
    cout << "Done filling in missing data" << endl;    
  }

  void getGradientForDomain(double* const & gradient, const int& domainIdx)
  {
    Array<bool>& relevantClauses = relevantClausesPerDomain_[domainIdx];
    int clauseCnt = clauseCntPerDomain_[domainIdx];
    double* trainCnts = NULL;
    double* inferredCnts = NULL;
    double* clauseTrainCnts = new double[clauseCnt]; 
    double* clauseInferredCnts = new double[clauseCnt];
    double trainCnt, inferredCnt;
    Array<double>& totalTrueCnts = totalTrueCnts_[domainIdx];
    Array<double>& defaultTrueCnts = defaultTrueCnts_[domainIdx];    
    const MLN* mln = inferences_[domainIdx]->getState()->getMLN();
    const Domain* domain = inferences_[domainIdx]->getState()->getDomain();

    memset(clauseTrainCnts, 0, clauseCnt*sizeof(double));
    memset(clauseInferredCnts, 0, clauseCnt*sizeof(double));

    if (!lazyInference_)
    {
      if (!inferredCnts) inferredCnts = new double[clauseCnt];

      const Array<double>* clauseTrueCnts =
        inferences_[domainIdx]->getClauseTrueCnts();
      assert(clauseTrueCnts->size() == clauseCnt);
      for (int i = 0; i < clauseCnt; i++)
        inferredCnts[i] = (*clauseTrueCnts)[i];
      trainCnts = trainTrueCnts_[domainIdx];
    }
      //loop over all the training examples
    //cout << "\t\ttrain count\t\t\t\tinferred count" << endl << endl;
    for (int clauseno = 0; clauseno < clauseCnt; clauseno++) 
    {
      if (!relevantClauses[clauseno]) continue;
      
      if (lazyInference_)
      {
      	Clause* clause = (Clause*) mln->getClause(clauseno);

      	trainCnt = totalTrueCnts[clauseno];
      	inferredCnt =
          clause->getNumTrueGroundings(domain, domain->getDB(), false);
      	trainCnt -= defaultTrueCnts[clauseno];
      	inferredCnt -= defaultTrueCnts[clauseno];
      
      	clauseTrainCnts[clauseno] += trainCnt;
      	clauseInferredCnts[clauseno] += inferredCnt;
      }
      else
      {
      	clauseTrainCnts[clauseno] += trainCnts[clauseno];
      	clauseInferredCnts[clauseno] += inferredCnts[clauseno];
      }
      //cout << clauseno << ":\t\t" <<trainCnt<<"\t\t\t\t"<<inferredCnt<<endl;
    }

    if (vpdebug)
    {
      cout << "net counts : " << endl;
      cout << "\t\ttrain count\t\t\t\tinferred count" << endl << endl;
    }

    for (int clauseno = 0; clauseno < clauseCnt; clauseno++) 
    {
      if (!relevantClauses[clauseno]) continue;
      
      if (vpdebug)
        cout << clauseno << ":\t\t" << clauseTrainCnts[clauseno] << "\t\t\t\t"
             << clauseInferredCnts[clauseno] << endl;
      if (rescaleGradient_ && clauseTrainCnts[clauseno] > 0)
      {
        gradient[clauseno] += 
          (clauseTrainCnts[clauseno] - clauseInferredCnts[clauseno])
            / clauseTrainCnts[clauseno];
      }
      else
      {
        gradient[clauseno] += clauseTrainCnts[clauseno] - 
                              clauseInferredCnts[clauseno];
      }
    }

    delete[] clauseTrainCnts;
    delete[] clauseInferredCnts;
  }


    // Get the gradient 
  void getGradient(double* const & weights, double* const & gradient,
                   const int numWts) 
  {
    // Set the weights and run inference
    
    //cout << "New Weights = **** " << endl << endl;
    
      // If there is one db or the clauses for multiple databases line up
    if (idxTrans_ == NULL)
    {
      int clauseCnt = clauseCntPerDomain_[0];
      for (int i = 0; i < domainCnt_; i++)
      {
        Array<bool>& relevantClauses = relevantClausesPerDomain_[i];
        assert(clauseCntPerDomain_[i] == clauseCnt);
        const MLN* mln = inferences_[i]->getState()->getMLN();
        
        for (int j = 0; j < clauseCnt; j++) 
        {
          Clause* c = (Clause*) mln->getClause(j);
          if (relevantClauses[j]) c->setWt(weights[j]);
          else                    c->setWt(0);
        }
      }
    }
    else
    {   // The clauses for multiple databases do not line up
      Array<Array<double> >* wtsPerDomain = idxTrans_->getWtsPerDomain();
      const Array<Array<Array<IdxDiv>*> >* cIdxToCFIdxsPerDomain 
        = idxTrans_->getClauseIdxToClauseFormulaIdxsPerDomain();
      
      for (int i = 0; i < domainCnt_; i++)
      {
        Array<double>& wts = (*wtsPerDomain)[i];
        memset((double*)wts.getItems(), 0, wts.size()*sizeof(double));

          //map clause/formula weights to clause weights
        for (int j = 0; j < wts.size(); j++)
        {
          Array<IdxDiv>* idxDivs = (*cIdxToCFIdxsPerDomain)[i][j];          
          for (int k = 0; k < idxDivs->size(); k++)
            wts[j] += weights[ (*idxDivs)[k].idx ] / (*idxDivs)[k].div;
        }
      }
      
      for (int i = 0; i < domainCnt_; i++)
      {
        Array<bool>& relevantClauses = relevantClausesPerDomain_[i];
        int clauseCnt = clauseCntPerDomain_[i];
        Array<double>& wts = (*wtsPerDomain)[i];
        assert(wts.size() == clauseCnt);
        const MLN* mln = inferences_[i]->getState()->getMLN();

        for (int j = 0; j < clauseCnt; j++)
        {
          Clause* c = (Clause*) mln->getClause(j);
          if (relevantClauses[j]) c->setWt(wts[j]);
          else                   c->setWt(0);
        }
      }
    }
    //for (int i = 0; i < numWts; i++) cout << i << " : " << weights[i] << endl;

    if (withEM_) fillInMissingValues();
    cout << "Running inference ..." << endl;
    infer();
    cout << "Done with inference" << endl;

      // Compute the gradient
    memset(gradient, 0, numWts*sizeof(double));

      // There is one DB or the clauses of multiple DBs line up
    if (idxTrans_ == NULL)
    {
      for (int i = 0; i < domainCnt_; i++) 
      {		  
        //cout << "For domain number " << i << endl << endl; 
        getGradientForDomain(gradient, i);        
      }
    }
    else
    {
        // The clauses for multiple databases do not line up
      Array<Array<double> >* gradsPerDomain = idxTrans_->getGradsPerDomain();
      const Array<Array<Array<IdxDiv>*> >* cIdxToCFIdxsPerDomain 
        = idxTrans_->getClauseIdxToClauseFormulaIdxsPerDomain();
     
      for (int i = 0; i < domainCnt_; i++) 
      {		  
        //cout << "For domain number " << i << endl << endl; 

        Array<double>& grads = (*gradsPerDomain)[i];
        memset((double*)grads.getItems(), 0, grads.size()*sizeof(double));
        
        getGradientForDomain((double*)grads.getItems(), i);
        
          // map clause gradient to clause/formula gradients
        assert(grads.size() == clauseCntPerDomain_[i]);
        for (int j = 0; j < grads.size(); j++)
        {
          Array<IdxDiv>* idxDivs = (*cIdxToCFIdxsPerDomain)[i][j];          
          for (int k = 0; k < idxDivs->size(); k++)
            gradient[ (*idxDivs)[k].idx ] += grads[j] / (*idxDivs)[k].div;
        }
      }
    }

      // Add the deriative of the prior 
    if (usePrior_) 
    {
	  for (int i = 0; i < numWts; i++) 
      {
        if (!relevantClausesFormulas_[i]) continue;
        double priorDerivative = -(weights[i]-priorMeans_[i])/
                                 (priorStdDevs_[i]*priorStdDevs_[i]);
        //cout << i << " : " << "gradient : " << gradient[i]
        //     << "  prior gradient : " << priorDerivative;
        gradient[i] += priorDerivative; 
	    //cout << "  net gradient : " << gradient[i] << endl; 
      }
    }
  }


 private:
  int domainCnt_;
  //Array<Domain*> domains_;  
  //Array<MLN*> mlns_;
  Array<Array<double> > logOddsPerDomain_;
  Array<int> clauseCntPerDomain_;

	// Used in lazy version
  Array<Array<double> > totalTrueCnts_; 
  Array<Array<double> > defaultTrueCnts_;

  Array<Array<bool> > relevantClausesPerDomain_;
  Array<bool> relevantClausesFormulas_;

	// Used to compute cnts from mrf
  Array<double*> trainTrueCnts_;

  bool usePrior_;
  const double* priorMeans_, * priorStdDevs_; 

  IndexTranslator* idxTrans_; //not owned by object; don't delete
  
  bool lazyInference_;
  bool rescaleGradient_;
  bool isQueryEvidence_;

  Array<Inference*> inferences_;
  
    // Using EM to fill in missing values?
  bool withEM_;
};


#endif
