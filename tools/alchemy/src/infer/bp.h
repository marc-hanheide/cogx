/*
 * All of the documentation and software included in the
 * Alchemy Software is copyrighted by Stanley Kok, Parag
 * Singla, Matthew Richardson, Pedro Domingos, Marc
 * Sumner, Hoifung Poon, Daniel Lowd, and Jue Wang.
 * 
 * Copyright [2004-09] Stanley Kok, Parag Singla, Matthew
 * Richardson, Pedro Domingos, Marc Sumner, Hoifung
 * Poon, Daniel Lowd, and Jue Wang. All rights reserved.
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
 * Poon, Daniel Lowd, and Jue Wang in the Department of
 * Computer Science and Engineering at the University of
 * Washington".
 * 
 * 4. Your publications acknowledge the use or
 * contribution made by the Software to your research
 * using the following citation(s): 
 * Stanley Kok, Parag Singla, Matthew Richardson and
 * Pedro Domingos (2005). "The Alchemy System for
 * Statistical Relational AI", Technical Report,
 * Department of Computer Science and Engineering,
 * University of Washington, Seattle, WA.
 * http://alchemy.cs.washington.edu.
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
#ifndef BP_H_
#define BP_H_

#include "inference.h"
#include "bpparams.h"
#include "twowaymessage.h"
#include "superclause.h"
#include "auxfactor.h"
#include "node.h"
#include "factorgraph.h"

const int bpdebug = true;

/**
 * Class for belief propagation algorithm. This version of BP works on a factor
 * graph, which can be in a lifted representation or not.
 */
class BP : public Inference
{
 public:

  /**
   * Constructor. Requires a factor graph and a set of parameters for the
   * algorithm. Optionally, a set of query formulas is used.
   */
  BP(FactorGraph* factorGraph, BPParams* bpParams,
     Array<Array<Predicate* >* >* queryFormulas = NULL)
    : Inference(NULL, -1, false, queryFormulas)
  {
    factorGraph_ = factorGraph;
    maxSteps_ = bpParams->maxSteps;
    maxSeconds_ = bpParams->maxSeconds;
    convergenceThresh_ = bpParams->convergenceThresh;
    convergeRequiredItrCnt_ = bpParams->convergeRequiredItrCnt;
    outputNetwork_ = bpParams->outputNetwork;
  }

  /**
   * Destructor.
   */
  ~BP()
  {
  }
  
  /**
   * Initializes belief propagation. The factor graph is built.
   */
  void init()
  {
    Timer timer1;
    cout << "Initializing ";
    cout << "Belief Propagation..." << endl;

    factorGraph_->init();
    if (bpdebug)
    {
      cout << "[init] ";
      Timer::printTime(cout, timer1.time());
      cout << endl;
      timer1.reset();
    }
  }

  /**
   * Performs Belief Propagation inference.
   */
  void infer()
  {
    Timer timer1;

    double oldProbs[2];
    double newProbs[2];
    double diff;
    double maxDiff;
    int maxDiffNodeIndex;
    int convergeItrCnt = 0;
    bool converged = false;
    int numFactors = factorGraph_->getNumFactors();
    int numNodes = factorGraph_->getNumNodes();

    cout << "factorcnt = " << numFactors
         << ", nodecnt = " << numNodes << endl;

    if (bpdebug)
    {
      cout << "factors:" << endl;
      for (int i = 0; i < numFactors; i++)
      {
        factorGraph_->getFactor(i)->print(cout);
        cout << endl;
      }
      cout << "nodes:" << endl;
      for (int i = 0; i < numNodes; i++)
      {
        factorGraph_->getNode(i)->print(cout);
        cout << endl;
      }
    }
    
      // Pass around (send) the messages
    int itr;
    
      // Move to next step to transfer the message in the nextMsgsArr in the
      // beginning to the the msgsArr 
    for (itr = 1; itr <= maxSteps_; itr++)
    {
      if (bpdebug)
      {
        cout<<"*************************************"<<endl;
        cout<<"Performing Iteration "<<itr<<" of BP"<<endl;
        cout<<"*************************************"<<endl;
      }

      for (int i = 0; i < numFactors; i++)
      {
        if (bpdebug)
        {
          cout << "Sending messages for Factor: ";
          factorGraph_->getFactor(i)->print(cout); cout << endl;
        }
        factorGraph_->getFactor(i)->sendMessage();
      }
      
      for (int i = 0; i < numNodes; i++)
      {
        if (bpdebug)
        {
          cout << "Sending messages for Node: ";
          factorGraph_->getNode(i)->print(cout); cout << endl;
        }
        factorGraph_->getNode(i)->sendMessage();
      }

      for (int i = 0; i < numFactors; i++)
      {
        factorGraph_->getFactor(i)->moveToNextStep();
        if (bpdebug)
        {
          cout << "BP-Factor Iteration " << itr << " => ";
          factorGraph_->getFactor(i)->print(cout); cout << endl;
        }
      }
          
      maxDiff = -1;
      maxDiffNodeIndex = -1;
      for (int i = 0; i < numNodes; i++)
      {
        if (bpdebug)
        {
          cout<<"************************************"<<endl;
          cout<<"Node "<<i<<":"<<endl;
          cout<<"************************************"<<endl;
          cout<<"Getting Old Probabilities =>"<<endl; 
          cout<<endl;
          cout<<"Moving to next step "<<endl;
          cout<<endl;
          cout<<"Getting New Probabilities =>"<<endl; 
        }

        factorGraph_->getNode(i)->getProbs(oldProbs);
        factorGraph_->getNode(i)->moveToNextStep();
        factorGraph_->getNode(i)->getProbs(newProbs);

        diff = abs(newProbs[1] - oldProbs[1]);

        if (bpdebug)
        {
          cout << endl << endl << "Final Probs : " << endl;
          cout << "Node " << i << ": probs[" << 0 << "] = " << newProbs[0]
               << ", probs[" << 1 << "] = " << newProbs[1] << endl;
          cout << "BP-Node Iteration " << itr << ": " << newProbs[0]
               << "  probs[" << 1 << "] = " << newProbs[1] << endl;
          cout << " : => ";
          factorGraph_->getNode(i)->print(cout);
          cout << endl;
        }
        
        if (maxDiff < diff)
        {
          maxDiff = diff;
          maxDiffNodeIndex = i;
        }
      }
          
      cout << "At Iteration " << itr << ": MaxDiff = " << maxDiff << endl;
      cout << endl;
           
        //check if BP has converged
      if (maxDiff < convergenceThresh_)
        convergeItrCnt++;
      else
        convergeItrCnt = 0;

        // Check if for N continuous iterations, maxDiff has been below the
        // threshold
      if (convergeItrCnt >= convergeRequiredItrCnt_)
      {
        converged = true;
        break;
      }
    }

    if (converged)
    {
      cout << "Converged in " << itr << " Iterations " << endl;
    }
    else
    {
      cout << "Did not converge in " << maxSteps_ << " (max allowed) Iterations"
           << endl;
    }
    
    if (queryFormulas_)
    {
      cout << "Computing probabilities of query formulas ..." << endl;
      for (int i = 0; i < numNodes; i++)
      {
        if (bpdebug)
        {
          cout << "Sending auxiliary messages for Node: ";
          factorGraph_->getNode(i)->print(cout); cout << endl;
        }
        factorGraph_->getNode(i)->sendAuxMessage();
          // Now, messages have been sent to the aux. factors
      }
      for (int j = 0; j < qfProbs_->size(); j++)
      {
        (*qfProbs_)[j] = factorGraph_->getAuxFactor(j)->getProb();
      }
    }
  }

  /**
   * Prints out the network.
   */
  void printNetwork(ostream& out)
  {
    factorGraph_->printNetwork(out);
  }

  /**
   * Prints the probabilities of each predicate to a stream.
   */
  void printProbabilities(ostream& out)
  {
    double probs[2];
    Array<int>* constants;
    Predicate* pred;
    int predId;
    Node* node;
    double exp;
    Domain* domain = factorGraph_->getDomain();
    for (int i = 0; i < factorGraph_->getNumNodes(); i++)
    { 
      node = factorGraph_->getNode(i);
      predId = node->getPredId();
      node->getProbs(probs);
      exp = node->getExp();
      SuperPred * superPred = node->getSuperPred();

      if (superPred)
      {        
        for (int index = 0; index < superPred->getNumTuples(); index++)
        {
          constants = superPred->getConstantTuple(index);
          pred = domain->getPredicate(constants, predId);
          pred->printWithStrVar(out, domain);
          out << " " << probs[1] << endl;
          //out<<" "<<exp<<endl;
        }
      }
      else
      {
        constants = node->getConstants(); 
        assert(constants != NULL);
        pred = domain->getPredicate(constants, predId);
        pred->printWithStrVar(out, domain);
        out << " " << probs[1] << endl;
        //out<<" "<<exp<<endl;
      }
    }
  }

  /**
   * Puts the predicates whose probability has changed with respect to the
   * reference vector oldProbs by more than probDelta in string form and the
   * corresponding probabilities of each predicate in two vectors. Currently
   * not implemented.
   * 
   * @param changedPreds Predicates whose probability have changed more than
   * probDelta are put here.
   * @param probs The probabilities corresponding to the predicates in
   * changedPreds are put here.
   * @param oldProbs Reference probabilities for checking for changes.
   * @param probDelta If probability of an atom has changed more than this
   * value, then it is considered to have changed.
   */
  void getChangedPreds(vector<string>& changedPreds, vector<float>& probs,
                       vector<float>& oldProbs, const float& probDelta)
  {
  }

  /**
   * Gets the probability of a ground predicate.
   * 
   * @param gndPred GroundPredicate whose probability is being retrieved.
   * @return Probability of gndPred if present in state, otherwise 0.
   */
  double getProbability(GroundPredicate* const& gndPred)
  {
    double probs[2];
    Array<int>* constants;
    Predicate* pred;
    unsigned int predId;
    Node* node;
    Domain* domain = factorGraph_->getDomain();
    bool found = false;
    for (int i = 0; i < factorGraph_->getNumNodes(); i++)
    { 
      node = factorGraph_->getNode(i);
      predId = node->getPredId();
      if (predId != gndPred->getId()) continue;
      node->getProbs(probs);         
      SuperPred * superPred = node->getSuperPred();

      if (superPred)
      {        
        for (int index = 0; index < superPred->getNumTuples(); index++)
        {
          constants = superPred->getConstantTuple(index);
          pred = domain->getPredicate(constants, predId);
          if (!pred->same(gndPred))
          {
            delete pred;
            continue;
          }
          delete pred;
          found = true;
          return probs[1];
        }
      }
      else
      {
        constants = node->getConstants(); 
        assert(constants != NULL);
        pred = domain->getPredicate(constants, predId);
        if (!pred->same(gndPred))
        {
          delete pred;
          continue;
        }
        delete pred;
        found = true;
        return probs[1];
      }
    }
    return 0.5;
  }

  /**
   * Gets the probability of a ground predicate. Currently not implemented.
   * 
   * @param gndPred GroundPredicate whose probability is being retrieved.
   * @return Probability of gndPred if present in state, otherwise 0.
   */
  double getProbabilityH(GroundPredicate* const& gndPred)
  {
    return 0.0;
  }

  /**
   * Prints each predicate with a probability of 0.5 or greater to a stream.
   * Currently not implemented.
   */
  void printTruePreds(ostream& out)
  {
  }
  
  /**
   * Prints each predicate with a probability of 0.5 or greater to a stream.
   * Currently not implemented.
   */
  void printTruePredsH(ostream& out)
  {
  }

 private:
    // Network on which BP is run
  FactorGraph* factorGraph_;
    // Max. no. of BP iterations to perform
  int maxSteps_;
    // Max. no. of seconds BP should run
  int maxSeconds_;
    // Maximum difference between probabilities must be less than this
    // in order to converge
  double convergenceThresh_;
    // Convergence must last this number of iterations
  int convergeRequiredItrCnt_;
    // No inference is run, rather the factor graph is built
  bool outputNetwork_;
};

#endif /*BP_H_*/
