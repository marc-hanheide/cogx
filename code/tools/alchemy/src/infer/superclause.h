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
#ifndef SUPERCLAUSE_H_NOV_2007
#define SUPERCLAUSE_H_NOV_2007

#include "util.h"
#include "mrf.h"
#include "array.h"
#include "hashint.h"
#include <ext/hash_set>
#include "variable.h"

using namespace std;
using namespace __gnu_cxx;

class SuperClause
{
 public:

  SuperClause(Clause * const & clause, Array<Variable *> * const & eqVars,
              Array<int> * const & varIdToCanonicalVarId, bool useImplicit,
              double outputWt)
  {
    int parentSuperClauseId = -1;
    outputWt_ = outputWt;
    init(clause, eqVars, varIdToCanonicalVarId, useImplicit,
         parentSuperClauseId);
  }

  SuperClause(Clause * const & clause, Array<Variable *> * const & eqVars,
              Array<int> * const & varIdToCanonicalVarId, bool useImplicit,
              int parentSuperClauseId, double outputWt)
  {
    outputWt_ = outputWt;
    init(clause, eqVars, varIdToCanonicalVarId, useImplicit,
         parentSuperClauseId);
  }

  void init(Clause * const & clause, Array<Variable *> * const & eqVars,
            Array<int> * const & varIdToCanonicalVarId, bool useImplicit,
            int parentSuperClauseId)
  {
    clause_ = clause;
    constantTuples_ = new IntArrayHashArray();
    tupleCnts_ = new Array<double>();
    implicitTupleFlags_ = new Array<bool>();
    eqVars_ = new Array<Variable *>(*eqVars);
    varIdToCanonicalVarId_ = new Array<int>(*varIdToCanonicalVarId);
    useImplicit_ = useImplicit;
    superClauseId_ = superClauseIndex__++; 
    parentSuperClauseId_ = parentSuperClauseId;
  }
                    
  ~SuperClause()
  {
    delete constantTuples_;
    delete tupleCnts_;
    delete implicitTupleFlags_;
    delete eqVars_;
  }
          
  SuperClause * createSuperClauseFromTemplate()
  {
    return new SuperClause(clause_, eqVars_, varIdToCanonicalVarId_,
                           useImplicit_, superClauseId_, outputWt_);
  }

  int getSuperClauseId() { return superClauseId_;}

  int getParentSuperClauseId() { return parentSuperClauseId_;}

  Clause * getClause() { return clause_;}

  double getTupleCount(int index) {return (*tupleCnts_)[index];}

  int getNumTuples(){ return constantTuples_->size();}

  int getTupleIndex(Array<int> * const & constants)
  {
    return constantTuples_->find(constants);
  }

  Array<int> * getConstantTuple(int tindex) {return (*constantTuples_)[tindex];}

  Array<int> * getVarIdToCanonicalVarId() { return varIdToCanonicalVarId_;}

  bool isUseImplicit() { return useImplicit_;}

  bool checkIfImplicit(Array<int> * const & constants)
  {
    bool isImplicit = false;
    for (int i = 0; i < constants->size(); i++)
    {
      if ((*eqVars_)[i] == NULL)
        continue;
      if ((*eqVars_)[i]->isImplicit((*constants)[i]))
      {
        isImplicit = true;
        break;
      }
    }
    return isImplicit;
  }

    //increments the count of the given constant tuple. Assumes that
    //tuple is already present
  void incrementTupleCount(Array<int> * const & constants, double cnt)
  {
    int index = constantTuples_->find(constants);
    (*tupleCnts_)[index] += cnt;
  }

    //add the constant tuple - returns true if the addition was
    //successful i.e. if the tuple was not already present
  bool addConstantTuple(Array<int> * const & constants)
  {
    bool isImplicit; 
    int index = constantTuples_->find(constants);
      //this tuple does not exist already then add it
    if (index < 0)
    {
      constantTuples_->append(constants);
      tupleCnts_->append(0.0);
      if (useImplicit_)
        isImplicit = checkIfImplicit(constants);
      else
        isImplicit = false;
      implicitTupleFlags_->append(isImplicit);
      return true;
    }
    else
    {
      return false;
    }
  }

    //add a copy of the constants (after replacing the non existent vars by
    //their var ids) and increment the count. 
  void addNewConstantsAndIncrementCount(Array<int> * const & constants,
                                        double cnt)
  {
    bool addedNew = false;
    Array<int>* newConstants = new Array<int>(*constants);
      //add this constant tuple to the superclause
    addedNew = addConstantTuple(newConstants);   
    incrementTupleCount(newConstants,cnt);
      //delete this newly created tuple if it was already present
    if (!addedNew)
      delete newConstants;
  }

    //get the constants corresponding to this predicate in the given tuple id
  Array<int> * getPredicateConstants(int tindex, Predicate *pred)
  {
    bool isImplicitTuple = useImplicit_ && (*implicitTupleFlags_)[tindex];
    Array<Variable*> *pvars = new Array<Variable *>();
    Array<int> * pconstants = new Array<int>;
    Array<int> * constants = (*constantTuples_)[tindex];
    for (int i = 0; i < pred->getNumTerms(); i++)
    {
      const Term *term = pred->getTerm(i);
      int id = term->getId();
      assert(id < 0);
      pconstants->append((*constants)[-id]);
        //need to do this only if implicit representation
      if (isImplicitTuple)
        pvars->append((*eqVars_)[-id]);
    }

    if (!isImplicitTuple)
    {
      delete pvars;
      return pconstants;
    }

      //to make sure this is not used below
    constants = NULL;
               
      //now standardize the implicit constants
    Array<bool> *seenIds = new Array<bool>(pconstants->size(), false);
    IntHashArray *pconstantsForVar = new IntHashArray();
    for (int i = 0; i < pconstants->size(); i++)
    {
      if (!(*pvars)[i]->isImplicit((*pconstants)[i]) || (*seenIds)[i])
        continue;
      pconstantsForVar->clear();
      for (int j = i; j < pconstants->size(); j++)
      {
        if(!(*pvars)[j]->isImplicit((*pconstants)[j]))
          continue;
        if((*pvars)[i] != (*pvars)[j])
          continue;
        pconstantsForVar->append((*pconstants)[j]);
        (*seenIds)[j] = true;
      }
                   
      for (int j = i; j < pconstants->size(); j++)
      {
        if(!(*pvars)[j]->isImplicit((*pconstants)[j]))
          continue;
        if((*pvars)[i] != (*pvars)[j])
          continue;
        int index = pconstantsForVar->find((*pconstants)[j]);
        (*pconstants)[j] = (*pvars)[i]->getImplicitConstant(index);
      }
    }

    delete pvars;
    delete seenIds;
    delete pconstantsForVar;
    return pconstants;
  }

  int getImplicitCount(Array<int> * const & constants,
                       Array<bool> * const & relevantIds,
                       Array<bool> * const & predIds)
  {
    if(!useImplicit_) return 1;
    int cnt = 1;
    bool print = false;
    Array<bool> *seenIds = new Array<bool>(constants->size(),false);
    IntHashArray *constantsForVar = new IntHashArray();
    IntHashArray *predConstantsForVar = new IntHashArray();
    if (print)
    {
      cout<<"Constants : ";
      printArray(*constants,1,cout);
      cout<<endl;
    }
    for (int i = 0; i < constants->size(); i++)
    {
      if (!(*relevantIds)[i] || (*seenIds)[i])
        continue;
      constantsForVar->clear();
      predConstantsForVar->clear();
      for (int j = i; j < constants->size(); j++)
      {
        if (!(*relevantIds)[j])
          continue;
        if ((*eqVars_)[i] != (*eqVars_)[j])
          continue;
                      
          //take a note of this if it appears in the predicate
        if (predIds && (*predIds)[j])
          predConstantsForVar->append((*constants)[j]);
        constantsForVar->append((*constants)[j]);
        (*seenIds)[j] = true;
      }
      int numImplicitConstants = (*eqVars_)[i]->getNumImplicitConstants();
      int constantsForVarSize = constantsForVar->size();
      int predConstantsForVarSize = predConstantsForVar->size();
      assert(predConstantsForVarSize <= constantsForVarSize);
                   
      if (print)
      {
        cout<<"numImplicitConstants = "<<numImplicitConstants;
        cout<<", constantsForVarSize = "<<constantsForVarSize<<endl;
        cout<<", predConstantsForVarSize = "<<predConstantsForVarSize<<endl;
      }
      cnt = cnt * Util::permute(numImplicitConstants - predConstantsForVarSize,
                                constantsForVarSize - predConstantsForVarSize);
    }

    delete seenIds;
    delete constantsForVar;
    delete predConstantsForVar;

    return cnt;
  }

    //get the count of implicit constants (partial tuples) joining with this
    //predicate in the given tuple 
  int getImplicitCountJoiningWithPred(int tindex, Predicate *pred)
  {
      //count is 1 if we are not using implicit representation
    bool isImplicitTuple = useImplicit_ && (*implicitTupleFlags_)[tindex];
    if (!isImplicitTuple)
      return 1;

    Array<int> *constants = (*constantTuples_)[tindex];
    Array<bool> *relevantIds = new Array<bool>(constants->size(),true);

    for (int i = 0; i < constants->size(); i++)
    {
      int constantId = (*constants)[i];
      if (!(*eqVars_)[i] || !((*eqVars_)[i]->isImplicit(constantId)))
      {
        (*relevantIds)[i] = false;
      }
    }

    Array<bool> *predIds = new Array<bool>(constants->size(), false);
      //those appearing in this predicate should also be ignored
      //(made irrelevant)
    for (int i = 0; i < pred->getNumTerms(); i++)
    {
      const Term *term = pred->getTerm(i);
      int id = term->getId();
      assert(id < 0);
      (*predIds)[-id] = true;
    }

    int cnt = getImplicitCount(constants, relevantIds, predIds);
    delete relevantIds;
    delete predIds;
    return cnt;
  }
          
    //get the total number of implicit tuples which are represented by the
    //tuple at given index
  int getNumImplicitTuples(int tindex)
  {
      //count is 1 if we are not using implicit representation
    bool isImplicitTuple = useImplicit_ && (*implicitTupleFlags_)[tindex];
    if (!isImplicitTuple)
      return 1;
    Array<int> *constants;
    constants = (*constantTuples_)[tindex];
    Array<bool> *relevantIds = new Array<bool>(constants->size(), true);
    for (int i = 0; i < constants->size(); i++)
    {
      int constantId = (*constants)[i];
      if (!(*eqVars_)[i] || !((*eqVars_)[i]->isImplicit(constantId)))
      {
        (*relevantIds)[i] = false;
      }
    }
    Array<bool> * predIds = NULL;
    int cnt = getImplicitCount(constants, relevantIds, predIds);
    delete relevantIds;
    return cnt;
  }
          
  int getNumTuplesIncludingImplicit()
  {
    int num = 0;
    for (int index = 0; index < constantTuples_->size(); index++)
    {
      num = num + getNumImplicitTuples(index);
    }
    return num;
  }

  double getOutputWt()
  {
    return outputWt_;
  }

  void addOutputWt(const double& outputWt)
  {
    outputWt_ += outputWt;
  }

    //print the tuples  
  ostream& print(ostream& out)
  {
    int beginIndex = 1;
    for (int i = 0; i < constantTuples_->size(); i++)
    {
      printArray(*((*constantTuples_)[i]), beginIndex, out);
      out << endl;
    }
    return out;
  }

    //static function
  void static resetIndex() { superClauseIndex__ = 0;}

 private:
  Clause* clause_;
  double wtScale_;
  IntArrayHashArray *constantTuples_;
  Array<bool> * implicitTupleFlags_;
  Array<double> * tupleCnts_;
  Array<Variable *> * eqVars_;
  Array<int> * varIdToCanonicalVarId_;
  bool useImplicit_;
  int superClauseId_;
  int parentSuperClauseId_;
  double outputWt_;
          
  static int superClauseIndex__;
};

//extern function defined in superpred.cpp
extern void createSuperClauses(
                           Array<Array<SuperClause*>*>* const & superClausesArr,
                               Domain * const & domain);

typedef hash_map<Array<int>*, SuperClause*, HashIntArray, EqualIntArray>
  IntArrayToSuperClause;

#endif
