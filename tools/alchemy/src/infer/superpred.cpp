/*
 * All of the documentation and software included in the
 * Alchemy Software is copyrighted by Stanley Kok, Parag
 * Singla, Matthew Richardson, Pedro Domingos, Marc
 * Sumner, Hoifung Poon, and Daniel Lowd.
 * 
 * Copyright [2004-08] Stanley Kok, Parag Singla, Matthew
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
#include "util.h"
#include "mrf.h"
#include "superclause.h"
#include "superpred.h"

  //this static variable is used to intialize all the static vars appropriately
bool SuperPred::isFirstAccessForStaticVars_ = true;
Array<IntArrayToSuperPredMap*>* SuperPred::constantsToSuperPredArr_ = NULL;
Array<Array<SuperPred*>*>* SuperPred::superPredsArr_ = NULL;

  //create the super preds given the current set of super features
void createSuperPreds(Array<Array<SuperClause*>*>* const & superClausesArr,
                      Domain* const & domain)
{
  IntHashArray seenPredIds;
  Array<SuperClause *> * superClauses;

  int domainPredCnt = domain->getNumPredicates();
  Array<IntArrayToPredicateConstantsInfoMap*>* pconstantsToPCInfoArr =
    new Array<IntArrayToPredicateConstantsInfoMap*>();
  pconstantsToPCInfoArr->growToSize(domainPredCnt);
  IntArrayToPredicateConstantsInfoMap * pconstantsToPCInfo;
  IntArrayToPredicateConstantsInfoMap::iterator iaToPCInfoItr;

    //initialize the mappings from pred constants to their clause counts
  for (int i = 0; i < domainPredCnt; i++)
  {
    (*pconstantsToPCInfoArr)[i] = new IntArrayToPredicateConstantsInfoMap();
  }

  Array<int> *pconstants;
  Clause* clause;
  Predicate *pred;
  SuperClause *superClause;
  PredicateConstantsInfo *pcInfo;
  ClauseCounter  *clauseCounter;
  int superClauseCnt;

  int prevCnt = 0;
    //iterate over all the super clauses
  for (int arrIndex = 0; arrIndex < superClausesArr->size(); arrIndex++)
  {
    superClauses = (*superClausesArr)[arrIndex];
    superClauseCnt = superClauses->size();

    for (int scindex = 0; scindex < superClauseCnt; scindex++)
    {
      superClause = (*superClauses)[scindex];
      clause = superClause->getClause();
      int predCnt = clause->getNumPredicates();

        //for each tuple, extract the predicate tuple
      int numTuples = superClause->getNumTuples();
      for (int tindex = 0; tindex < numTuples; tindex++)
      {
          //number of times this tuple appears in the superclause
        double tcnt = superClause->getTupleCount(tindex);
          //this will be the count of impilcit constant tuples joining with
          //a particular predicate
        int implicitCnt;
        for (int pindex = 0; pindex < predCnt; pindex++)
        {
          pred = clause->getPredicate(pindex);
          pconstants = superClause->getPredicateConstants(tindex, pred);
          implicitCnt = superClause->getImplicitCountJoiningWithPred(tindex,
                                                                     pred);
          int predId = domain->getPredicateId(pred->getName());
          seenPredIds.append(predId);
          pconstantsToPCInfo = (*pconstantsToPCInfoArr)[predId];
          iaToPCInfoItr = pconstantsToPCInfo->find(pconstants);
          if (iaToPCInfoItr == pconstantsToPCInfo->end())
          {
            int superPredId = SuperPred::getSuperPredId(pconstants, predId);
            clauseCounter = new ClauseCounter();
            pcInfo = new PredicateConstantsInfo(clauseCounter, superPredId);
            (*pconstantsToPCInfo)[pconstants] = pcInfo;
          }
          else
          {
              //delete the newly created array if its copy was already present
            delete pconstants;
            pcInfo = iaToPCInfoItr->second;
            clauseCounter = pcInfo->getClauseCounter();
          }
          clauseCounter->incrementCount(scindex + prevCnt, pindex, predCnt,
                                        tcnt * implicitCnt);
        }
      }
    }
    prevCnt += superClauseCnt;
  }

    //now, cluster together all the pconstants having the same count
  ClauseCounterToSuperPredMap * clauseCounterToSuperPred;
  ClauseCounterToSuperPredMap::iterator ocToSpItr;
  SuperPred *superPred;

  int parentSuperPredId;

    //clear all the previously stored super preds and their counts
  SuperPred::clear(domainPredCnt);

  for (int predId = 0; predId < domainPredCnt; predId++)
  {
      //don't need to worry about the preds which did not appear in any of
      // the superclauses
    if (seenPredIds.find(predId) < 0)
      continue;

    clauseCounterToSuperPred = new ClauseCounterToSuperPredMap(); 
    pconstantsToPCInfo = (*pconstantsToPCInfoArr)[predId];
 
    for (iaToPCInfoItr = pconstantsToPCInfo->begin();
         iaToPCInfoItr != pconstantsToPCInfo->end();
         iaToPCInfoItr++)
    {
      pconstants = iaToPCInfoItr->first;
      pcInfo = iaToPCInfoItr->second;
      parentSuperPredId = pcInfo->getSuperPredId();
      clauseCounter = pcInfo->getClauseCounter();
      ocToSpItr = clauseCounterToSuperPred->find(clauseCounter);
      if (ocToSpItr == clauseCounterToSuperPred->end())
      {
        superPred = new SuperPred(predId, clauseCounter, parentSuperPredId);
        (*clauseCounterToSuperPred)[clauseCounter] = superPred;
      }
      else
      {
        delete clauseCounter;
        superPred = ocToSpItr->second;
      }
      superPred->addConstantTuple(pconstants, predId);
    }
    clauseCounterToSuperPred->clear();
    delete clauseCounterToSuperPred;
  }

    // clean up
  for (int predId = 0; predId < domainPredCnt; predId++)
  {
    pconstantsToPCInfo = (*pconstantsToPCInfoArr)[predId];
    pconstantsToPCInfo->clear();
    delete pconstantsToPCInfo;
  }
  delete pconstantsToPCInfoArr;
}

