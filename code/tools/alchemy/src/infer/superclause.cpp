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

int SuperClause::superClauseIndex__ = 0;

  //create the super clauses given the current set of super preds
void createSuperClauses(Array<Array<SuperClause*>*>* const & superClausesArr,
                        Domain * const & domain)
{
    //reset the index of superClauses (i.e. we will assign new ids starting
    //from 0)
  SuperClause::resetIndex();

  IntArrayToSuperClause * superPredIdsToSuperClause;
  IntArrayToSuperClause::iterator iaToScItr;
  Array<int> *pconstants;
  Array<int> *superPredIds;
  Clause* clause;
  SuperClause *superClause, *newSuperClause;
  Predicate *pred;
  Array<SuperClause *> *superClauses;
  Array<SuperClause *> *newSuperClauses;
  int superClauseCnt;

    //iterate over all the super clauses
  for (int arrIndex = 0; arrIndex < superClausesArr->size(); arrIndex++)
  {
    superClauses = (*superClausesArr)[arrIndex];
    superClauseCnt = superClauses->size();
    newSuperClauses = new Array<SuperClause*>();
    superPredIdsToSuperClause = new IntArrayToSuperClause();

      //iterate over all the super clauses
    for (int scindex = 0; scindex < superClauseCnt; scindex++)
    {
      superPredIdsToSuperClause->clear();
      superClause = (*superClauses)[scindex];
      clause = superClause->getClause();
      int predCnt = clause->getNumPredicates();

        //for each tuple, first extract the predicate tuple and
        //then corresponding superpred id
      int tupleCnt = superClause->getNumTuples();
      for (int tindex = 0; tindex < tupleCnt; tindex++)
      {
        superPredIds = new Array<int>();
        for (int pindex = 0; pindex < predCnt; pindex++)
        {
          pred = clause->getPredicate(pindex);
          pconstants = superClause->getPredicateConstants(tindex, pred);
          int predId = domain->getPredicateId(pred->getName());
          superPredIds->append(SuperPred::getSuperPredId(pconstants, predId));
          delete pconstants;
        }
        iaToScItr = superPredIdsToSuperClause->find(superPredIds);
        if (iaToScItr == superPredIdsToSuperClause->end())
        {
          newSuperClause = superClause->createSuperClauseFromTemplate();
          (*superPredIdsToSuperClause)[superPredIds] = newSuperClause;
          newSuperClauses->append(newSuperClause);
        }
        else
        {
          delete superPredIds;
          newSuperClause = iaToScItr->second;
        }
        double tcnt = superClause->getTupleCount(tindex);
        Array<int> * constants = superClause->getConstantTuple(tindex);
        newSuperClause->addConstantTuple(constants);
        newSuperClause->incrementTupleCount(constants, tcnt);
      }

        //trick to dereference/delete the keys of the map
        //- first extract all the keys, then clear the map,
        //then delete the keys
      Array<Array<int>*> keysArr;
      keysArr.clear();
      for (iaToScItr = superPredIdsToSuperClause->begin();
           iaToScItr != superPredIdsToSuperClause->end();
           iaToScItr++)
      {
        superPredIds = iaToScItr->first;
        keysArr.append(superPredIds);
      }
      superPredIdsToSuperClause->clear();
      for (int i = 0; i < keysArr.size(); i++)
      {
        delete keysArr[i];
      }
    }

      //clean up
    delete superPredIdsToSuperClause;
    for (int i = 0; i < superClauses->size(); i++)
    {
      delete (*superClauses)[i];
    }
    superClauses->clear();

      //store the newly create superclauses into the array of superclauses
    for (int i = 0; i < newSuperClauses->size(); i++)
    {
      superClauses->append((*newSuperClauses)[i]);
    }
    delete newSuperClauses;
  }
}


