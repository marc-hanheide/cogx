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
#ifndef _SUPERPRED_H_DEC_2007
#define _SUPERPRED_H_DEC_2007

#include "superclause.h"

/******************************************************************************/
// Clause Counter
/******************************************************************************/

//used in the following class
typedef hash_map<int, int, HashInt, EqualInt> IntToIntMap;

//this class is for maintaining the counts of for an indexed set of
//clauses (where each entity has a unique id)
class ClauseCounter
{
 public:
  ClauseCounter()
  {
    idToIndex_ = new IntToIntMap();
    clauseIds_ = new Array<int>;
    clauseCntsArr_ = new Array<Array<double>*>;
    dirty_ = true;
  }

  ~ClauseCounter()
  {
    delete idToIndex_;
    delete clauseIds_;
    for (int i = 0; i < clauseCntsArr_->size(); i++)
    {
      delete (*clauseCntsArr_)[i];
    }
    delete clauseCntsArr_;
  }

  int getNumClauses() { return clauseIds_->size();}

  const Array<int>* getClauseIds() const { return clauseIds_;}

  const Array<double>* getClauseCounts(int index) const
  {
    return (*clauseCntsArr_)[index];
  }

  int  getClauseId(int index) { return (*clauseIds_)[index];}

  void incrementCount(int clauseId, int predId, int clausePredCnt, double cnt)
  {
    int index;
    int id = clauseId;
    Array<double> *clauseCnts;
    IntToIntMap::iterator itr;
    itr = idToIndex_->find(id);

    if (itr == idToIndex_->end())
    {
      index = clauseIds_->size();
      (*idToIndex_)[id] = index;
      clauseIds_->append(id);

      clauseCnts = new Array<double>();
      clauseCnts->growToSize(clausePredCnt, 0);
      clauseCntsArr_->append(clauseCnts);
    }
    else
    {
      index = (*idToIndex_)[id];
    }

      //increment the count for this clauseId/predId combination
    clauseCnts = (*clauseCntsArr_)[index];
    (*clauseCnts)[predId] += cnt;
    dirty_ = true;
  }

  size_t getHashCode()
  {
    if (!dirty_)
      return hashCode_;
    IntToIntMap::iterator itr;
    Array<double> * clauseCnts;
    int code = 1;
    for (int index = 0; index < clauseIds_->size(); index++)
    {
      code = 31*code + (*clauseIds_)[index];
      clauseCnts = (*clauseCntsArr_)[index];
      for (int predId = 0; predId < clauseCnts->size(); predId++)
      {
        code = 31*code + (int)(*clauseCnts)[predId];
      }
    }
    dirty_ = false;
    hashCode_ = (size_t)code;
    return hashCode_;
  }

  ostream & print(ostream & out)
  {
    for (int index = 0; index < clauseIds_->size(); index++)
    {
      out<<(*clauseIds_)[index]<<" : ";
      printArray(*(*clauseCntsArr_)[index], out);
      out<<" ** ";
    }
    return out;
  }

 private:
  IntToIntMap *idToIndex_;

  //NOTE: Assumes that clauseIds_ are stored in a pre-defined order for
  //all the ClauseCounter instances. This is critical for making
  //sure that hashCode/Equal functions on two instances of this
  //class work as expected

  Array<int> * clauseIds_;
  Array<Array<double>*> * clauseCntsArr_;
  bool dirty_;
  size_t hashCode_;
};

class PredicateConstantsInfo
{
 public:
  PredicateConstantsInfo(ClauseCounter * const & cc, int superPredId)
  {
    cc_ = cc;
    superPredId_ = superPredId;
  }

  ~PredicateConstantsInfo()
  {
    delete cc_;
  }

  int getSuperPredId() { return superPredId_;}
  ClauseCounter * getClauseCounter() {return cc_;}

 private:
  int superPredId_;
  ClauseCounter *cc_;
};


class SuperPred;
typedef hash_map<Array<int>*, SuperPred*, HashIntArray, EqualIntArray>
  IntArrayToSuperPredMap;

//extern function defined in superpred.cpp
extern void createSuperPreds(
                           Array<Array<SuperClause*>*>* const & superClausesArr,
                             Domain * const & domain);

class SuperPred
{
 public:
  SuperPred(int & predId, ClauseCounter* const & clauseCounter,
            int parentSuperPredId)
  {
    constantTuples_ = new Array<Array<int>*>;
    predId_ = predId;

      //find the id of this superpred - this is simply the
      //current cnt of the superpreds for this predId
    Array<SuperPred *> * superPreds = (*(SuperPred::superPredsArr_))[predId];
    superPredId_ = superPreds->size();
    superPreds->append(this);
    clauseCounter_ = clauseCounter;
    parentSuperPredId_ = parentSuperPredId;
  }

    //not responsible for deleting the constants
  ~SuperPred()
  {
    delete constantTuples_;
      //note: though, clauseCounter is allocated memory somewhere outside,
      //it is delete here
    delete clauseCounter_;
  }

  int getPredId() {return predId_;}

  Array<int> * getConstantTuple(int tindex) {return (*constantTuples_)[tindex];}

  int getSuperPredId() {return superPredId_;}

  int getParentSuperPredId() {return parentSuperPredId_;}

  int getNumTuples() { return constantTuples_->size();}
 
  const ClauseCounter * getClauseCounter() { return clauseCounter_;}

  void addConstantTuple(Array<int> * constants, int predId)
  {
    constantTuples_->append(constants);
    IntArrayToSuperPredMap * constantsToSuperPred;
    constantsToSuperPred = (*(SuperPred::constantsToSuperPredArr_))[predId];
    (*constantsToSuperPred)[constants] = this;
  }

		  //static functions
  static int getSuperPredCount(int predId)
  {
    return (*superPredsArr_)[predId]->size();
  }

    //get all the superpreds for the given predId
  static Array<SuperPred*>* getSuperPreds(int predId)
  {
    return (*superPredsArr_)[predId];
  }

  static void clear(int predCnt)
  {
      //if first time access then, need to allocate memory
    if (isFirstAccessForStaticVars_)
    {
      superPredsArr_ = new Array<Array<SuperPred *>*>();
      constantsToSuperPredArr_ = new Array<IntArrayToSuperPredMap *>();
      for (int predId = 0; predId < predCnt; predId++)
      {
        constantsToSuperPredArr_->append(new IntArrayToSuperPredMap());
        superPredsArr_->append(new Array<SuperPred *>());
      }
      isFirstAccessForStaticVars_ = false;
    }
    else
    {
        //this is not the first time access - need to clean up
      Array<SuperPred *> *superPreds;
      IntArrayToSuperPredMap * constantsToSuperPred;
      IntArrayToSuperPredMap::iterator iaToSpItr;
      for (int predId = 0; predId < predCnt; predId++)
      {
          //first delete super preds	 
        superPreds = (*superPredsArr_)[predId];
        for (int i = 0; i < superPreds->size(); i++)
        {
          delete (*superPreds)[i];
        }

          //now delete the constants
        Array<Array<int> *> keysArr;
        constantsToSuperPred = (*constantsToSuperPredArr_)[predId];
        keysArr.clear();
        for (iaToSpItr = constantsToSuperPred->begin();
             iaToSpItr != constantsToSuperPred->end();
             iaToSpItr++)
        {
          keysArr.append(iaToSpItr->first);
        }
        for (int i = 0; i < keysArr.size(); i++)
        {
          delete keysArr[i];
        }

          //reinitialize
        superPreds->clear();
        constantsToSuperPred->clear();
      }
    }
  }

  static int getSuperPredId(Array<int> * constants, int & predId)
  {
    IntArrayToSuperPredMap * constantsToSuperPred;
    IntArrayToSuperPredMap::iterator iaToSpItr;
    if (isFirstAccessForStaticVars_)
      return -1;
    assert(constantsToSuperPredArr_);
    constantsToSuperPred = (*constantsToSuperPredArr_)[predId];
    iaToSpItr = constantsToSuperPred->find(constants); 
    SuperPred *superPred = iaToSpItr->second;
    return superPred->getSuperPredId();
  }

 private:
  int predId_;
  int superPredId_;
  int parentSuperPredId_;
  Array<Array<int> *> *constantTuples_;
  ClauseCounter *clauseCounter_;

  static bool isFirstAccessForStaticVars_;
  static Array<IntArrayToSuperPredMap*>* constantsToSuperPredArr_;
  static Array<Array<SuperPred *>*>* superPredsArr_;
};


class HashClauseCounter
{
 public:
  size_t operator()(ClauseCounter *cc) const
  {
    return cc->getHashCode();
  }
};


class EqualClauseCounter
{
 public:
  bool operator()(ClauseCounter* const & cc1, ClauseCounter* const & cc2) const
  {
    bool same;
    const int *items1, *items2;
    const Array<double> *cnts1, *cnts2;

    int size1, size2;

    size1 = cc1->getNumClauses();
    size2 = cc2->getNumClauses();

    if (size1 != size2) return false;

      //first check if the clause ids match
    items1 = (cc1->getClauseIds())->getItems();
    items2 = (cc2->getClauseIds())->getItems();

    same = memcmp(items1, items2, size1*sizeof(int))==0;
    if (!same)
      return same;

    double epsilon = 1e-6;
      //now check if the clause cnts match for each of the indices 
      //(no need check the index sizes again - they must be same at this point)
    for (int index = 0; index < size1; index++)
    {
      cnts1 = cc1->getClauseCounts(index);
      cnts2 = cc2->getClauseCounts(index);
      for (int i = 0;i < cnts1->size(); i++)
      {
        if(((*cnts1)[i] + epsilon < (*cnts2)[i]) || 
           ((*cnts1)[i] - epsilon > (*cnts2)[i]))
          return false;
      }
    }
    return true;
  }
};

typedef hash_map<Array<int>*, PredicateConstantsInfo*, HashIntArray,
                 EqualIntArray> IntArrayToPredicateConstantsInfoMap;
typedef hash_map<ClauseCounter*, SuperPred*, HashClauseCounter,
                 EqualClauseCounter> ClauseCounterToSuperPredMap;

#endif
