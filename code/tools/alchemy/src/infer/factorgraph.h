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
#ifndef FACTORGRAPH_H_
#define FACTORGRAPH_H_

#include "twowaymessage.h"
#include "superclause.h"
#include "auxfactor.h"
#include "node.h"

const int fgdebug = false;

/**
 * Class for a factor graph. It consists of an array of nodes and an array of
 * factors. The graph can be lifted or not, depending on if the lifted_ flag is
 * set or not. An optional array of auxiliary factors holds factors which are
 * attached to some nodes, but which do not send messages. These are used to
 * compute probabilities of query formulas (after inference is run, the
 * messages from the nodes attached to the auxiliary factors are to be sent.
 * Marginal probabilities are read directly from the nodes after inference is
 * run.
 */
class FactorGraph
{
 public:

  /**
   * Constructor. Data structures are initialized.
   */
  FactorGraph(bool lifted, MLN* mln, Domain* domain,
              Array<Array<Predicate* >* >* queryFormulas = NULL)
  {
    lifted_ = lifted;
    
    lidToTWMsg_ = new LinkIdToTwoWayMessageMap();
    superClausesArr_ = new Array<Array<SuperClause*>*>();
    factors_ = new Array<Factor*>();
    nodes_ = new Array<Node*>();
    mln_ = mln;
    domain_ = domain;
    
    auxFactors_ = NULL;
    if (queryFormulas)
    {
      auxFactors_ = new Array<AuxFactor*>();
      for (int i = 0; i < queryFormulas->size(); i++)
        auxFactors_->append(new AuxFactor((*queryFormulas)[i]));
    }
  }

  /**
   * Destructor. Data structures are destroyed.
   */
  ~FactorGraph()
  {
    delete lidToTWMsg_;
    delete superClausesArr_;
    delete factors_;
    delete nodes_;
    if (auxFactors_) delete auxFactors_;
  }
  
  /**
   * Builds the factor graph. If lifted_ is true, then the graph will be in a
   * lifted representation.
   */
  void init()
  {
    Timer timer1;
    cout << "Building ";
    if (lifted_) cout << "Lifted ";
    cout << "Factor Graph..." << endl;

    if (lifted_)
    {
      createSuper();
      createSuperNetwork();
    }
    else
    {
      createGround();
      createGroundNetwork();
    }
    
    if (fgdebug)
    {
      cout << "[init] ";
      Timer::printTime(cout, timer1.time());
      cout << endl;
      timer1.reset();
    }
  }

  /**
   * Prints out the network. The format is:
   * variables:
   * [var1]
   * [var2]
   * [var3]
   * ....
   * [varn]
   * factors:
   * [vari1] / [varj1] // [log table] /// [weight to vari1] [weight to varj1]
   * [vari2] / [varj2] / [vark2] // [log table] /// [weight to vari2] [weight to varj2] [weight to vark2]
   * 
   * The first section of the file begins with a line containing only the text
   * "variables:".
   * Following which is list of of all the variable names. The only constraint
   * for variable names is that the variable name must not contain the '/'
   * symbol.
   *
   * The second section of file begins with a line containing only the text
   * "factors:".
   * Following which is list of of all the factors with one factor per line.
   * To demonstrate how the factor is stored, lets consider a factor F(X,Y,Z)
   * with binary variables.
   *
   * The part of the line to the left of the string '//' is a list of all the
   * variables in the factor, seperated by the '/' symbol.
   *
   * For instance, for the factor F(X,Y,Z), we will write
   * X / Y / Z // .....
   * The rest of the line is the table for log F(X,Y,Z) written in the iteration
   * order:
   * [log F(0,0,0)] [log F(1,0,0)] [log F(0,1,0)] [log F(1,1,0)] [log F(0,0,1)]
   * (basically increment the left most variable until it hits the limit, then
   * reset and increment the next variable and so on)
   * 
   * The part of each factor line to the right of '///' is a list of integer
   * weights which represent the weight of the edge connecting the factor to
   * each adjacent variable.
   * 
   */
  void printNetwork(ostream& out)
  {
    out << "variables:" << endl;
    for (int i = 0; i < nodes_->size(); i++)
    {
      (*nodes_)[i]->print(out);
      out << endl;
    }
    out << "factors:" << endl;
    for (int i = 0; i < factors_->size(); i++)
    {
      Factor* factor = (*factors_)[i];
      factor->print(out);
      out << "// ";
      factor->printWts(out);
      if (lifted_)
      {
        out << " ///";
        int numLinks = factor->getNumLinks();
        for (int lno = 0; lno < numLinks; lno++)
        {
          Link* link = factor->getLink(lno);
          Node* node = link->getNode();
          double cnt = node->getGroundNodeCount();
          out << " " << cnt;
        }
      }
      out << endl;
    }
  }

  /**
   * Returns the link to message map.
   */
  LinkIdToTwoWayMessageMap* getLinkIdToTwoWayMessageMap()
  {
    return lidToTWMsg_;
  }

  /**
   * Returns the number of nodes in the graph.
   */
  const int getNumNodes()
  {
    return nodes_->size();
  }
  
  /**
   * Returns the number of factors in the graph.
   */
  const int getNumFactors()
  {
    return factors_->size();
  }
  
  /**
   * Returns the number of auxiliary factors in the graph.
   */
  const int getNumAuxFactors()
  {
    if (auxFactors_ == NULL) return 0;
    return auxFactors_->size();
  }
  
  /**
   * Returns a node in the graph.
   * 
   * @param index Index of the node to be retrieved.
   */
  Node* getNode(const int& index)
  {
    return (*nodes_)[index];
  }

  /**
   * Returns a factor in the graph.
   * 
   * @param index Index of the factor to be retrieved.
   */
  Factor* getFactor(const int& index)
  {
    return (*factors_)[index];
  }

  /**
   * Returns an auxiliary factor in the graph.
   * 
   * @param index Index of the auxiliary factor to be retrieved.
   */
  AuxFactor* getAuxFactor(const int& index)
  {
    return (*auxFactors_)[index];
  }

  /**
   * Returns the domain on which the factor graph is built.
   */
  Domain* getDomain()
  {
    return domain_;
  }

 private:
 
 /**
  * Ground away the evidence from each clause. Then, create one 
  * superclause for each ground clause
  */
  void createGround()
  {
    MLN* mln = mln_;
    Domain* domain = domain_;
    Clause* mlnClause;

      //clause with all variables in it
    Clause *varClause;

    Array<Clause *> allClauses;
    Array<int> *mlnClauseTermIds;
  
    ClauseToSuperClauseMap *clauseToSuperClause;
    ClauseToSuperClauseMap::iterator clauseItr;
    int numClauses = mln->getNumClauses();
  
    PredicateTermToVariable *ptermToVar = NULL;
  
    double gndtime;
    Util::elapsed_seconds();
  
    clauseToSuperClause = new ClauseToSuperClauseMap();
    for (int i = 0; i < numClauses; i++)
    {
        //remove the unknown predicates
      mlnClause = (Clause*) mln->getClause(i);
      varClause = new Clause(*mlnClause);
      mlnClauseTermIds = varClause->updateToVarClause();
     
      mlnClause->getConstantTuples(domain, domain->getDB(), mlnClauseTermIds, 
                                   varClause, ptermToVar, clauseToSuperClause,
                                   false);

        //can delete the var Clause now
      delete varClause;
    }

    clauseToSuperClause = mergeSuperClauses(clauseToSuperClause);
    addSuperClauses(clauseToSuperClause);
  
    gndtime = Util::elapsed_seconds();

      //create the super preds  
    int totalGndClauseCnt = 0;
    for (int i = 0; i < superClausesArr_->size(); i++)
    {
      SuperClause *superClause = (*(*superClausesArr_)[i])[0];
      int gndCnt = superClause->getNumTuplesIncludingImplicit();
      totalGndClauseCnt += gndCnt;
    }
    cout<<"Total Number of Ground Clauses = "<<totalGndClauseCnt<<endl;
  }
 
 
  /**
   * Creates the ground network.
   */
  void createGroundNetwork()
  {
    Domain* domain = domain_;
    int domainPredCnt = domain->getNumPredicates();
    Array<IntArrayToIntMap*>* pconstantsToNodeIndexArr =
      new Array<IntArrayToIntMap*>();
    pconstantsToNodeIndexArr->growToSize(domainPredCnt);
     
      // Initialize the mappings from pred constants to the nodeIndices
    for(int i = 0; i < domainPredCnt; i++)
    {
      (*pconstantsToNodeIndexArr)[i] = new IntArrayToIntMap();
    }
     
    IntArrayToIntMap *pconstantsToNodeIndex;
    IntArrayToIntMap::iterator itr;
     
    Array<SuperClause*>* superClauses;
    SuperClause* superClause;

    Clause *clause;
    Predicate *pred;
    Array<int>* constants;
    Array<int>* pconstants;
    
    Factor* factor;
    Node* node;

    for (int arrIndex = 0; arrIndex < superClausesArr_->size(); arrIndex++)
    {
      superClauses = (*superClausesArr_)[arrIndex];
      for (int scindex = 0; scindex < superClauses->size(); scindex++)
      {
        superClause = (*superClauses)[scindex];
        clause = superClause->getClause();
        int numTuples = superClause->getNumTuples();
        for (int tindex = 0; tindex < numTuples; tindex++)
        {
            // Number of times this tuple appears in the superclause
            // (because of counting elimination)
          double tcnt = superClause->getTupleCount(tindex);
          constants = superClause->getConstantTuple(tindex);

            // MS: Set to real weight, not 1
          //clause->setWt(superClause->getOutputWt());
          factor = new Factor(clause, NULL, constants, domain,
                                superClause->getOutputWt());
          factors_->append(factor);
          for (int pindex = 0; pindex < clause->getNumPredicates(); pindex++)
          {
            pred = clause->getPredicate(pindex);
            pconstants = pred->getPredicateConstants(constants);
            
            int predId = domain->getPredicateId(pred->getName());
            pconstantsToNodeIndex = (*pconstantsToNodeIndexArr)[predId];
            itr = pconstantsToNodeIndex->find(pconstants);
                 
            if (itr == pconstantsToNodeIndex->end())
            {
              int nodeIndex = nodes_->size();
              (*pconstantsToNodeIndex)[pconstants] = nodeIndex;

              node = new Node(predId, NULL, pconstants, domain);
              nodes_->append(node);
            }
            else
            {
              delete pconstants;
              int nodeIndex = itr->second;
              node = (*nodes_)[nodeIndex];
            }

              // Now, add the links to the node/factor
            int reverseNodeIndex = factor->getNumLinks();
              // Index where this factor would be stored in the list of nodes
            int reverseFactorIndex = node->getNumLinks();
            Link * link = new Link(node, factor, reverseNodeIndex,
                                       reverseFactorIndex, pindex, tcnt);
            node->addLink(link, NULL);
            factor->addLink(link, NULL);
          }
        }
      }
    }

      // If query formulas present, make aux. links to the nodes
    if (auxFactors_)
    {
      for (int i = 0; i < auxFactors_->size(); i++)
      {
        AuxFactor* auxFactor = (*auxFactors_)[i];
        Array<Predicate* >* formula = auxFactor->getFormula();
        for (int pindex = 0; pindex < formula->size(); pindex++)
        {
          double z = 1.0;
          pred = (*formula)[pindex];
          pconstants = pred->getPredicateConstants();
          int predId = domain->getPredicateId(pred->getName());

          pconstantsToNodeIndex = (*pconstantsToNodeIndexArr)[predId];
          itr = pconstantsToNodeIndex->find(pconstants);

          if (itr == pconstantsToNodeIndex->end())
          {
            cout << "ERROR: couldn't find predicate ";
            pred->printWithStrVar(cout, domain);
            cout << " from query formula in factor graph" << endl;
            exit(-1);
          }
          else
          {
            delete pconstants;
            int nodeIndex = itr->second;
            node = (*nodes_)[nodeIndex];
          }

            // Now, add the links to the node/factor
          int reverseNodeIndex = auxFactor->getNumLinks();
            // Index where this factor would be stored in the list of nodes
          int reverseFactorIndex = node->getNumLinks();
          Link* link = new Link(node, auxFactor, reverseNodeIndex,
                                    reverseFactorIndex, pindex, z);
          node->addAuxLink(link);
          auxFactor->addLink(link, NULL);
        }
      }
    }

      // clean up
    for (int predId = 0; predId < domainPredCnt; predId++)
    {
      pconstantsToNodeIndex = (*pconstantsToNodeIndexArr)[predId];
      pconstantsToNodeIndex->clear();
      delete pconstantsToNodeIndex;
    }
    delete pconstantsToNodeIndexArr;
    cout << "Created Ground Network" << endl;
  }

 /**
  * Ground away the evidence from each clause. Then,
  * create the super clause/predicate network
  */
  void createSuper()
  {
    MLN* mln = mln_;
    Domain* domain = domain_;
    Clause* mlnClause;

      //clause with all variables in it
    Clause *varClause;
    Array<Clause *> allClauses;
    Array<int> *mlnClauseTermIds;
       
    ClauseToSuperClauseMap *clauseToSuperClause;
    ClauseToSuperClauseMap::iterator clauseItr;

    int numClauses = mln->getNumClauses();
       
    PredicateTermToVariable *ptermToVar = NULL;
    double gndtime, setuptime;
    Util::elapsed_seconds();

    if (implicitRep_)
    {
      ptermToVar = getPredicateTermToVariableMap(mln, domain);
      getIndexedConstants(ptermToVar, mln, domain);
    }
      
    clauseToSuperClause = new ClauseToSuperClauseMap();
    for (int i = 0; i < numClauses; i++)
    {
        //remove the unknown predicates
      mlnClause = (Clause*) mln->getClause(i);
      varClause = new Clause(*mlnClause);
      mlnClauseTermIds = varClause->updateToVarClause();
          
      mlnClause->getConstantTuples(domain, domain->getDB(), mlnClauseTermIds, 
                                   varClause, ptermToVar, clauseToSuperClause,
                                   implicitRep_);
          
        //can delete the var Clause now
      delete varClause;
    }
      
    clauseToSuperClause = mergeSuperClauses(clauseToSuperClause);
    addSuperClauses(clauseToSuperClause);

    gndtime = Util::elapsed_seconds();

    cout << endl << endl;
    cout << "*****************************************************************"
         << endl << endl;
    cout << "Now, starting the iterations of creating supernodes/superfeatures"
         << endl;
      //now create the super preds corresponding to the current set of
      //superclauses

    SuperClause *superClause;
    int totalTupleCnt = 0;
    int totalGndTupleCnt = 0;

    cout << "Counts in the beginning:" << endl;
    for (int i = 0; i < superClausesArr_->size(); i++)
    {
      superClause = (*(*superClausesArr_)[i])[0];
         
      int cnt = superClause->getNumTuples();
      int gndCnt = superClause->getNumTuplesIncludingImplicit();

      totalTupleCnt += cnt;
      totalGndTupleCnt += gndCnt;
    }
    cout << "Total Number of Ground Tuples = " << totalGndTupleCnt << endl;
    cout << "Total Number of Tuples Created = " << totalTupleCnt << endl;

    /*************************************************************************
     * Start the Iterations now */
    /*************************************************************************/
       
    int newSuperClauseCnt = getNumArrayArrayElements(*superClausesArr_);
    int superClauseCnt = newSuperClauseCnt;
    int itr = 1;
    cout << "********************************************************"
         << endl << endl;
    setuptime = 0;

      // For creation of the Network
    Array<Factor *> * factors = new Array<Factor *>();
    Array<Node *> * nodes = new Array<Node *>();

    while (newSuperClauseCnt != superClauseCnt || itr <= 2)
    {
      superClauseCnt = newSuperClauseCnt;
      cout << "***************************************************************"
           << endl;
      cout << "Iteration: " << itr << endl;
           
        //for iteration 1, superclauses have already been created
      if (itr > 1)
      {
        cout << "Creating Super Clauses.. " << endl;
        createSuperClauses(superClausesArr_, domain);
        newSuperClauseCnt = getNumArrayArrayElements(*superClausesArr_);
      }
       
      cout << "Creating New Super Preds.. " << endl;
      createSuperPreds(superClausesArr_, domain);
       
      cout << "Number of superclauses after this iteration is = "
           << newSuperClauseCnt << endl;
      itr++;
    }

    superClauseCnt = getNumArrayArrayElements(*superClausesArr_);
    cout << "***************************************************************"
         << endl;
    cout << "Total Number of Super Clauses = " << superClauseCnt << endl;
  
    int predCnt = domain->getNumPredicates();
    const PredicateTemplate *ptemplate;
    for (int predId = 0; predId < predCnt; predId++)
    {
      ptemplate = domain->getPredicateTemplate(predId);
      if (ptemplate->isEqualPredicateTemplate())
        continue;
      int cnt = SuperPred::getSuperPredCount(predId);
      if (cnt > 0)
      {
        cout<<"SuperPred count for pred: ";
        ptemplate->print(cout);
        cout << " = " << cnt << endl;
      }
    }

    for (int i = 0; i < nodes->size(); i++) delete (*nodes)[i];
    for (int i = 0; i < factors->size(); i++) delete (*factors)[i];
    nodes->clear();
    factors->clear();
  }


  /**
   * Create the super network.
   */
  void createSuperNetwork()
  {
    Domain* domain = domain_;
    Array<SuperPred*> * superPreds;
    Array<SuperClause*> *superClauses;
  
    SuperClause *superClause;
    SuperPred *superPred;

    Factor *factor;
    Node *node;
    Clause *clause;
    Array<int>* constants = NULL;

      //create the factor (superclause) nodes
    for (int arrIndex = 0; arrIndex < superClausesArr_->size(); arrIndex++)
    {
      superClauses = (*superClausesArr_)[arrIndex];
      for (int scindex = 0; scindex < superClauses->size(); scindex++)
      {
        superClause = (*superClauses)[scindex];
        clause = superClause->getClause();
          // MS: Set to real weight, not 1
        //clause->setWt(superClause->getOutputWt());
        factor = new Factor(clause, superClause, constants, domain,
                              superClause->getOutputWt());
        factors_->append(factor);
      }
    }

      //create the variable (superpreds) nodes
    int predCnt = domain->getNumPredicates();
    for (int predId = 0;predId < predCnt; predId++)
    {
      superPreds = SuperPred::getSuperPreds(predId);
      for (int spindex = 0; spindex < superPreds->size(); spindex++)
      {
        superPred = (*superPreds)[spindex];
        node = new Node(predId, superPred, constants, domain);
        node->addFactors(factors_, getLinkIdToTwoWayMessageMap());
        nodes_->append(node);
      }
    }
  }
  
  /**
   *  Manipulate the link id to message map
   */
  void updateLinkIdToTwoWayMessageMap()
  {
    Node *node;
    Link *link;
    Factor *factor;
    double nodeToFactorMsgs[2];
    double factorToNodeMsgs[2];

    LinkId *lid;
    TwoWayMessage *tmsg;
    LinkIdToTwoWayMessageMap::iterator lidToTMsgItr;
     
      // Delete the old values
    Array<LinkId*> keysArr;
    for (lidToTMsgItr = lidToTWMsg_->begin();
         lidToTMsgItr != lidToTWMsg_->end();
         lidToTMsgItr++)
    {
      keysArr.append(lidToTMsgItr->first);
      tmsg = lidToTMsgItr->second;
      delete tmsg;
    }
                       
    for (int i = 0; i < keysArr.size(); i++)
    {
      delete keysArr[i];
    }
    lidToTWMsg_->clear();

      // Now populate
    for (int i = 0; i < nodes_->size(); i++)
    {
      node = (*nodes_)[i];
      for (int j = 0; j < node->getNumLinks(); j++)
      {
        link = node->getLink(j);
        factor = link->getFactor();
              
        int predId = node->getPredId();
        int superPredId = node->getSuperPredId();
        int superClauseId = factor->getSuperClauseId();
        int predIndex = link->getPredIndex(); 
              
        lid = new LinkId(predId, superPredId, superClauseId, predIndex);

        int reverseFactorIndex = link->getReverseFactorIndex();
        node->getMessage(reverseFactorIndex, nodeToFactorMsgs);

        int reverseNodeIndex = link->getReverseNodeIndex();
        factor->getMessage(reverseNodeIndex, factorToNodeMsgs);
 
        tmsg = new TwoWayMessage(nodeToFactorMsgs,factorToNodeMsgs);
        (*lidToTWMsg_)[lid] = tmsg;
      }
    }
  }
  
  /**
   * Add the super clauses.
   */
  void addSuperClauses(ClauseToSuperClauseMap* const & clauseToSuperClause)
  {
    ClauseToSuperClauseMap::iterator clauseItr;
    Array<SuperClause *> * superClauses;
    SuperClause *superClause;
     
    for(clauseItr = clauseToSuperClause->begin();
        clauseItr != clauseToSuperClause->end(); 
        clauseItr++)
    {
      superClauses = new Array<SuperClause *>();
      superClausesArr_->append(superClauses);
      superClause = clauseItr->second;
      superClauses->append(superClause);
    }
  }
  
  
  /**
   * Merge the super clauses.
   */
  ClauseToSuperClauseMap*
  mergeSuperClauses(ClauseToSuperClauseMap* const & clauseToSuperClause)
  {
    Domain* domain = domain_;
    ClauseToSuperClauseMap *mergedClauseToSuperClause =
      new ClauseToSuperClauseMap();
    SuperClause *superClause, *mergedSuperClause;
    Clause *keyClause;
    ClauseToSuperClauseMap::iterator itr, mergedItr;
    Array<int> * constants;
    double tcnt;
    for (itr = clauseToSuperClause->begin();
         itr != clauseToSuperClause->end();
         itr++)
    {
      superClause = itr->second;
      keyClause = superClause->getClause();
      mergedItr = mergedClauseToSuperClause->find(keyClause);
      if (mergedItr != mergedClauseToSuperClause->end())
      {
        mergedSuperClause = mergedItr->second;
        for (int tindex = 0; tindex < superClause->getNumTuples(); tindex++)
        {
          constants = superClause->getConstantTuple(tindex);
          tcnt = superClause->getTupleCount(tindex);
          mergedSuperClause->addNewConstantsAndIncrementCount(constants, tcnt);
          delete constants;
        }
        delete superClause;
      }
      else
      {
        (*mergedClauseToSuperClause)[keyClause] = superClause;
        keyClause->print(cout, domain);
        cout << endl;
      }
    }
    return mergedClauseToSuperClause;
  }
  
  /**
   * Creates the equivalence classes for the variables appearing in various
   * clauses.
   */
  PredicateTermToVariable* getPredicateTermToVariableMap(MLN * const & mln,
                                                         Domain* const & domain)
  {
    Clause *clause;
    Predicate *pred;
    const Term* term;
    Array<Variable *> *eqVars = new Array<Variable *>();
    int eqClassId = 0;
    Variable *var;
    const PredicateTemplate *ptemplate;
    const Array<int>* constants;

    for (int clauseno = 0; clauseno < mln->getNumClauses(); clauseno++)
    {
      clause = (Clause *)mln->getClause(clauseno);
      for (int predno = 0; predno < clause->getNumPredicates(); predno++)
      {
        pred = clause->getPredicate(predno);
        ptemplate = pred->getTemplate();
        for (int termno = 0; termno < pred->getNumTerms(); termno++)
        {
          term = pred->getTerm(termno);
          int varId = term->getId();
          int varTypeId = ptemplate->getTermTypeAsInt(termno);
          constants = domain->getConstantsByType(varTypeId); 
          var = new Variable(clause,varId,pred,termno,eqClassId,constants);
          eqVars->append(var);
          eqClassId++;
        }
      }
    }
    
    Variable  *var1, *var2;
    for (int i = 0; i < eqVars->size(); i++)
    {
      var1 = (*eqVars)[i];
      for (int j = i + 1; j < eqVars->size(); j++)
      {
        var2 = (*eqVars)[j];
        if (var1->same(var2))
        {
          var1->merge(var2); 
        }
      }
    }

      //now populate the map
    PredicateTermToVariable *ptermToVar = new PredicateTermToVariable();
    PredicateTermToVariable::iterator itr;
    PredicateTerm *pterm;
    Variable *tiedVar;
    int uniqueCnt = 0;
    for (int i = 0; i < eqVars->size(); i++)
    {
      var = (*eqVars)[i];
      if (var->isRepresentative())
      {
        uniqueCnt++;
        for (int j = 0; j < var->getNumTiedVariables(); j++)
        {
          tiedVar = var->getTiedVariable(j);
          int predId = tiedVar->getPredId();
          int termno = tiedVar->getTermno();
          pterm = new PredicateTerm(predId,termno);
          itr = ptermToVar->find(pterm);
          if (itr == ptermToVar->end())
          {
            (*ptermToVar)[pterm] = var;
          }
          else
          {
            delete pterm;
          }
        }
      }
    }

    cout << "size of PtermToVarMap is " << ptermToVar->size() << endl;
    cout << "count of Variable Eq Classes (Unique) is = " << uniqueCnt << endl;
    return ptermToVar;
  }


  void getIndexedConstants(PredicateTermToVariable * const & ptermToVar, 
                           MLN * const & mln, 
                           Domain * const & domain)
  {
    Predicate *pred, *gndPred;
    IntHashArray seenPredIds;
    const Clause *clause;
    const Term *term;
     
    Array<Predicate *> * indexedGndings;
    PredicateTerm *pterm;
    Database * db;
     
    int predId, termId, constantId;
    bool ignoreActivePreds = true;

    PredicateTermToVariable::iterator itr;
    Variable * var;

    indexedGndings = new Array<Predicate *>();
    db = domain->getDB();
    cout << "size of PtermToVarMap is " << ptermToVar->size() << endl;
     
    Clause *varClause;
    for (int clauseno = 0; clauseno < mln->getNumClauses(); clauseno++)
    {
      clause = mln->getClause(clauseno);    
      varClause = new Clause(*clause);
      varClause->updateToVarClause();

        //to make sure that we do not use clause
      clause = NULL;

      for (int predno = 0; predno < varClause->getNumPredicates(); predno++)
      {
        pred = varClause->getPredicate(predno);
        predId = pred->getId();

        if (seenPredIds.append(predId) < 0)
          continue;
        indexedGndings->clear();
          //Note: we assume that every predicate is indexable
        if(db->isClosedWorld(predId))
        {
            //precidate is closed world - rettrieve only true groundings
          db->getIndexedGndings(indexedGndings,pred,ignoreActivePreds,true);
        }
        else
        {
            //predicate is open world - retrieve both true and false groundings  
          db->getIndexedGndings(indexedGndings,pred,ignoreActivePreds,true);
          db->getIndexedGndings(indexedGndings,pred,ignoreActivePreds,false);
        }
        
        for (int gndno = 0; gndno < indexedGndings->size(); gndno++)
        {
          gndPred = (*indexedGndings)[gndno];

          for (int termno = 0; termno < gndPred->getNumTerms(); termno++)
          {
            pterm = new PredicateTerm(predId, termno);
            itr = ptermToVar->find(pterm);
            assert(itr != ptermToVar->end());
            var = itr->second;
            term = gndPred->getTerm(termno);
            constantId = term->getId();
            var->removeImplicit(constantId);
            delete pterm;
          }
          delete (*indexedGndings)[gndno];
        }
      }
      delete varClause;
    }
   
      //now explicitly handle the constants appearing in the clause
    for (int clauseno = 0; clauseno < mln->getNumClauses(); clauseno++)
    {
      clause = mln->getClause(clauseno);    
      for (int predno = 0; predno < clause->getNumPredicates(); predno++)
      {
        pred = clause->getPredicate(predno);
        predId = pred->getId();
        for (int termno = 0; termno < pred->getNumTerms(); termno++)
        {
          term = pred->getTerm(termno);
          termId = term->getId();
            // if it is a variable, nothing to do
          if (termId < 0) continue;
            // else, this constant also should be added the list of
            // indexed constants
          pterm = new PredicateTerm(predId, termno);
          itr = ptermToVar->find(pterm);
          assert(itr != ptermToVar->end());
          var = itr->second;
          var->removeImplicit(termId);
          delete pterm;
        }
      }
    }

    IntHashArray *seenEqClassIds = new IntHashArray();
    cout << "Implicit Set of constants are: " << endl;
    for (itr = ptermToVar->begin(); itr != ptermToVar->end(); itr++)
    {
      pterm = itr->first;
      var = itr->second;
      int eqClassId = var->getEqClassId();
      if (seenEqClassIds->find(eqClassId) >= 0)
        continue;
      seenEqClassIds->append(eqClassId);
      cout << "Implicit Constants for Eq class " << eqClassId << endl;
      cout << "Count =  " << var->getNumImplicitConstants() << " => " << endl;
      var->printImplicitConstants(cout, domain);
      cout << endl << endl << endl;
    }
    delete seenEqClassIds;
  }
  
 private:
    // Indicates if lifted inference will be run
  bool lifted_;
    // Indicates if implicit representation is to be used
  bool implicitRep_;

  LinkIdToTwoWayMessageMap* lidToTWMsg_;

  Array<Array<SuperClause*>*>* superClausesArr_;

    // Factors in the graph  
  Array<Factor*>* factors_;
    // Nodes in the graph
  Array<Node*>* nodes_;

    // MLN from which the factor graph is built
  MLN* mln_;
    // Domain containing the constants from which the factor graph is built
  Domain* domain_;
  
    // Stores auxiliary factors used for query formulas
  Array<AuxFactor*>* auxFactors_;
};

#endif /*FACTORGRAPH_H_*/
