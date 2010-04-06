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

/************************************
 * Near-Uniform Sampler
 * Adapted for MC-SAT
 * Original:
 	WalkSAT v45 by Kautz
	SampleSat by Wei et.al.
 ************************************/
#include "samplesat.h"
#include "timer.h"

char* heuristic_names[] = { "random", "best", "tabu", "sa"};

// ---------------------------------- //
// - Impl
// ---------------------------------- //
SampleSat::SampleSat(const SampleSatParams & params, int numGndPreds, int maxClause,
					 int maxLen, int maxLitOccurence, int* fixedAtoms, int** clauses,
                     const Array<Array<int> >* const & blocks,
                     const Array<bool >* const & blockEvidence)
{
  cout<<"Init Samplesat ..."<<endl;

	// -- samplesat param
  saRatio = params.saRatio;
  temperature = params.temperature;
  latesa = params.latesa;
  numsol = params.numsol;
  numerator = params.ws_noise;
  numrun = params.ws_restart;
  cutoff = base_cutoff = params.ws_cutoff;

	// -- WS data
  numatom = MAXATOM = numGndPreds;		 
  MAXCLAUSE = maxClause;
  MAXLENGTH = maxLen;
	//MAXLENGTH = 500;

  numoccurence = new int[2*MAXATOM+1];
  atom = new int[2*MAXATOM+1];
  lowatom = new int[2*MAXATOM+1];
  solution = new int[2*MAXATOM+1];
  changed = new int[2*MAXATOM+1];
  breakcount = new int[2*MAXATOM+1];
  makecount = new int[2*MAXATOM+1];

	// for unit propagation
	//fixedatom = new int[MAXATOM+1];
  fixedatom = fixedAtoms;
  isSat = new bool[MAXCLAUSE];

  blocks_ = new Array<Array<int> >(*blocks);
  blockEvidence_ = new Array<bool>(*blockEvidence);
  
    // Init to not in block
  inBlock = false;
  flipInBlock = NOVALUE;
  clause = clauses;

  size = new int[MAXCLAUSE];
  wfalse = new int[MAXCLAUSE];
  lowfalse = new int[MAXCLAUSE];
  wherefalse = new int[MAXCLAUSE];
  numtruelit = new int[MAXCLAUSE];

	// -- for new flip
  watch1 = new int[MAXCLAUSE];
  watch2 = new int[MAXCLAUSE];

  occurence = new int*[2*MAXATOM+1];
  for (int i = 0; i <= 2*MAXATOM; i++)
  {
	occurence[i] = new int[maxLitOccurence];
  }

  int idx = 0;
  pickcode[idx++] = &SampleSat::pickrandom;
  pickcode[idx++] = &SampleSat::pickbest;
  pickcode[idx++] = &SampleSat::picktabu;
  pickcode[idx++] = &SampleSat::picksa;

	// from initVars()
  denominator = 100;
  heuristic = SA;		/* heuristic to be used */
  tabu_length = 5;		/* length of tabu list */
  wp_numerator = NOVALUE;	/* walk probability numerator/denominator */
  wp_denominator = 100;		
  superlinear = true; 
    /* SA - set to true by heuristics that require the make values to be calculated */
  makeflag = true;

	/* Histogram of tail */
  tail = 3;

	/* Printing options */
  printonlysol = false;
  printsolcnf = false;
  printfalse = false;
  printlow = false;
  printhist = false;
  printtrace = 0;
  //printtrace = 1000;
  trace_assign = false;

  outfile[0] = 0;

	/* Initialization options */
  initfile[0] = 0;
  initoptions = false;
}

SampleSat::~SampleSat()
{
  for (int i = 0; i <= 2*MAXATOM; i++)
  {
	delete [] occurence[i];
  }
  delete [] occurence;
  delete [] numoccurence;
  delete [] atom;
  delete [] lowatom;
  delete [] solution;
  delete [] changed;
  delete [] breakcount;
  delete [] makecount;
	
  delete [] size;
  delete [] wfalse;
  delete [] lowfalse;
  delete [] wherefalse;
  delete [] numtruelit;

  delete [] watch1;
  delete [] watch2;
  
  for (int i = 0; i < blocks_->size(); i++)
    (*blocks_)[i].clearAndCompress();
  delete blocks_;
    
  delete blockEvidence_;
}

// Samplesat algorithm
bool SampleSat::sample(bool*& assignments, int numClauses,
                       const Array<Array<int> >* const & blocks,
                       const Array<bool>* const & blockEvidence)
{
    // init random: this is independent of the randomizer used in infer
  gettimeofday(&tv,&tzp);
  seed = (unsigned int)((( tv.tv_sec & 0177 ) * 1000000) + tv.tv_usec);
  srandom(seed);

  for (int i = 0; i < blocks_->size(); i++)
    (*blocks_)[i].clearAndCompress();
  delete blocks_;
  delete blockEvidence_;

  blocks_ = new Array<Array<int> >(*blocks);
  blockEvidence_ = new Array<bool>(*blockEvidence);
    
	//cout<<"In SampleSat.sample ..."<<endl;
  numclause = numClauses;

	// -- if no curr sat, randomly flip
  if (numclause == 0)
  {
      // For each block: select one to set to true
    for (int i = 0; i < blocks_->size(); i++)
    {
      Array<int> block = (*blocks_)[i];
      
        // True fixed atom in the block: do nothing
      if (fixedAtomInBlock(i))
        continue;

        // If evidence atom exists, then all others are false
      if ((*blockEvidence_)[i])
      {
          // If 2nd argument is -1, then all are set to false
        setOthersInBlockToFalse(assignments, -1, i);
        continue;
      }
      
      int chosen = random() % block.size();
      assignments[block[chosen]] = true;
      setOthersInBlockToFalse(assignments, chosen, i);
    }
      
      // Random tv for all not in blocks
	for (int i = 1; i <= MAXATOM; i++)
	{
  	  if (fixedatom[i] >= 0) assignments[i-1] = (fixedatom[i] == 1);
      else if (getBlock(i-1) >= 0) continue;
	  else assignments[i-1] = (random() < RAND_MAX/2);
	}
	return true;
  }

  initVars();		// initialize variables	
    
  Timer timer;
  double begSec = timer.time();
  
  if (!initprob())
  {
	return false;
  }
  //cout << "time taken for UP = "; Timer::printTime(cout, timer.time()-begSec);
  //cout << endl;

//cout << "Numatom after UP: " << numatom << endl;
//cout << "Numclause after UP: " << numclause << endl;
    // -- if no curr sat, randomly flip
  if (numclause == 0)
  {
      // For each block: select one to set to true
    for (int i = 0; i < blocks_->size(); i++)
    {
      Array<int> block = (*blocks_)[i];
        // Fixed atoms have been removed from the blocks
        // If evidence atom exists, then all others are false
      if ((*blockEvidence_)[i])
      {
        for (int j = 0; j < block.size(); j++)
        {
          int newIdx = block[j];
          int oldIdx = newToOldMapping_[newIdx];
          assignments[oldIdx - 1] = false;
        }
        continue;
      }
      
      int chosen = random() % block.size();
      int newIdx = block[chosen];
      int oldIdx = newToOldMapping_[newIdx];
      assignments[oldIdx - 1] = true;
      for (int j = 0; j < block.size(); j++)
      {
        if (j == chosen) continue;
        int newOtherIdx = block[j];
        int oldOtherIdx = newToOldMapping_[newOtherIdx];
        assignments[oldOtherIdx - 1] = false;
      }
    }
      
      // Random tv for all not in blocks
    for (int i = 1; i <= MAXATOM; i++)
    {
      if (fixedatom[i] >= 0) assignments[i-1] = (fixedatom[i] == 1);
      else if (getBlock(-(fixedatom[i] + 1)) >= 0) continue;
      else assignments[i-1] = (random() < RAND_MAX/2);
    }

    return true;
  }

  initRandom();	// random restart each time

  begSec = timer.time(); 
  
  while (numsuccesstry < numsol && numtry < numrun*numsol)
  {
	numtry++;
	  //update_statistics_start_try();
	numflip = 0;	
	if (superlinear) cutoff = base_cutoff * super(numtry);	

	while(numflip < cutoff && numsuccesstry < numsol)
	{
	  print_statistics_start_flip();
	  numflip++;
	  flipatom((this->*(pickcode[heuristic]))());
        // If in a block, then another atom was also chosen to flip
      if (inBlock)
      {
        flipatom(flipInBlock);
      }
        // set in block back to false
      inBlock = false;
      flipInBlock = NOVALUE;
      
		//update_statistics_end_flip();
	  if (numfalse==0) update_and_print_statistics_end_try();
	}
		//update_and_print_statistics_end_try();

	  // If unsat random restart
    if (numsuccesstry <= 0) initRandom();
	  // if (numfalse > target) initRandom();

  }

  //cout << "time taken for flipping = "; Timer::printTime(cout, timer.time()-begSec);
  //cout << endl;
  
  if (numsuccesstry > 0)
  {
	for (int i = 1; i <= MAXATOM; i++)
	{
	  if (fixedatom[i] >= 0) assignments[i-1] = (fixedatom[i] == 1);
	  else assignments[i-1] = (solution[-fixedatom[i]] == 1);
	}
	return true;
  }
  else return false;
}

// Init w. Unit propagation
bool SampleSat::initprob()
{
  //cout << "=== initProb w. unit prop: query clauses=" << numclause << endl;
  int i;
  int lit;

  if(numclause > MAXCLAUSE)
  {
	fprintf(stderr,"ERROR - too many clauses\n"); 
	exit(-1);                              
  }

  bool done = false;
  int numfixedatom = 0, numsat = 0;
  for (i = 0; i < numclause; i++)
  {
	isSat[i] = false;
  }

  while (!done)
  {
	done = true;
    for (i = 0; i < numclause; i++)
    {
      if (!isSat[i])
      {
		int nonUnitNum = 0;
		int nonUnitIdx = -1;
		int j = 0;
		while (clause[i][j] != 0)
		{
		  int lit = clause[i][j];
		  int var = (lit > 0)? lit : -lit;
		  if (fixedatom[var] == -1)
		  { // Not fixed
			nonUnitNum++;
			nonUnitIdx = j;
		  }
		  else
          if ((fixedatom[var] == 0 && lit < 0) ||
              (fixedatom[var] == 1 && lit > 0))
		  { // Fixed and satisfied
			isSat[i] = true;
			numsat++;
			break;
		  }
		  j++;
		}
		
          // Exactly one non-fixed atom in clause and not satisfied
        if (!isSat[i] && nonUnitNum==1)
		{
		  int lit = clause[i][nonUnitIdx];
		  int var = (lit > 0)? lit : -lit;
		  fixedatom[var] = (lit > 0)? 1 : 0;
		  isSat[i] = true;
		  done = false;
		  numfixedatom++;
          numsat++;
		}
	  }
    }
  }
  //cout << "\tnumunit=" << numunit <<endl;
  //cout << "\tafter unit propagation: numfixedatom="<<numfixedatom<<" : numsat="<<numsat<<endl;

	// delegate to subproblem w backbone removed
  numliterals = 0;
  for(i = 0; i < 2*MAXATOM + 1; i++) numoccurence[i] = 0;

  int nc = numclause;
  numatom = 0, numclause = 0;
  newToOldMapping_.clearAndCompress();
  for (i = 1; i <= MAXATOM; i++) 
  {
    // mapping i => -k_i
    if (fixedatom[i] < 0)
    {
      fixedatom[i] = -(++numatom);
      newToOldMapping_.append(i);
    }
  }
  
    // Take new mapping into account in the blocks
  for (i = 0; i < blocks_->size(); i++)
  {
    int fixedTrueAtoms = 0;
    Array<int>& block = (*blocks_)[i];
    for (int j = 0; j < block.size(); j++)
    {
      int num = fixedatom[block[j] + 1];
        // Atom is fixed to true
      if (num == 1)
      {
        if (fixedTrueAtoms > 0)
        {
          cout << "Inconsistent fixed assignment! Two atoms in one block "
               << "are fixed to true through unit propagation." << endl;
          exit(-1);
        }
        if ((*blockEvidence_)[i])
        {
          cout << "Inconsistent fixed assignment! An atom in a block "
               << "is fixed to true which contradicts evidence." << endl;
          exit(-1);
        }
        fixedTrueAtoms++;
        block.removeItem(j--);
      }
        // Atom is fixed to false
      else if (num == 0)
      {
        block.removeItem(j--);
      }
        // Atom is not fixed
      else
      {
        assert (num < 0);
        block[j] = -(num + 1);
      }
    }

      // If an atom is fixed to true in the block,
      // then treat as evidence and set others to false
    if (fixedTrueAtoms > 0 || (*blockEvidence_)[i])
    {
      (*blockEvidence_)[i] = true;
      for (int j = 0; j < block.size(); j++)
      {
        atom[block[j] + 1] = false;
      }
    }    
      // If block has been emptied, then remove it
    else if (block.size() == 0)
    {
      blocks_->removeItem(i);
      blockEvidence_->removeItem(i);
      i--;
    }
      // Otherwise block with one true and others false
    else
    {
      int chosen = random() % block.size();
      atom[block[chosen] + 1] = true;
      for (int j = 0; j < block.size(); j++)
      {
        if (j != chosen)
        {
          atom[block[j] + 1] = false;
        }
      }
    }
  }

  for (i = 0; i < nc; i++)
  {
    if (!isSat[i])
    {
	  int idx = 0;
	  int k = 0;
	  while (clause[i][k] != 0)
	  {
		lit = clause[i][k++];
		int var = (lit > 0) ? lit: -lit;
			
		assert(fixedatom[var] != (lit > 0));
          // if fixed but unsat, must be false, ignore
		if (fixedatom[var] >= 0) continue;
          // fixedatom[var] is negative
		int newlit = (lit < 0) ? fixedatom[var] : -fixedatom[var];	// mapped lit
		assert(numclause <= i && idx < k);
		clause[numclause][idx++] = newlit;
		numliterals++;
		occurence[newlit + MAXATOM][numoccurence[newlit + MAXATOM]++] = numclause;
	  }
		//assert(idx>0);
	  if (idx == 0)
	  {
		cout << "Inconsistent fixed assignment!" << endl;
		return false;
	  }
	  size[numclause] = idx;
	  numclause++;
	}
  }
  return true;
}

void SampleSat::initRandom()
{
  int i;
  int j;
  int thetruelit = -1;
  int truelit1;
  int truelit2;

  for(i = 0; i < numclause; i++) numtruelit[i] = 0;
  numfalse = 0;

  for(i = 1; i < numatom + 1; i++)
  {
    changed[i] = -BIG;
	breakcount[i] = 0;
	makecount[i] = 0;
  }

	// UP: Unit propagation
	//for(i = 1;i < numatom+1;i++) atom[i] = (fixedatom[i]!=-1) ? fixedatom[i] : (random()%2);

	// BB: the backbone is removed
    // For each block: select one to set to true
  for (int i = 0; i < blocks_->size(); i++)
  {
    Array<int> block = (*blocks_)[i];
      // If evidence atom exists, then all others are false
    if ((*blockEvidence_)[i])
    {
        // If 1st argument is -1, then all are set to false
      setOthersInBlockToFalse(-1, i);
      continue;
    }
    
    int chosen = random() % block.size();
    atom[block[chosen] + 1] = true;
    setOthersInBlockToFalse(chosen, i);
  }
      
    // Random tv for all not in blocks
  for (int i = 1; i <= numatom; i++)
  {
    if (getBlock(i - 1) >= 0) continue;
    else atom[i] = (random() < RAND_MAX/2) ? 1 : 0;
  }

	// Initialize breakcount and makecount in the following:
  for(i = 0; i < numclause; i++)
  {
    truelit1 = 0;
    truelit2 = 0;
	for(j = 0; j < size[i]; j++)
	{
	  if((clause[i][j] > 0) == atom[abs(clause[i][j])])
	  { // ij is true lit
		numtruelit[i]++;
		thetruelit = clause[i][j];

        if (!truelit1) truelit1 = clause[i][j];
        else if (truelit1 && !truelit2) truelit2 = clause[i][j];
	  }
	}
	
    if(numtruelit[i] == 0)
	{
	  wherefalse[i] = numfalse;
	  wfalse[numfalse] = i;
	  numfalse++;
	  for(j = 0; j < size[i]; j++) makecount[abs(clause[i][j])]++;
	}
	else
    if (numtruelit[i] == 1)
	{
	  breakcount[abs(thetruelit)]++;
      watch1[i] = abs(thetruelit);
	}
	else
	{ /*if (numtruelit[i] == 2)*/
	  watch1[i] = abs(truelit1);
	  watch2[i] = abs(truelit2);
	}
  }

  if (hamming_flag)
  {
	hamming_distance = calc_hamming_dist(atom, hamming_target, numatom);
	fprintf(hamming_fp, "0 %i\n", hamming_distance);
  }
}

void SampleSat::initVars()
{
  status_flag = 0; /* value returned from main procedure */

  target = 0;
  numtry = 0; /* total attempts at solutions */
  numsuccesstry = 0; /* total found solutions */

	// Hamming calcualations
  hamming_target_file[0] = 0;
  hamming_data_file[0] = 0;
  hamming_flag = false;
}

void SampleSat::print_parameters(int argc, char * argv[])
{
  printf("cutoff = %li\n",cutoff);
  printf("tries = %i\n",numrun);

  printf("heuristic = ");
  switch(heuristic)
  {
	case TABU:
	  printf("tabu %d", tabu_length);
	  break;
	default:
	  printf("%s", heuristic_names[heuristic]);
	  break;
  }
  if (numerator > 0)
  {
	printf(", noise %d / %d", numerator, denominator);
  }
  if (wp_numerator > 0)
  {
	printf(", wp %d / %d", wp_numerator, wp_denominator);
  }
  printf("\n");
}

int SampleSat::picksa(void)
{
  int change;
  int toflip;

  if (numfalse == 0 || (random() % 100 < saRatio && !latesa))
  {
	toflip = random()%numatom + 1;

    int blockIdx = getBlock(toflip - 1);
    if (blockIdx == -1)
      change = makecount[toflip] - breakcount[toflip];
    else // Atom is in a block
    {  
        // If evidence atom exists or in block of size 1, then can not flip
      if ((*blockEvidence_)[blockIdx] || (*blocks_)[blockIdx].size() == 1)
        return NOVALUE;
      change = calculateChange(toflip);
    }
    
	if (change >= 0)
	  return toflip;
	else if (random() <= exp(change/(temperature/100.0)) * RAND_MAX)
	  return toflip;
	else
    {
        // set back flipInBlock
      inBlock = false;
      flipInBlock = NOVALUE;
      return NOVALUE;
    }
  }
  else
  {
	//return picktabu();
    return pickbest();
  }
}

void SampleSat::update_and_print_statistics_end_try(void)
{
  if(numfalse <= target)
  {
	status_flag = 0;
	save_solution();
	numsuccesstry++;
  }
}

