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
#ifndef SAMPLESAT_H_OCT_23_2005
#define SAMPLESAT_H_OCT_23_2005

/************************************
 * Near-Uniform Sampler
 * Adapted for MC-SAT
 * Original:
    WalkSAT v45 by Kautz
    SampleSat by Wei et.al.
 ************************************/
#define VERSION "samplesat-WSv45"
#define UNIX 1

// ---------------------------------- //
// - Includes
// ---------------------------------- //
#include "samplesatparams.h"
#include "array.h"
#include "hashint.h"
#include <assert.h>
#include <iostream>
using namespace std;

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <limits.h>
#include <signal.h>

// -- UNIX
#include <sys/times.h>
#include <sys/time.h>

// ---------------------------------- //
// - Constants
// ---------------------------------- //
#ifdef DYNAMIC
  #define STOREBLOCK 2000000	/* size of block to malloc each time */
#else
  #define STOREBLOCK 2000000	/* size of block to malloc each time */
#endif
 
#define BIGINT long int
#ifndef CLK_TCK
  #define CLK_TCK 60
#endif
#define NOVALUE -1
#define INIT_PARTIAL 1
//#define HISTMAX 101		/* length of histogram of tail */
#define HISTMAX 1		/* length of histogram of tail */
#define BIG 100000000

enum heuristics {RANDOM, BEST, TABU, SA};

// ---------------------------------- //
// - Class
// ---------------------------------- //
class SampleSat
{
public:
  SampleSat(const SampleSatParams & params, int numGndPreds, int maxClause,
            int maxLen, int maxLitOccurence, int* fixedAtoms, int** clauses,
            const Array<Array<int> >* const & blocks,
            const Array<bool>* const & blockEvidence);
  ~SampleSat();
  bool sample(bool*& assignments, int numClauses,
              const Array<Array<int> >* const & blocks,
              const Array<bool>* const & blockEvidence);

private:
	// ---------------------------------- //
	// - variables
	// ---------------------------------- //
  int saRatio;       // saRatio/100: percentage of SA steps
  int temperature;   // temperature/100: fixed temperature for SA step
  bool latesa;

	// - WS data
	// = {pickrandom, pickbest, picktabu, picknovelty, pickrnovelty, 
	//    picknoveltyplus, pickrnoveltyplus, picksa};
  int (SampleSat::*pickcode[15])(void); 

  int numatom, MAXATOM;
  int numclause, MAXCLAUSE;
  int numliterals;
  int MAXLENGTH;		/* maximum number of literals which can be in any clause */

  int ** clause;		/* clauses to be satisfied */
    					/* indexed as clause[clause_num][literal_num] */
  int * size;			/* length of each clause */
  int * wfalse;		/* clauses which are false */
  int * lowfalse;
  int * wherefalse;	/* where each clause is listed in false */
  int * numtruelit;	/* number of true literals in each clause */

  int** occurence;
  int* numoccurence;
  int* atom;	
  int* lowatom;
  int* solution;
  int* changed;
  int* breakcount;
  int* makecount;
  int numfalse;

	// For unit propagation
  int* fixedatom;
  bool* isSat;

    // For block inference
  Array<Array<int> >* blocks_;
  Array<bool >* blockEvidence_;
    
    // Which other atom to flip in block
  bool inBlock;
  int flipInBlock;
    
    // Keeps track of old indices after unit prop.
  Array<int> newToOldMapping_;
    
  int* watch1;
  int* watch2;	

	// params
  int status_flag;   /* value returned from main procedure */
  int abort_flag;

  int heuristic;     /* heuristic to be used */
  int numerator;     /* make random flip with numerator/denominator frequency */
  int denominator;		
  int tabu_length;   /* length of tabu list */

  int wp_numerator;  /* walk probability numerator/denominator */
  int wp_denominator;		

  BIGINT numflip;    /* number of changes so far */
  BIGINT numnullflip;/*  number of times a clause was picked, but no  */
					   /*  variable from it was flipped  */
  BIGINT cutoff;
  BIGINT base_cutoff;
  int target;
  int numtry;        /* total attempts at solutions */

  int numrun;
  int numsol;

  bool superlinear;

    // set to true by heuristics that require the make values to be calculated
  bool makeflag;
	
    /* Histogram of tail */
  long int tailhist[HISTMAX];	/* histogram of num unsat in tail of run */
  long histtotal;
  int tail;
  int tail_start_flip;

	/* Printing options */
  bool printonlysol;
  bool printsolcnf;
  bool printfalse;
  bool printlow;
  bool printhist;
  int printtrace;
  bool trace_assign;

  char outfile[1024];

	/* Initialization options */
  char initfile[100];
  int initoptions;

	/* Randomization */
  int seed;

 #ifdef UNIX
  struct timeval tv;
  struct timezone tzp;
 #endif
 #ifdef NT
  DWORD win_time;     // elapsed time in ms, since windows boot up
 #endif

	/* Statistics */
	double expertime;
	BIGINT flips_this_solution;
	long int lowbad;           /* lowest number of bad clauses during try */
	BIGINT totalflip;          /* total number of flips in all tries so far */
	BIGINT totalsuccessflip;   /* total number of flips in all tries which succeeded so far */
	int numsuccesstry;         /* total found solutions */

	BIGINT x;
	BIGINT integer_sum_x;
	double sum_x;
	double sum_x_squared;
	double mean_x;
	double second_moment_x;
	double variance_x;
	double std_dev_x;
	double std_error_mean_x;
	double seconds_per_flip;
	int r;
	int sum_r;
	double sum_r_squared;
	double mean_r;
	double variance_r;
	double std_dev_r;
	double std_error_mean_r;

	double avgfalse;
	double sumfalse;
	double sumfalse_squared;
	double second_moment_avgfalse, variance_avgfalse, std_dev_avgfalse, ratio_avgfalse;
	//double std_dev_avgfalse;
	double f;
	double sample_size;

	double sum_avgfalse;
	double sum_std_dev_avgfalse;
	double mean_avgfalse;
	double mean_std_dev_avgfalse;
	int number_sampled_runs;
	double ratio_mean_avgfalse;

	double suc_sum_avgfalse;
	double suc_sum_std_dev_avgfalse;
	double suc_mean_avgfalse;
	double suc_mean_std_dev_avgfalse;
	int suc_number_sampled_runs;
	double suc_ratio_mean_avgfalse;

	double nonsuc_sum_avgfalse;
	double nonsuc_sum_std_dev_avgfalse;
	double nonsuc_mean_avgfalse;
	double nonsuc_mean_std_dev_avgfalse;
	int nonsuc_number_sampled_runs;
	double nonsuc_ratio_mean_avgfalse;

	/* Hamming calcualations */
	char hamming_target_file[512];
	char hamming_data_file[512];
	int hamming_sample_freq;
	int hamming_flag;
	int hamming_distance;
	int* hamming_target;
	FILE * hamming_fp;

	/* Noise level */
	int samplefreq;

	// ---------------------------------- //
	// - utility
	// ---------------------------------- //
	void initRandom();
	void initVars();
	void print_parameters(int argc, char * argv[]);

	int Var(int c, int p) {return abs(clause[c][p]);}
	
	void parse_parameters(int argc,char *argv[])
	{
		int i;
		int temp;

		// -- set SA param
		heuristic = SA;

		// -- parse params
		for (i=1;i < argc;i++)
		{
		if (strcmp(argv[i],"-seed") == 0)
			scanone(argc,argv,++i,&seed);
		else if (strcmp(argv[i],"-out") == 0 && i<argc-1)
			strcpy(outfile, argv[++i]);
		else if (strcmp(argv[i],"-hist") == 0)
			printhist = true;
		else if (strcmp(argv[i],"-status") == 0)
			status_flag = 1;
		else if (strcmp(argv[i],"-cutoff") == 0)
			scanonell(argc,argv,++i,&cutoff);
		else if (strcmp(argv[i],"-late_sa")==0)
			latesa = true;
		else if (strcmp(argv[i],"-random") == 0)
			heuristic = RANDOM;
		else if (strcmp(argv[i],"-best") == 0)
			heuristic = BEST;
		else if (strcmp(argv[i],"-sa") == 0){
			heuristic = SA;
			makeflag = true;
			scanone(argc, argv, ++i, &saRatio);
			scanone(argc, argv, ++i, &temperature);
		}
		else if (strcmp(argv[i],"-noise") == 0){
			scanone(argc,argv,++i,&numerator);
			if (i < argc-1 && sscanf(argv[i+1],"%i",&temp)==1){
			denominator = temp;
			i++;
			}
		}
			else if (strcmp(argv[i],"-wp") == 0){
				scanone(argc,argv,++i,&wp_numerator);
				if (i < argc-1 && sscanf(argv[i+1],"%i",&temp)==1){
					wp_denominator = temp;
					i++;
				}
			}
		else if (strcmp(argv[i],"-init") == 0  && i < argc-1)
			sscanf(argv[++i], " %s", initfile);
		else if (strcmp(argv[i],"-hamming") == 0  && i < argc-3){
			sscanf(argv[++i], " %s", hamming_target_file);
			sscanf(argv[++i], " %s", hamming_data_file);
			sscanf(argv[++i], " %i", &hamming_sample_freq);
			hamming_flag = true;
			numrun = 1;
		}
		else if (strcmp(argv[i],"-partial") == 0)
			initoptions = INIT_PARTIAL;
		else if (strcmp(argv[i],"-super") == 0)
			superlinear = true;
		else if (strcmp(argv[i],"-tries") == 0 || strcmp(argv[i],"-restart") == 0)
			scanone(argc,argv,++i,&numrun);
		else if (strcmp(argv[i],"-target") == 0)
			scanone(argc,argv,++i,&target);
		else if (strcmp(argv[i],"-tail") == 0)
			scanone(argc,argv,++i,&tail);
		else if (strcmp(argv[i],"-sample") == 0)
			scanone(argc,argv,++i,&samplefreq);
		else if (strcmp(argv[i],"-tabu") == 0)
		{
			scanone(argc,argv,++i,&tabu_length);
			heuristic = TABU;
		}
		else if (strcmp(argv[i],"-low") == 0)
			printlow = true;
		else if (strcmp(argv[i],"-sol") == 0)
		{
			printonlysol = true;
			printlow = true;
		}
		else if (strcmp(argv[i],"-solcnf") == 0)
		{
			printsolcnf = true;
			if (numsol == NOVALUE) numsol = 1;
		}
		else if (strcmp(argv[i],"-bad") == 0)
			printfalse = true;
		else if (strcmp(argv[i],"-numsol") == 0)
			scanone(argc,argv,++i,&numsol);
		else if (strcmp(argv[i],"-trace") == 0)
			scanone(argc,argv,++i,&printtrace);
		else if (strcmp(argv[i],"-assign") == 0){
			scanone(argc,argv,++i,&printtrace);
			trace_assign = true;
		}
		else 
		{
			fprintf(stderr, "General parameters:\n");
			fprintf(stderr, "  -seed N -cutoff N -restart N\n");
			fprintf(stderr, "  -numsol N = stop after finding N solutions\n");
			fprintf(stderr, "  -super = use the Luby series for the cutoff values\n");
			fprintf(stderr, "  -init FILE = set vars not included in FILE to false\n");
			fprintf(stderr, "  -partial FILE = set vars not included in FILE randomly\n");
			fprintf(stderr, "  -status = return fail status if solution not found\n");
			fprintf(stderr, "  -target N = succeed if N or fewer clauses unsatisfied\n");
			fprintf(stderr, "Heuristics:\n");
			fprintf(stderr, "  -random -best -tabu N \n");
			fprintf(stderr, "  -sa SA_RATIO TEMPERATURE walksat with SA_RATIO percent SA\n               moves added at temp TEMPERAURE/100. (default: -sa 50 10)\n");
			fprintf(stderr, "  -late_sa  use mixed simulated annealing moves only at solution cluster\n");
			fprintf(stderr, "  -noise N or -noise N M (default M = 100)\n");
			fprintf(stderr, "Printing:\n");
			fprintf(stderr, "  -out FILE = print solution as a list of literals to FILE\n");
			fprintf(stderr, "  -trace N = print statistics every N flips\n");
			fprintf(stderr, "  -assign N = print assignments at flip N, 2N, ...\n");
			fprintf(stderr, "  -sol = print satisfying assignments to stdout\n");
			fprintf(stderr, "  -solcnf = print sat assign to stdout in DIMACS format, and exit\n");
			fprintf(stderr, "  -low = print lowest assignment each try\n");
			fprintf(stderr, "  -bad = print unsat clauses each try\n");
			fprintf(stderr, "  -hist = print histogram of tail\n");
			fprintf(stderr, "  -tail N = assume tail begins at nvars*N\n");
			fprintf(stderr, "  -sample N = sample noise level every N flips\n");
			fprintf(stderr, "  -hamming TARGET_FILE DATA_FILE SAMPLE_FREQUENCY\n");
			exit(-1);
		}
		}
		base_cutoff = cutoff;
		if (numsol==NOVALUE || numsol>numrun) numsol = numrun;
		if (numerator==NOVALUE){
		switch(heuristic) {
		case BEST:
		default:
			numerator = 0;
			break;
		}
		}
		if (wp_numerator==NOVALUE){
			switch(heuristic) {
			default:
			wp_numerator = 0;
			break;
			}
		}
	}



	void print_statistics_header(void)
	{
		/*
		printf("numatom = %i, numclause = %i, numliterals = %i\n",numatom,numclause,numliterals);
		printf("wff read in\n\n");
		printf("    lowest     final       avg     noise     noise     total                 avg        mean        mean\n");
		printf("    #unsat    #unsat     noise   std dev     ratio     flips              length       flips       flips\n");
		printf("      this      this      this      this      this      this   success   success       until         std\n");
		printf("       try       try       try       try       try       try      rate     tries      assign         dev\n\n");

		fflush(stdout);
		*/
	}

	void initialize_statistics(void)
	{
		x = 0; r = 0;
		if (hamming_flag) {
		read_hamming_file(hamming_target_file);
		open_hamming_data(hamming_data_file);
		}
		tail_start_flip = tail * numatom;
		//printf("tail starts after flip = %i\n", tail_start_flip);
		numnullflip = 0;
	}

	void update_statistics_start_try(void)
	{
		int i;

		lowbad = numfalse;

		sample_size = 0;
		sumfalse = 0.0;
		sumfalse_squared = 0.0;

		for (i=0; i<HISTMAX; i++)
		tailhist[i] = 0;
		if (tail_start_flip == 0){
		tailhist[numfalse < HISTMAX ? numfalse : HISTMAX - 1] ++;
		}
		      
		if (printfalse) save_false_clauses(lowbad);
		if (printlow) save_low_assign();
	}

	void update_and_print_statistics_end_try(void);

	void print_statistics_start_flip(void) 
	{	
	if (printtrace && (numflip % printtrace == 0)){
		printf(" %9li %9i                     %9li\n", lowbad,numfalse,numflip);
		if (trace_assign)
			print_current_assign();
		fflush(stdout);
	}	
	}

	void update_statistics_end_flip(void)
	{
		if (numfalse < lowbad){
		lowbad = numfalse;	
		if (printfalse) save_false_clauses(lowbad);
		if (printlow) save_low_assign();
		}
		if (numflip >= tail_start_flip){
		tailhist[(numfalse < HISTMAX) ? numfalse : (HISTMAX - 1)] ++;
		if ((numflip % samplefreq) == 0){
		sumfalse += numfalse;
		sumfalse_squared += numfalse * numfalse;
		sample_size ++;
		}
		}
	}

	void print_statistics_final(void)
	{
		if (numsuccesstry > 0)
		{
		  printf("ASSIGNMENT FOUND\n");
		  if (printsolcnf) print_sol_cnf();
		  if (outfile[0]) print_sol_file(outfile);
		}
		else
		  printf("ASSIGNMENT NOT FOUND\n");
	}


	static long super(int i)
	{
		long power;
		int k;

		if (i<=0){
		fprintf(stderr, "bad argument super(%d)\n", i);
		exit(1);
		}
		/* let 2^k be the least power of 2 >= (i+1) */
		k = 1;
		power = 2;
		while (power < (i+1)){
		k += 1;
		power *= 2;
		}
		if (power == (i+1)) return (power/2);
		return (super(i - (power/2) + 1));
	}

	void handle_interrupt(int sig)
	{
		if (abort_flag) exit(-1);
		abort_flag = true;
	}

	void scanone(int argc, char *argv[], int i, int *varptr)
	{
		if (i>=argc || sscanf(argv[i],"%i",varptr)!=1){
		fprintf(stderr, "Bad argument %s\n", i<argc ? argv[i] : argv[argc-1]);
		exit(-1);
		}
	}

	void scanoneu(int argc, char *argv[], int i, unsigned int *varptr)
	{
		if (i>=argc || sscanf(argv[i],"%u",varptr)!=1){
		fprintf(stderr, "Bad argument %s\n", i<argc ? argv[i] : argv[argc-1]);
		exit(-1);
		}
	}

	void scanonell(int argc, char *argv[], int i, BIGINT *varptr)
	{
		if (i>=argc || sscanf(argv[i],"%li",varptr)!=1){
		fprintf(stderr, "Bad argument %s\n", i<argc ? argv[i] : argv[argc-1]);
		exit(-1);
		}
	}


	int calc_hamming_dist(int atom[], int hamming_target[], int numatom)
	{
		int i;
		int dist = 0;
	    
		for (i=1; i<=numatom; i++){
		if (atom[i] != hamming_target[i]) dist++;
		}
		return dist;
	}

	void open_hamming_data(char initfile[])
	{
		if ((hamming_fp = fopen(initfile, "w")) == NULL){
		fprintf(stderr, "Cannot open %s for output\n", initfile);
		exit(1);
		}
	}


	void read_hamming_file(char initfile[])
	{
		int i;			/* loop counter */
		FILE * infile;
		int lit;    

		printf("loading hamming target file %s ...", initfile);

		if ((infile = fopen(initfile, "r")) == NULL){
		fprintf(stderr, "Cannot open %s\n", initfile);
		exit(1);
		}
		i=0;
		for(i = 1;i < numatom+1;i++)
		hamming_target[i] = 0;

		while (fscanf(infile, " %d", &lit)==1){
		if (abs(lit)>numatom){
			fprintf(stderr, "Bad hamming file %s\n", initfile);
			exit(1);
		}
		if (lit>0) hamming_target[lit]=1;
		}
		printf("done\n");
	}


	void
	print_false_clauses(long int lowbad)
	{
		int i, j;
		int cl;

		printf("Unsatisfied clauses:\n");
		for (i=0; i<lowbad; i++){
		cl = lowfalse[i];
		for (j=0; j<size[cl]; j++){
			printf("%d ", clause[cl][j]);
		}
		printf("0\n");
		}
		printf("End unsatisfied clauses\n");
	}

	void
	save_false_clauses(long int lowbad)
	{
		int i;

		for (i=0; i<lowbad; i++)
		lowfalse[i] = wfalse[i];
	}
	
	//void initprob(int numvararg, int numclausearg, int* wff)
	bool initprob();

    /* new flipping function based on SAT2004 submission work */
  void flipatom(int toflip)
  {
    int i, j;			
    int toenforce;		
    register int cli;
    register int lit;
    int numocc;
    register int sz;
    register int * litptr;
    int * occptr;
    register int v;
		/* printf("flipping %i\n", toflip); */

    if (toflip == NOVALUE)
    {
      numnullflip++;
      return;
    }

    changed[toflip] = numflip;
    if(atom[toflip] > 0)
    {
      toenforce = -toflip;
    }
    else
    {
      toenforce = toflip;
    }
    atom[toflip] = 1 - atom[toflip];

    if (hamming_flag)
    {
      if (atom[toflip] == hamming_target[toflip])
        hamming_distance--;
      else
        hamming_distance++;
      
      if ((numflip % hamming_sample_freq) == 0)
        fprintf(hamming_fp, "%li %i\n", numflip, hamming_distance);
    }
	    
    numocc = numoccurence[MAXATOM - toenforce];
    occptr = occurence[MAXATOM - toenforce];
    for(i = 0; i < numocc ; i++)
    {
        /* cli = occurence[MAXATOM-toenforce][i]; */
      cli = *(occptr++);

      if (--numtruelit[cli] == 0)
      {
        wfalse[numfalse] = cli;
        wherefalse[cli] = numfalse;
        numfalse++;
          // Decrement toflip's breakcount
        breakcount[toflip]--;

        if (makeflag)
        {
			// Increment the makecount of all vars in the clause
          sz = size[cli];
          litptr = clause[cli];
          for (j = 0; j < sz; j++)
          {
            /* lit = clause[cli][j]; */
            lit = *(litptr++);
            makecount[abs(lit)]++;
          }
        }
      }
      else if (numtruelit[cli] == 1)
      {
        if (watch1[cli] == toflip)
        {
          assert(watch1[cli] != watch2[cli]);
          watch1[cli] = watch2[cli];
        }
		breakcount[watch1[cli]]++;
      }
      else 
      { /* numtruelit[cli] >= 2 */
        if (watch1[cli] == toflip)
        {
            /* find a true literal other than watch1[cli] and watch2[cli] */
          sz = size[cli];
          litptr = clause[cli];
          for (j = 0; j < sz; j++)
          {
            lit = *(litptr++);
            v = abs(lit);
            if ((lit>0) == atom[v] && v != watch1[cli] && v != watch2[cli])
            {
              watch1[cli] = v;
              break;
            }
          }
        }
        else
        if (watch2[cli] == toflip)
        {
            /* find a true literal other than watch1[cli] and watch2[cli] */
          sz = size[cli];
          litptr = clause[cli];
          for (j = 0; j < sz; j++)
          {
            lit = *(litptr++);
            v =abs(lit);
            if ((lit>0) == atom[v] && v != watch1[cli] && v != watch2[cli])
            {
              watch2[cli] = v;
              break;
            }
          }
        }
      }
    }
	    
    numocc = numoccurence[MAXATOM+toenforce];
    occptr = occurence[MAXATOM+toenforce];
    for(i = 0; i < numocc; i++)
    {
        /* cli = occurence[MAXATOM+toenforce][i]; */
      cli = *(occptr++);

      if (++numtruelit[cli] == 1)
      {
        numfalse--;
        wfalse[wherefalse[cli]] = wfalse[numfalse];
        wherefalse[wfalse[numfalse]] = wherefalse[cli];
          // Increment toflip's breakcount
        breakcount[toflip]++;

        if (makeflag)
        {
            // Decrement the makecount of all vars in the clause
          sz = size[cli];
          litptr = clause[cli];
          for (j = 0; j < sz; j++)
          {
              /* lit = clause[cli][j]; */
            lit = *(litptr++);
            makecount[abs(lit)]--;
          }
        }
        watch1[cli] = toflip;
      }
      else
      if (numtruelit[cli] == 2)
      {
        watch2[cli] = toflip;
        breakcount[watch1[cli]]--;
      }
    }
  }

  int pickrandom(void)
  {
    int tofix;

    tofix = wfalse[random()%numfalse];
    return Var(tofix, random()%size[tofix]);
  }

  int pickbest(void)
  {
    int numbreak;
	int tofix;
	int clausesize;
	int i;		
	int best[MAXLENGTH];
	register int numbest;
	register int bestvalue;
	register int var;

    Array<int> canNotFlip;
      // If var in candidateBlockedVars is chosen, then
      // corresponding var in othersToFlip is flipped as well
    Array<int> candidateBlockedVars;
    Array<int> othersToFlip;
    
	tofix = wfalse[random()%numfalse];
	clausesize = size[tofix];
	numbest = 0;
	bestvalue = BIG;

	for (i = 0; i < clausesize; i++)
    {
	  var = abs(clause[tofix][i]);
      int otherToFlip = NOVALUE;
      int blockIdx = getBlock(var - 1);
      if (blockIdx == -1) // Atom not in a block
        numbreak = breakcount[var];
      else // Atom is in a block
      {
          // If evidence atom exists or in block of size 1, then can not flip
        if ((*blockEvidence_)[blockIdx] || (*blocks_)[blockIdx].size() == 1)
        {
          numbreak = INT_MAX;
          //canNotFlip.append(var);
          canNotFlip.append(i);
        }
        else
        {
          numbreak = calculateNumbreak(var, otherToFlip);
          candidateBlockedVars.append(var);
          othersToFlip.append(otherToFlip);
        }
      }

	  if (numbreak <= bestvalue)
      { // Tied or better than previous best
		if (numbreak < bestvalue) numbest = 0;
		bestvalue = numbreak;
		best[numbest++] = var;
	  }
	}

      // Choose one of the best at random
      // (default if none of the following 2 cases occur
    int toflip = best[random()%numbest];

      // 1. If at least 1 clause is broken by best,
      // then with prob. choose a random atom
	if (bestvalue > 0 && (random()%denominator < numerator))
      toflip = abs(clause[tofix][random()%clausesize]);
      // 2. Exactly one best atom
	else if (numbest == 1)
      toflip = best[0];

      // If atom can not be flipped, then null flip
    if (canNotFlip.contains(toflip)) toflip = NOVALUE;
    else
    { // If toflip is in block, then set other to flip
      int idx = candidateBlockedVars.find(toflip);
      if (idx >= 0)
      {
        inBlock = true;
        flipInBlock = othersToFlip[idx];
      }
    }
	return toflip;
  }

  int picktabu(void)
  {
    int numbreak[MAXLENGTH];
    int tofix;
    int clausesize;
    int i;			/* a loop counter */
    int best[MAXLENGTH];	/* best possibility so far */
    int numbest;		/* how many are tied for best */
    int bestvalue;		/* best value so far */
    int noisypick;

    tofix = wfalse[random()%numfalse];
    clausesize = size[tofix];
    for(i = 0; i < clausesize; i++)
      numbreak[i] = breakcount[abs(clause[tofix][i])];

    numbest = 0;
	bestvalue = BIG;

	noisypick = (numerator > 0 && random()%denominator < numerator); 
	for (i = 0; i < clausesize; i++)
    {
      if (numbreak[i] == 0)
      {
		if (bestvalue > 0)
        {
		  bestvalue = 0;
		  numbest = 0;
		}
		best[numbest++] = i;
	  }
	  else if (tabu_length < numflip - changed[abs(clause[tofix][i])])
      {
        if (noisypick && bestvalue > 0)
        { 
		  best[numbest++] = i; 
        }
        else
        {
          if (numbreak[i] < bestvalue)
          {
			bestvalue = numbreak[i];
			numbest = 0;
		  }
		  if (numbreak[i] == bestvalue)
          {
			best[numbest++] = i; 
		  }
        }
      }
    }
	if (numbest == 0) return NOVALUE;
	if (numbest == 1) return Var(tofix, best[0]);
	return (Var(tofix, best[random()%numbest]));
  }

  int picksa(void);

  int countunsat(void)
  {
    int i, j, unsat, lit, sign;
    bool bad;

    unsat = 0;
    for (i = 0; i < numclause; i++)
    {
      bad = true;
      for (j = 0; j < size[i]; j++)
      {
        lit = clause[i][j];
        sign = lit > 0 ? 1 : 0;
        if ( atom[abs(lit)] == sign )
        {
          bad = false;
          break;
        }
      }
      
      if (bad) unsat++;
    }
    return unsat;
  }


	double elapsed_seconds(void) 
	{ 
		double answer;

	#ifdef ANSI
		static long prev_time = 0;
		time( &long_time );
		/* Note:  time(&t) returns t in seconds, so do not need to /CLK_TCK */
		answer = long_time - prev_time;
		prev_time = long_time;
	#endif
	#ifdef NT
		static DWORD prev_time = 0;
		win_time = timeGetTime();
		/* Note:  return value of timeGetTime() is ms, so divide by 1000*/
		answer = (double)(win_time - prev_time) / (double)1000; 
		prev_time = win_time;
	#endif
	#ifdef UNIX
		static struct tms prog_tms;
		static long prev_times = 0;
		(void) times(&prog_tms);
		answer = ((double)(((long)prog_tms.tms_utime)-prev_times))/((double) CLK_TCK);
		prev_times = (long) prog_tms.tms_utime;
	#endif
		return answer; 
	}


	void print_sol_cnf(void)
	{
		int i;
		for(i = 1;i < numatom+1;i++)
		printf("v %i\n", solution[i] == 1 ? i : -i);
	}


	void print_sol_file(char * filename)
	{
		FILE * fp;
		int i;

		if ((fp = fopen(filename, "w"))==NULL){
		fprintf(stderr, "Cannot open output file\n");
		exit(-1);
		}
		for(i = 1;i < numatom+1;i++){
		fprintf(fp, " %i", solution[i] == 1 ? i : -i);
		if (i % 10 == 0) fprintf(fp, "\n");
		}
		if ((i-1) % 10 != 0) fprintf(fp, "\n");
		fclose(fp);
	}


  void print_low_assign(long int lowbad)
  {
    int i;

    printf("Begin assign with lowest # bad = %li\n", lowbad);
    for (i = 1; i <= numatom; i++)
    {
      printf(" %d", lowatom[i]==0 ? -i : i);
      if (i % 10 == 0) printf("\n");
    }

    if ((i-1) % 10 != 0) printf("\n");
    printf("End assign\n");
  }

  void print_current_assign(void)
  {
    int i;

    printf("Begin assign at flip = %li\n", numflip);
    for (i = 1; i <= numatom; i++)
    {
      printf(" %d", atom[i]==0 ? -i : i);
      if (i % 10 == 0) printf("\n");
    }
  
    if ((i-1) % 10 != 0) printf("\n");
    printf("End assign\n");
  }

  void save_low_assign(void)
  {
    int i;

    for (i = 1; i <= numatom; i++)
      lowatom[i] = atom[i];
  }

  void save_solution(void)
  {
    int i;

    for (i = 1; i <= numatom; i++)
      solution[i] = atom[i];
  }

    // Returns true if a fixed atom is in block, otherwise false    
  bool fixedAtomInBlock(const int blockIdx)
  {
    Array<int> block = (*blocks_)[blockIdx];
    for (int i = 0; i < block.size(); i++)
      if (fixedatom[block[i] + 1] > 0) return true;
    return false;
  }

    // Sets the truth values of all atoms in the
    // block blockIdx except for the one with index atomIdx
  void setOthersInBlockToFalse(bool*& assignments,
                               const int& atomIdx,
                               const int& blockIdx)
  {
    Array<int> block = (*blocks_)[blockIdx];
    for (int i = 0; i < block.size(); i++)
    {
      if (i != atomIdx)
        assignments[block[i]] = false;
    }
  }

    // Sets the truth values of all atoms in the
    // block blockIdx except for the one with index atomIdx
  void setOthersInBlockToFalse(const int& atomIdx,
                               const int& blockIdx)
  {
    Array<int> block = (*blocks_)[blockIdx];
    for (int i = 0; i < block.size(); i++)
    {
      if (i != atomIdx)
        atom[block[i] + 1] = 0;
    }
  }

    // returns the index of the block which the atom
    // with index atomIdx is in. If not in any, returns -1
  int getBlock(const int& atomIdx)
  {
    for (int i = 0; i < blocks_->size(); i++)
    {
      int blockIdx = (*blocks_)[i].find(atomIdx);
      if (blockIdx >= 0)
        return i;
    }
    return -1;
  }
    
    // Calculates the change in clauses satisfied by flipping atom
    // and stores idx of other atom to flip in block in flipInBlock
  int calculateChange(const int& atomIdx)
  {
    int blockIdx = getBlock(atomIdx - 1);
    assert(blockIdx >= 0);
      
      //Dealing with atom in a block
    Array<int> block = (*blocks_)[blockIdx];
    int chosen = -1;
    
      // 0->1: choose atom in block which is already 1
    if (atom[atomIdx] == 0)
    {
      for (int i = 0; i < block.size(); i++)
      {
        if (atom[block[i] + 1] == 1)
        {
          chosen = i;
          break;
        }
      }
    }
      // 1->0: choose to flip from others at random
    else
    {
      bool ok = false;
      while(!ok)
      {
        chosen = random() % block.size();
        if (block[chosen] + 1 != atomIdx)
          ok = true;
      }
    }
    
    assert(chosen >= 0);
    inBlock = true;
    flipInBlock = block[chosen] + 1;
    return makecount[atomIdx] + makecount[flipInBlock] -
           breakcount[atomIdx] - breakcount[flipInBlock];    
  }
  
    // Calculates the breakcost by flipping atom
    // and stores idx of other atom to flip in block in flipInBlock
  int calculateNumbreak(const int& atomIdx, int& otherToFlip)
  {
    int blockIdx = getBlock(atomIdx - 1);
    assert(blockIdx >= 0);

      //Dealing with atom in a block
    Array<int> block = (*blocks_)[blockIdx];
    int chosen = -1;
    
      // 0->1: choose atom in block which is already 1
    if (atom[atomIdx] == 0)
    {
      for (int i = 0; i < block.size(); i++)
      {
        if (atom[block[i] + 1] == 1)
        {
          chosen = i;
          break;
        }
      }
    }
      // 1->0: choose to flip the other randomly
      // TODO: choose to flip the other with best breakcost
    else
    {
      bool ok = false;
      while(!ok)
      {
        chosen = random() % block.size();
        if (block[chosen] + 1 != atomIdx)
          ok = true;
      }
    }
    
    assert(chosen >= 0);
    otherToFlip = block[chosen] + 1;
    return breakcount[atomIdx] + breakcount[otherToFlip];    
  }
};

#endif

