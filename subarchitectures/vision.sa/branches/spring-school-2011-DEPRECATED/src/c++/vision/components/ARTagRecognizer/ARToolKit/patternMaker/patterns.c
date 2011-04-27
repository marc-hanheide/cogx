/*
 * Copyright (c) 2002 The University of Utah
 * All Rights Reserved. 
 * 
 * This is an experimental, research release of a component of the
 * Alpha_1 Geometric Modelling System, and as such is subject to a
 * license agreement with Engineering Geometry Systems, Inc. and may only
 * be used under the terms of that agreement.  Any other use is
 * prohibited.
 * 
 * Having received prior authorization from EGS, the University of Utah grants
 * permission to use, copy, or modify this software and its
 * documentation for educational, research and non-profit purposes,
 * without fee, and without a written agreement.  It is explicitly prohibited to
 * redistribute this software or any derivative. 
 * 
 *       IN NO EVENT SHALL THE UNIVERSITY OF UTAH OR ENGINEERING GEOMETRY
 *       SYSTEMS, INC.  BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
 *       SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST
 *       PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 *       DOCUMENTATION, EVEN IF THE UNIVERSITY OF UTAH HAVE BEEN ADVISED
 *       OF THE POSSIBILITY OF SUCH DAMAGES.
 * 
 *       THE UNIVERSITY OF UTAH AND ENGINEERING GEOMETRY SYSTEMS, INC.
 *       SPECIFICALLY DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
 *       TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *       PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 *       "AS IS" BASIS, AND THE UNIVERSITY OF UTAH AND ENGINEERING
 *       GEOMETRY SYSTEMS, INC. HAVE NO OBLIGATIONS TO PROVIDE
 *       MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * The above permission is granted provided that this whole notice
 * appears in all copies.
 * 
 * Contacts are: 
 * 
 *       EMail:  {dejohnso, cohen} @ cs.utah.edu
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* Fill in this section with the size of the (square) matrix
 * you want to compute the patterns for
 */
#define SIZE 4 /* SIZE = n */

/* THe name of the output file
 * WARNING: the proram will die ungracefully if there is a problem
 * with what you enter here. No error checking is done.
 * It will also overwrite the file if it exists.
 */
#define OUTPUT "4x4"

/* Make the delimiter whatever you want */
#define DELIM " "


/************* END USER ALTERED CODE *****************/
/* You shouldn't have to edit below here unless you want to 
 * change the way the program runs
 *
 */
#define MAX_STR_LEN SIZE*SIZE+1
#define NUM_ELTS SIZE*SIZE
#define RIDX(i,j,n) ((i)*(n)+(j))

/* Globals */
//int pattern[NUM_POS_PATTERNS][NUM_ELTS];
int **pattern;

/* declarations */
void makePattern(int n, long i, short int *pattern);
long uniqueBin(long n, short int *bin);
int isUnique(const short int *tmp, long numPatterns);
void printArray(short int *array, int size);
int equal(int I, const short int *b, int size);
void rotate90(short int *dst, const short int *src, int dim);
void copy(int index, const short int *src, int size);
void printPatterns(long num);
int allocateMemory(int posPatterns);
int freeMemory(int posPatterns);


int main(int argc, char **argv){
  short int tmp[NUM_ELTS];
  int i;
  long numPatterns=0;
  int num_pos_patterns;

  num_pos_patterns = (int)pow(2, SIZE*SIZE) - 1;
  if (!allocateMemory(num_pos_patterns))
  {
      fprintf(stderr, "Could not allocate memory!\n");
      return 0;
  }
  
  
  for (i = 0; i < num_pos_patterns; ++i){
/*
      if ( i%100 == 0)
	  printf("checking %d\n",i);
*/      
      
      makePattern(NUM_ELTS, i, tmp);
    /* check to see if the pattern is 
     * rotationaly unique
     */

    /* copy it to the valid pattern array */
    /* if it isn't unique, kick the stack back */
    copy(numPatterns++, tmp, NUM_ELTS);
    if (!isUnique(tmp, numPatterns))
	numPatterns--;
    
  }
  
  printPatterns(numPatterns);


  freeMemory(num_pos_patterns);
  
  return 1;
}

int allocateMemory(int posPatterns)
{
    int i;
    
    pattern = (int **)calloc(posPatterns, sizeof(int *));
    if (pattern == NULL)
	return 0;
    else
	for (i = 0; i < posPatterns; ++i)
	{
	    pattern[i] = (int *)calloc(NUM_ELTS, sizeof(int));
	    if (pattern[i] == NULL)
		return 0;
	}
    return 1;
}

int freeMemory(int posPatterns)
{
    int i;

    for (i = 0; i < posPatterns; ++i)
	free(pattern[i]);

    free(pattern);

    return 1;
}


void makePattern(int n, long I, short int *pattern){
  short int bin[MAX_STR_LEN];
  long i, pad;

  pad = uniqueBin(I, bin);
  /* pad the string with zeros if we need to */
  for (i = 0; i <= n - pad; ++i)
    pattern[i] = 0;
  /* now put in the number
   * reverse bin so that we get the bin representation of I
   */
  for (i = pad-1; i >= 0; --i)
    pattern[NUM_ELTS-1-i] = bin[i];
}

long uniqueBin(long N, short int *bin){
  int m=1, n=N, r, index=0;

  while (m != 0){
    m = floor(n/2);
    r = n % 2;
    bin[index++] = r;
    n = m;
  }
  return index;
}

int isUnique(const short int *tmp, long numPatterns){
  int i;
  short int rot1[NUM_ELTS];
  short int rot2[NUM_ELTS];

  /* check for rotate 0 */
  /* don't check against the last one in the stack since it is a copy of tmp */
  for (i = 0; i < numPatterns-1; ++i)
    if (equal(i, tmp, NUM_ELTS))
      return 0;

  /* check for rotate 90 */
  rotate90(rot1, tmp, SIZE);
  for (i = 0; i < numPatterns; ++i)
    if (equal(i, rot1, NUM_ELTS))
      return 0;

  /* check for rotate 180 */
  rotate90(rot2, rot1, SIZE);
  for (i = 0; i < numPatterns; ++i)
    if (equal(i, rot2, NUM_ELTS))
      return 0;

  /* check for rotate 270 */
  rotate90(rot1, rot2, SIZE);
  for (i = 0; i < numPatterns; ++i)
    if (equal(i, rot1, NUM_ELTS))
      return 0;

  return 1;
}

void printArray(short int *array, int size){
  long i;
  
  for (i = 0; i < size; ++i)
    printf("%d%s", array[i], DELIM);
}

int equal(int I, const short int *b, int size){
  int i;

  for (i = 0; i < size; ++i)
    if (pattern[I][i] != b[i])
      return 0;
  return 1;
}

void rotate90(short int *dst, const short int *src, int dim){
  int i, j;

  for (i = 0; i < dim; ++i)
    for (j = 0; j < dim; ++j)
      dst[RIDX(dim-1-j, i, dim)] = src[RIDX(i, j, dim)];
}

void copy(int index, const short int *src, int size){
  int i;

  for (i = 0; i < size; ++i)
    pattern[index][i] = src[i];
}

void printPatterns(long num){
  FILE *out;
  int i, j, k;

  out = fopen(OUTPUT, "w");
  fprintf(out, "%ld\n", num);
  for (i = 0; i < num; ++i){
      for (j = 0; j < SIZE; ++j)
      {
	for (k = 0; k < SIZE; ++k)
	    fprintf(out, "%d%s", pattern[i][j*SIZE+k], DELIM);
	fprintf(out,"\n");
      }
      
    fprintf(out, "\n");
  }
  fclose(out);  
}
