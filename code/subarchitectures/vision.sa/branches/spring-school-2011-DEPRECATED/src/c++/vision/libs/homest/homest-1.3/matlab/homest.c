/* ////////////////////////////////////////////////////////////////////////////////
// 
//  Matlab MEX file for homest
//  Copyright (C) 2007-2008  Manolis Lourakis (lourakis **at** ics.forth.gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//////////////////////////////////////////////////////////////////////////////// */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

#include <homest.h>

#include "mex.h"

/**
#define DEBUG
**/

#define MAX(A, B)     ((A)>=(B)? (A) : (B))


/* display printf-style error messages in matlab */
static void matlabFmtdErrMsgTxt(char *fmt, ...)
{
char  buf[256];
va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);

  mexErrMsgTxt(buf);
}

/* display printf-style warning messages in matlab */
static void matlabFmtdWarnMsgTxt(char *fmt, ...)
{
char  buf[256];
va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);

  mexWarnMsgTxt(buf);
}

/* matlab matrices are in column-major, this routine converts them to row major for homest */
static double *getTranspose(mxArray *Am)
{
int m, n;
double *At, *A;
register int i, j;

  m=mxGetM(Am);
  n=mxGetN(Am);
  A=mxGetPr(Am);
  At=mxMalloc(m*n*sizeof(double));

  for(i=0; i<m; i++)
    for(j=0; j<n; j++)
      At[i*n+j]=A[i+j*m];
  
  return At;
}

/*
[H, idxOutliers]=homest(pts0, pts1, inlPcent, normalize, NLrefine, verbose);
      pts0, pts1 are the input matching point pairs
      inlPcent is the expected fraction of inliers in the imput points
      normalize specifies weather Hartley's normalization should be applied to
                input points prior to DLT estimation (default)
      NLrefine controls non-linear refinement can be one of: 'norefine', 'xfer_err1',
               'xfer_err2' ('xfer_err'), 'sym_xfer_err', 'sampson_err', 'repr_err'
               or empty (implying 'norefine', default)
      verbose is optional

An affine homography can be estimated with
      [H, idxOutliers]=homest(pts0, pts1, inlPcent, normalize, 'aff');

*/

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *Prhs[])
{
register int i;
int normalize, NLrefine, affine=0, verbose, nbOutliers, *idxOutliers;
double (*pts0)[2], (*pts1)[2], inlPcent, H[NUM_HPARAMS];
register double *pdbl;
mxArray **prhs=(mxArray **)&Prhs[0];
int nmatches, len, status;

  /* parse input args; start by checking their number */
  if(nrhs<4 && nrhs>6)
    matlabFmtdErrMsgTxt("homest: between 4 and 6 input arguments required (got %d).", nrhs);
  if(nlhs>2)
    matlabFmtdErrMsgTxt("homest: at most 2 output arguments returned (got %d).", nlhs);
    
  /** pts0 **/
  /* first argument must be a two-column matrix */
  if(!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) || mxGetN(prhs[0])!=2)
    matlabFmtdErrMsgTxt("homest: first argument must be a two-column matrix (got %dx%d).", mxGetM(prhs[0]), mxGetN(prhs[0]));
  pts0=(double (*)[2])getTranspose(prhs[0]);
  nmatches=mxGetM(prhs[0]);

  /** pts1 **/
  /* second argument must be a two-column matrix */
  if(!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) || mxGetN(prhs[1])!=2)
    matlabFmtdErrMsgTxt("homest: second argument must be a two-column matrix (got %dx%d).", mxGetM(prhs[1]), mxGetN(prhs[1]));
  if(mxGetM(prhs[1])!=nmatches)
    matlabFmtdErrMsgTxt("homest: two first arguments should have the same number of rows (got %d and %d).", nmatches, mxGetM(prhs[1]));
  pts1=(double (*)[2])getTranspose(prhs[1]);

  /** inlPcent **/
  /* the third argument must be a scalar */
  if(!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]) || mxGetM(prhs[2])!=1 || mxGetN(prhs[2])!=1)
    mexErrMsgTxt("homest: inlPcent must be a scalar.");
  inlPcent=mxGetScalar(prhs[2]);

  /** normalize **/
  /* check whether the fourth argument is a scalar */
  if(nrhs>=4 && (mxIsDouble(prhs[3]) && !mxIsComplex(prhs[3]) && mxGetM(prhs[3])==1 && mxGetN(prhs[3])==1)){
    normalize=mxGetScalar(prhs[3])!=0.0;

    ++prhs;
    --nrhs;
  }
  else
    normalize=1;

  /** NLrefine or affine **/
  /* check whether fifth argument is a string */
  if(nrhs>=4 && mxIsChar(prhs[3])==1 && mxGetM(prhs[3])==1){
    char *str;

    /* examine supplied name */
    len=mxGetN(prhs[3])+1;
    str=mxCalloc(len, sizeof(char));
    status=mxGetString(prhs[3], str, len);
    if(status!=0)
      mexErrMsgTxt("homest: not enough space. String is truncated.");

    for(i=0; str[i]; ++i)
      str[i]=tolower(str[i]);

    if(!strcmp(str, "norefine")) NLrefine=HOMEST_NO_NLN_REFINE;
    else if(!strcmp(str, "xfer_err") || !strcmp(str, "xfer_err2")) NLrefine=HOMEST_XFER_ERROR2;
    else if(!strcmp(str, "xfer_err1")) NLrefine=HOMEST_XFER_ERROR1;
    else if(!strcmp(str, "sym_xfer_err")) NLrefine=HOMEST_SYM_XFER_ERROR;
    else if(!strcmp(str, "sampson_err")) NLrefine=HOMEST_SAMPSON_ERROR;
    else if(!strcmp(str, "repr_err")) NLrefine=HOMEST_REPR_ERROR;
    else if(!strcmp(str, "aff")) { NLrefine=HOMEST_NO_NLN_REFINE; affine=1; }
    else matlabFmtdErrMsgTxt("homest: unknown minimization type '%s'.", str);

    mxFree(str);

    ++prhs;
    --nrhs;
  }
  else
    NLrefine=HOMEST_NO_NLN_REFINE;

  /** verbose **/
  /* the sixth argument must be a scalar */
  if(nrhs>=4){
    if(!mxIsDouble(prhs[3]) || mxIsComplex(prhs[3]) || mxGetM(prhs[3])!=1 || mxGetN(prhs[3])!=1)
      mexErrMsgTxt("homest: verbose must be a scalar.");
    verbose=mxGetScalar(prhs[3])!=0.0;
  }
  else
    verbose=0;

  if(nlhs>1) /* outlier indices should be returned */
    idxOutliers=mxMalloc(nmatches*sizeof(int));
  else
    idxOutliers=NULL;

  /* invoke homest */
  if(!affine)
    homest(pts0, pts1, nmatches, inlPcent, H, normalize, NLrefine, idxOutliers, &nbOutliers, verbose);
  else
    homestaff(pts0, pts1, nmatches, inlPcent, H, normalize, idxOutliers, &nbOutliers, verbose);

  /* copy back return results */
  /** H **/
  plhs[0]=mxCreateDoubleMatrix(3, 3, mxREAL);
  pdbl=mxGetPr(plhs[0]);
  /* return transposed, i.e. column major */
  pdbl[0]=H[0]; pdbl[1]=H[3]; pdbl[2]=H[6];
  pdbl[3]=H[1]; pdbl[4]=H[4]; pdbl[5]=H[7];
  pdbl[6]=H[2]; pdbl[7]=H[5]; pdbl[8]=H[8];

  /** idxOutliers **/
  if(nlhs>1){
    plhs[1]=mxCreateDoubleMatrix(nbOutliers, 1, mxREAL);
    pdbl=mxGetPr(plhs[1]);
    for(i=0; i<nbOutliers; ++i)
      *pdbl++=idxOutliers[i];

    mxFree(idxOutliers);
  }

  /* cleanup */
  mxFree(pts0);
  mxFree(pts1);
}
