/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-09  Manolis Lourakis (lourakis **at** ics.forth.gr)
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
/////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>

#include "util.h"
#include "homest.h"
#include "maplefuncs.h"

#include "lqs.h"
#include <lm.h>

#define USE_BUCKETS

#define HOWTO_CALC_RESIDUALS homogResidualsAlg    // algebraic distances
//#define HOWTO_CALC_RESIDUALS homogResidualsGeom   // geometric distances

#define SQR(x) ((x)*(x))
#define UNROLLBLOCKSIZE  8 

/* global variables used by various homography estimation routines */
static struct {
  double **cfs;
  double *rhs; /* affine case only */
  double (*pts0)[2], (*pts1)[2];
  int *inliersidx, numInliers;

  /* following elements are used for avoiding multiple mallocs in est2DPtHomog()/est2DPtAffHomog */
  double *M;
  double *b; /* affine case only */
  int Mrows;
} globs;


/* memory clear using loop unrolling and blocking.
 * see http://www.abarnett.demon.co.uk/tutorial.html
 */

static void dblzero_(double *x, int n)
{ 
register int i=0; 
int blockn;

  /* n may not be divisible by UNROLLBLOCKSIZE, 
  * go as near as we can first, then tidy up.
  */ 
  blockn=(n/UNROLLBLOCKSIZE)*UNROLLBLOCKSIZE; 

  /* unroll the loop in blocks of `UNROLLBLOCKSIZE' */ 
  while(i<blockn) { 
    x[i]  =0.0;
    x[i+1]=0.0;
    x[i+2]=0.0;
    x[i+3]=0.0;
    x[i+4]=0.0;
    x[i+5]=0.0;
    x[i+6]=0.0;
    x[i+7]=0.0;

    /* update the counter */ 
    i+=UNROLLBLOCKSIZE;
  } 

 /* 
  * There may be some left to do.
  * This could be done as a simple for() loop, 
  * but a switch is faster (and more interesting) 
  */ 

  if(i<n){ 
    /* Jump into the case at the place that will allow
     * us to finish off the appropriate number of items. 
     */ 

    switch(n-i){ 
      case 7 : x[i]=0.0; i++;
      case 6 : x[i]=0.0; i++;
      case 5 : x[i]=0.0; i++;
      case 4 : x[i]=0.0; i++;
      case 3 : x[i]=0.0; i++;
      case 2 : x[i]=0.0; i++;
      case 1 : x[i]=0.0; i++;
    }
  }

  return;
}

/* estimate "point" homography H s.t. p'=H*p, with p, p' specified by ptsidx */
static int est2DPtHomog(double *h, int npts, int *ptsidx)
{
register int i, j;
double *mat, *coefs;
register double *matrow;
static int (*const solver)(double *, int, int, double *)=
                    homest_min_Ax_normSVD; /* using SVD is more stable but slower & needs more memory */
                    //homest_min_Ax_normEIG;

  if(h==NULL){ /* cleanup */
    (*solver)(NULL, 0, 0, NULL);

    /* free working memory */
    free(globs.M);
    globs.M=globs.b=NULL; globs.Mrows=0;
    return 0;
  }

  if(globs.Mrows!=2*npts){ /* free existing matrix and allocate a new one */
    if(globs.M) /* is there anything to free? */
      free(globs.M);

    if((mat=(double *)malloc(2*npts*NUM_HPARAMS*sizeof(double)))==NULL){
      fprintf(stderr, "Memory allocation request failed in est2DPtHomog()\n");
      exit(1);
    }
    globs.M=mat;
    globs.b=NULL;
    globs.Mrows=2*npts;
  }

  mat=globs.M;

  /* fill in constraints matrix row by row */
  for(i=0, matrow=mat; i<npts; ++i, matrow+=2*NUM_HPARAMS){
    coefs=globs.cfs[ptsidx[i]];
    for(j=0; j<2*NUM_HPARAMS; ++j) // 2 rows for each point pair
      matrow[j]=coefs[j];
  }

  /* solve min |mat*h|, |h|=1 */
  if(!(*solver)(mat, 2*npts, NUM_HPARAMS, h))
    return 0;

  return 1;
}

/* compute the (squared) algebraic residuals corresponding to homog in the second image */
static void homogResidualsAlg(double *homog, int numres, double *resid)
{
register int i, j;
double *cf1, *cf2, sum1, sum2;

  for(i=0; i<numres; ++i){
    cf1=globs.cfs[i];
    cf2=cf1+NUM_HPARAMS;
    sum1=sum2=0.0;
    for(j=0; j<NUM_HPARAMS; ++j){
      sum1+=cf1[j]*homog[j];
      sum2+=cf2[j]*homog[j];
    }
    resid[i]=sum1*sum1 + sum2*sum2;
  }
}

/* compute the geometric residuals corresponding to homog as the (squared) distance between transfered points in both images */
static void homogResidualsGeom(double *homog, int numres, double *resid)
{
register int i;
double *pt0, *pt1, pt01[4];

  for(i=0; i<numres; ++i){
    pt0=globs.pts0[i];
    pt1=globs.pts1[i];

    calc2DHomogSymTransfer(pt0, pt1, homog, pt01);
    resid[i]=(SQR(pt01[0]-pt0[0]) + SQR(pt01[1]-pt0[1])) + (SQR(pt01[2]-pt1[0]) + SQR(pt01[3]-pt1[1]));
  }
}

/* symmetric transfer error and Jacobian */
static void HomSymXfer(double *h, double *x, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;

  for(i=0; i<ninl; ++i){
    calc2DHomogSymTransfer(pts0[inlidx[i]], pts1[inlidx[i]], h, x+i*4);
  }
}

static void jacHomSymXfer(double *h, double *jac, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;
register double *jacrows;

  for(i=0, jacrows=jac; i<ninl; ++i, jacrows+=4*NUM_HPARAMS){
    calc2DHomogSymTransferJac(pts0[inlidx[i]], pts1[inlidx[i]], h, (double (*)[9])jacrows);
  }
}

/* Sampson error and Jacobian */
static void homSampsonerr(double *h, double *x, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;

  for(i=0; i<ninl; ++i){
    calc2DHomogSampsonErr(pts0[inlidx[i]], pts1[inlidx[i]], h, x+i);
  }
}

static void jachomSampsonerr(double *h, double *jac, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
double (*pts1)[2]=globs.pts1;
register double *jacrow;

  for(i=0, jacrow=jac; i<ninl; ++i, jacrow+=NUM_HPARAMS){
    calc2DHomogSampsonErrGrads(pts0[inlidx[i]], pts1[inlidx[i]], h, jacrow);
  }
}

/* (non symmetric) 2nd image transfer error and Jacobian */
static void HomXfer(double *h, double *x, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;

  for(i=0; i<ninl; ++i){
    calc2DHomogTransfer(pts0[inlidx[i]], h, x+i*2);
  }
}

static void jacHomXfer(double *h, double *jac, int m, int n, void *adata)
{
register int i;
int ninl=globs.numInliers, *inlidx=globs.inliersidx;
double (*pts0)[2]=globs.pts0;
register double *jacrows;

  for(i=0, jacrows=jac; i<ninl; ++i, jacrows+=2*NUM_HPARAMS){
    calc2DHomogTransferJac(pts0[inlidx[i]], h, (double (*)[9])jacrows);
  }
}

/* reprojection error and Jacobian */
static void HomRepr(double *p, double *x, int m, int n, void *adata)
{
int ninl=globs.numInliers;
register int i;

  for(i=0; i<ninl; ++i){
    calc2DHomogReprojection(p, p+NUM_HPARAMS+i*2, x+i*4);
  }
}

static void jacHomRepr(double *p, double *jac, int m, int n, void *adata)
{
register int i, cofs;
int ninl=globs.numInliers;
register double *jacrow0, *jacrow1, *jacrow2, *jacrow3;

  //for(i=0; i<m*n; ++i) jac[i]=0.0;
  dblzero_(jac, m*n);

  for(i=0, jacrow0=jac; i<ninl; ++i, jacrow0+=4*m){
    jacrow1=jacrow0+m;
    jacrow2=jacrow1+m;
    jacrow3=jacrow2+m;
    cofs=NUM_HPARAMS + (i<<1); // column offset: NUM_HPARAMS + i*2;
    calc2DHomogReprojectionJac(p, p+cofs, jacrow2, jacrow3, 
                              jacrow0+cofs, jacrow1+cofs, jacrow2+cofs, jacrow3+cofs);
  }
}


/* nonlinear refinement of a homography; based on minimizing symmetric transfer error */
static void refine2DPtHomog(double *h, int howto, int verbose)
{
register int i, j;
double opts[LM_OPTS_SZ], info[LM_INFO_SZ], *p, *x;
int m, n; // # unknowns & # constraints
void (*err)(double *p, double *hx, int m, int n, void *adata);
void (*jacerr)(double *p, double *j, int m, int n, void *adata);
int ret;

static char *howtoname[]={"HOMEST_NO_NLN_REFINE", "HOMEST_XFER_ERROR/HOMEST_XFER_ERROR2",
                    "HOMEST_SYM_XFER_ERROR", "HOMEST_SAMPSON_ERROR", "HOMEST_REPR_ERROR",
                    "HOMEST_XFER_ERROR1", "HOMEST_REPR_ERROR_SPARSE"};

  opts[0]=LM_INIT_MU; opts[1]=1E-12; opts[2]=1E-12; opts[3]=1E-15;
  opts[4]=LM_DIFF_DELTA; // relevant only if the finite difference Jacobian version is used 

  switch(howto){
    case HOMEST_XFER_ERROR:
      m=NUM_HPARAMS; n=2*globs.numInliers; // two measurements per point pair
      if((x=(double *)malloc(n*sizeof(double)))==NULL){
        fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
        exit(1);
      }
      for(i=0; i<globs.numInliers; ++i){
        j=i<<1; // 2*i
        x[j]  =globs.pts1[globs.inliersidx[i]][0];
        x[j+1]=globs.pts1[globs.inliersidx[i]][1];
      }
      p=h;
      err=HomXfer;
      jacerr=jacHomXfer;
    break;

    case HOMEST_SYM_XFER_ERROR:
      m=NUM_HPARAMS; n=4*globs.numInliers; // four measurements per point pair
      if((x=(double *)malloc(n*sizeof(double)))==NULL){
        fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
        exit(1);
      }
      for(i=0; i<globs.numInliers; ++i){
        j=i<<2; // 4*i
        x[j]  =globs.pts0[globs.inliersidx[i]][0];
        x[j+1]=globs.pts0[globs.inliersidx[i]][1];
        x[j+2]=globs.pts1[globs.inliersidx[i]][0];
        x[j+3]=globs.pts1[globs.inliersidx[i]][1];
      }
      p=h;
      err=HomSymXfer;
      jacerr=jacHomSymXfer;
    break;

    case HOMEST_SAMPSON_ERROR:
      m=NUM_HPARAMS; n=globs.numInliers; // one measurement per point pair
      if((x=(double *)malloc(n*sizeof(double)))==NULL){
        fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
        exit(1);
      }
      //for(i=0; i<n; ++i) x[i]=0.0;
      dblzero_(x, n);
      p=h;
      err=homSampsonerr;
      jacerr=jachomSampsonerr;
    break;

    case HOMEST_REPR_ERROR:
      m=NUM_HPARAMS+2*globs.numInliers; n=4*globs.numInliers; // four measurements per point pair
      if((p=(double *)malloc(m*sizeof(double)))==NULL){
        fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
        exit(1);
      }
      /* parameter vector layout: h, x_1, ..., x_k */
      for(i=0; i<NUM_HPARAMS; ++i)
        p[i]=h[i];
      for(i=0; i<globs.numInliers; ++i){
        j=i<<1; // i*2
        p[NUM_HPARAMS+j]  =globs.pts0[globs.inliersidx[i]][0];
        p[NUM_HPARAMS+j+1]=globs.pts0[globs.inliersidx[i]][1];
      }

      if((x=(double *)malloc(n*sizeof(double)))==NULL){
        fprintf(stderr, "Memory allocation request failed in refine2DPtHomog()\n");
        exit(1);
      }
      for(i=0; i<globs.numInliers; ++i){
        j=i<<2; // i*4
        x[j]  =globs.pts0[globs.inliersidx[i]][0];
        x[j+1]=globs.pts0[globs.inliersidx[i]][1];
        x[j+2]=globs.pts1[globs.inliersidx[i]][0];
        x[j+3]=globs.pts1[globs.inliersidx[i]][1];
      }
      err=HomRepr;
      jacerr=jacHomRepr;
    break;

    default:
      fprintf(stderr, "invalid or unknown non-linear homography refinement choice [%d] made in refine2DPtHomog(), exiting\n", howto);
      exit(1);
  }

  ret=dlevmar_der(err, jacerr, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // with analytic Jacobian
  //ret=dlevmar_dif(err, p, x, m, n, 1000, opts, info, NULL, NULL, NULL); // no Jacobian

  if(verbose){
    fprintf(stdout, "\nRefinement method %s, %d measurements, %d variables\n", howtoname[howto], n, m);
    fprintf(stdout, "LM returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals\n",
                    ret, info[5], info[6], info[1]/globs.numInliers, info[0]/globs.numInliers, (int)info[7], (int)info[8]);
#if 0
    fprintf(stdout, "\nSolution: ");
    for(i=0; i<m; ++i)
      fprintf(stdout, "%.7g ", p[i]);
    fprintf(stdout, "\n");
#endif
  }

#if 0
  /* when minimizing reprojection error, the code below prints
   * the (inlying) refined feature coordinates in the two images
   */
  if(howto==HOMEST_REPR_ERROR){
    for(i=0; i<globs.numInliers; ++i){
      double x0, y0, x1, y1, s;

      x0=p[NUM_HPARAMS+i*2];
      y0=p[NUM_HPARAMS+i*2+1];

      x1=p[0]*x0+p[1]*y0+p[2];
      y1=p[3]*x0+p[4]*y0+p[5];
       s=p[6]*x0+p[7]*y0+p[8];
      x1/=s;
      y1/=s;
      //printf("%d: ", globs.inliersidx[i]); // this displays the point's input index
      printf("%.4lf %.4lf  %.4lf %.4lf\n", x0, y0, x1, y1);
    }
  }
#endif

  if(p!=h){
    for(i=0; i<NUM_HPARAMS; ++i)
      h[i]=p[i];
    free(p);
  }

  free(x);
}

/* robust, non-linear homography estimation from "nmatches" matched point features, possibly
 * including outliers. "inpts0", "inpts1" contain the matched image point coordinates, "inlPcent"
 * is the expected percentage of inliers (>=0.5), "H01" contains the estimated homography upon
 * return, "normalize" is a flag specifing whether input coordinates should be normalized according
 * to Hartley's suggestion, "NLrefine" specifies which cost function should be employed for the
 * non-linear refinement step (see homest.h for appropriate values), "idxOutliers" points to
 * sufficiently large memory which upon return is set to the indices of the detected outlying
 * points (pass NULL if don't care), "nbOutliers" contains the number of outliers, "verbose"
 * specifies the verbosity level
 */
HOMEST_API_MOD int HOMEST_CALL_CONV
homest(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
            int normalize, int NLrefine, int *idxOutliers, int *nbOutliers, int verbose)
{
register int i, j;
int isSqr=1, maxNbSol=1;
double gate=2.0, premResid=-1.0, sampleProb=0.99, outlierThresh;
int *outliersMap, ret, **sets=NULL, nbSets=0;
double (*pts0)[2], (*pts1)[2], L0[9], L1[9];

  if(normalize){
    pts0=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    pts1=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    if(!pts0 || !pts1){
      fprintf(stderr, "Memory allocation request failed in homest()\n");
      exit(1);
    }
    homest_normalizePts(inpts0, pts0, nmatches, L0);
    homest_normalizePts(inpts1, pts1, nmatches, L1);
  }
  else{
    pts0=inpts0;
    pts1=inpts1;
  }

  globs.cfs=(double **)malloc(nmatches*sizeof(double *));
  if(!globs.cfs){
    fprintf(stderr, "Error: not enough memory for 'globs.cfs' in homest()\n");
    exit(1);
  }

  /* one "big" malloc instead of several "small" ones */
  globs.cfs[0]=(double *)malloc(2*nmatches*NUM_HPARAMS*sizeof(double)); /* two equations per point */
  if(!globs.cfs[0]){
    fprintf(stderr, "Error: not enough memory for 'globs.cfs[0]' in homest()\n");
    exit(1);
  }
	for(i=1; i<nmatches; ++i)
    globs.cfs[i]=globs.cfs[i-1] + 2*NUM_HPARAMS;
  globs.rhs=NULL;

	for(i=0; i<nmatches; ++i)
    calc2DHomogLinCoeffs2(pts0[i], pts1[i], globs.cfs[i]);

  nbSets=lqs_numtries(MIN_HMATCHED_PTS, inlPcent, sampleProb);
  sets=lqs_allocsets(MIN_HMATCHED_PTS, nbSets);

#ifdef USE_BUCKETS
  nbSets=homest_genRandomSetsWithBuckets(pts1, MIN_HMATCHED_PTS, nmatches, nbSets, sets);
#else
  nbSets=homest_genRandomSetsNoBuckets(MIN_HMATCHED_PTS, nmatches, nbSets, sets);
#endif /* USE_BUCKETS */

  globs.pts0=pts0; globs.pts1=pts1;
  globs.M=NULL; globs.Mrows=0;

  if(!(outliersMap=(int *)malloc(nmatches*sizeof(int)))){
    fprintf(stderr, "Error: not enough memory for 'outliersMap' in homest()\n");
    exit(1);
  }
  verbose=verbose>1;

#if 1
  j=lqsfit(nmatches, MIN_HMATCHED_PTS, sets, nbSets, HOWTO_CALC_RESIDUALS, est2DPtHomog,
            isSqr, verbose, maxNbSol, gate, premResid, NUM_HPARAMS, inlPcent, H01,
            NULL, outliersMap, nbOutliers, &outlierThresh);
#else
  //outlierThresh=ransac_getthresh(0.01, 4); // assume s=.01, symmetric distance involves 4 squared terms
  outlierThresh=sqrt(0.01*0.01*13.27670414); // assume s=.01, symmetric distance involves 4 squared terms
  j=ransacfit(nmatches, MIN_HMATCHED_PTS, sets, nbSets, homogResidualsGeom, est2DPtHomog,
            isSqr, verbose, maxNbSol, outlierThresh, 0, NUM_HPARAMS, inlPcent, H01, NULL, outliersMap, nbOutliers);
#endif

  if(verbose){
    fprintf(stderr, "Outlier threshold: %g\n", outlierThresh);
    fprintf(stderr, "homest(): LQS fit returned %d, %d outliers [out of %d]\n", j, *nbOutliers, nmatches);
  }

  if(sets) lqs_freesets(sets);

  if(j!=0){
    globs.inliersidx=(int *)malloc((nmatches - *nbOutliers)*sizeof(int));
    if(!globs.inliersidx){
      fprintf(stderr, "Error: not enough memory for 'globs.inliersidx' in homest()\n");
      exit(1);
    }

    for(i=j=0; i<nmatches; ++i)
      if(!outliersMap[i]) globs.inliersidx[j++]=i;

    /* LS estimation on inliers */
    globs.numInliers=nmatches - *nbOutliers;
    est2DPtHomog(H01, globs.numInliers, globs.inliersidx);

    /* expose outliers */
    if(idxOutliers!=NULL)
      for(i=j=0; i<nmatches; ++i)
        if(outliersMap[i]) idxOutliers[j++]=i;

    ret=HOMEST_OK;

#if 0
    /* include the following code fragment to print the (unnormalized) matching point pairs found to be inlying */
    for(i=0; i<globs.numInliers; ++i){
      printf("%.4lf %.4lf  %.4lf %.4lf\n", inpts0[globs.inliersidx[i]][0], inpts0[globs.inliersidx[i]][1],
                                          inpts1[globs.inliersidx[i]][0], inpts1[globs.inliersidx[i]][1]);
    }
#endif

  }
  else{ /* LQS failed */
    dblzero_(H01, NUM_HPARAMS);
    *nbOutliers=nmatches;
    globs.numInliers=0; /* makes sure the non-linear refinement below is avoided */
    globs.inliersidx=NULL;
    ret=HOMEST_ERR;
  }

  est2DPtHomog(NULL, 0, NULL); /* release memory */

  if(normalize){
    free(pts0);
    free(pts1);
    homest_denormalizeH(H01, L0, L1, H01);
  }

  /* the DLT estimate has now been computed. Time for the non linear refinement */
  if(globs.numInliers>=NUM_HPARAMS && NLrefine!=HOMEST_NO_NLN_REFINE){
    /* use the unnormalized points for the refinement */
    if(NLrefine!=HOMEST_XFER_ERROR1){
      globs.pts0=inpts0; globs.pts1=inpts1;
      refine2DPtHomog(H01, NLrefine, verbose);
    }
    else{ /* trick refine2DPtHomog() to use the transfer error in the 1st image */
      double H01_1[9];

      homest_matinvLU(H01, H01_1, 3);
      globs.pts0=inpts1; globs.pts1=inpts0;
      refine2DPtHomog(H01_1, HOMEST_XFER_ERROR2, verbose);
      homest_matinvLU(H01_1, H01, 3);
      homest_matinvLU(NULL, NULL, 0); /* cleanup */
    }
  }

  /* just in case ... */
  globs.pts0=globs.pts1=NULL;
  globs.numInliers=0;

  if(globs.inliersidx) free(globs.inliersidx);
  free(globs.cfs[0]);
	free(globs.cfs);
  free(outliersMap);

  globs.cfs=NULL;
  globs.inliersidx=NULL;

  return ret;
}

/* Compute the Root Mean Squared (RMS) and Root Median Squared (RMedS) symmetric average distance
 * error pertaining to a homography H that has been estimated from pairs of corresponding points.
 * Note that the RMS measure is sensitive to mismatched points while the RMedS is not
 */

#define _MEDIAN_(a, n) lqs_kth_smallest(a, n, (((n)&1)? ((n)/2) : (((n)/2)-1)))

HOMEST_API_MOD void HOMEST_CALL_CONV
homest_RMS_RMedS(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double H[NUM_HPARAMS],
                 double *rms, double *rmeds)
{
register int i;
double H1[NUM_HPARAMS], *errors, sum, q1[2], q2[2], s, dist1, dist2;

  if((errors=(double *)malloc(nmatches*sizeof(double)))==NULL){
     fprintf(stderr, "Memory allocation request failed in homest_RMS_RMedS()\n");
     exit(1);
  }
  homest_matinvLU(H, H1, 3);
  homest_matinvLU(NULL, NULL, 0);

  for(i=0, sum=0.0; i<nmatches; ++i){
      /* transform inpts1[i] according to H^-1 */
      q1[0]=H1[0]*inpts1[i][0]+H1[1]*inpts1[i][1]+H1[2];
      q1[1]=H1[3]*inpts1[i][0]+H1[4]*inpts1[i][1]+H1[5];
      s    =H1[6]*inpts1[i][0]+H1[7]*inpts1[i][1]+H1[8];
      q1[0]/=s; q1[1]/=s;

      /* transform inpts0[i] according to H */
      q2[0]=H[0]*inpts0[i][0]+H[1]*inpts0[i][1]+H[2];
      q2[1]=H[3]*inpts0[i][0]+H[4]*inpts0[i][1]+H[5];
      s    =H[6]*inpts0[i][0]+H[7]*inpts0[i][1]+H[8];
      q2[0]/=s; q2[1]/=s;

      dist1=(q1[0]-inpts0[i][0])*(q1[0]-inpts0[i][0]) + (q1[1]-inpts0[i][1])*(q1[1]-inpts0[i][1]);
      dist2=(q2[0]-inpts1[i][0])*(q2[0]-inpts1[i][0]) + (q2[1]-inpts1[i][1])*(q2[1]-inpts1[i][1]);
      sum+=errors[i]=(dist1+dist2)*0.5;
  }

  *rms=sqrt(sum/(double)(nmatches));
  *rmeds=sqrt(_MEDIAN_(errors, nmatches));

  free(errors);
}

/* compute H^-1 in H1, toggling the "direction" of a homography, i.e.
 * from img1 --> img2 to img2 --> img1 and vice versa
 */
HOMEST_API_MOD int HOMEST_CALL_CONV
homest_calcInv(double H[NUM_HPARAMS], double H1[NUM_HPARAMS])
{
int ret;

  ret=homest_matinvLU(H, H1, 3); // H1 <-- H^-1
  homest_matinvLU(NULL, NULL, 0);

#if 0
  /* normalize so that |H1|=1 */
  {
  register int i;
  double mag;

  for(i=0, mag=0.0; i<NUM_HPARAMS; ++i)
    mag+=H1[i]*H1[i];
  mag=1.0/sqrt(mag);
  for(i=0; i<NUM_HPARAMS; ++i)
    H1[i]*=mag;

  }
#endif

  return (ret==1)? HOMEST_OK : HOMEST_ERR;
}

/* compute H^-T in H1T, converting a point homography to a line one and vice versa */
HOMEST_API_MOD int HOMEST_CALL_CONV
homest_calcInvTrans(double H[NUM_HPARAMS], double H1T[NUM_HPARAMS])
{
int ret;
double tmp[NUM_HPARAMS];

  ret=homest_matinvLU(H, tmp, 3); // tmp <-- H^-1
  homest_matinvLU(NULL, NULL, 0);

#if 0
  /* normalize so that |H^-1|=1 */
  {
  register int i;
  double mag;

  for(i=0, mag=0.0; i<NUM_HPARAMS; ++i)
    mag+=tmp[i]*tmp[i];
  mag=1.0/sqrt(mag);
  for(i=0; i<NUM_HPARAMS; ++i)
    tmp[i]*=mag;

  }
#endif

  /* transpose */
  H1T[0]=tmp[0]; H1T[1]=tmp[3]; H1T[2]=tmp[6];
  H1T[3]=tmp[1]; H1T[4]=tmp[4]; H1T[5]=tmp[7];
  H1T[6]=tmp[2]; H1T[7]=tmp[5]; H1T[8]=tmp[8];

  return (ret==1)? HOMEST_OK : HOMEST_ERR;
}

/*************************************** Affine homography estimation ***************************************/


/* estimate "point" affine homography H s.t. p'=H*p, with p, p' specified by ptsidx */
static int est2DPtAffHomog(double *ah, int npts, int *ptsidx)
{
register int i, j;
double *mat, *coefs, *b;
register double *matrow, *bp;
static int (*const solver)(double *, double *, int, int, double *)=
                    homest_min_AxbSVD;
                    //homest_min_AxbQR; /* using QR is faster */
  
  if(ah==NULL){ /* cleanup */
    (*solver)(NULL, NULL, 0, 0, NULL);

    /* free working memory */
    free(globs.M);
    globs.M=globs.b=NULL; globs.Mrows=0;
    return 0;
  }

  if(globs.Mrows!=2*npts){ /* free existing matrix and allocate a new one */
    if(globs.M) /* is there anything to free? */
      free(globs.M);

    if((mat=(double *)malloc(2*npts*(NUM_HAFFPARAMS+1)*sizeof(double)))==NULL){
      fprintf(stderr, "Memory allocation request failed in est2DPtAffHomog()\n");
      exit(1);
    }
    globs.M=mat;
    globs.b=mat+2*npts*NUM_HAFFPARAMS;
    globs.Mrows=2*npts;
  }

  mat=globs.M;
  b=globs.b;

  /* fill in constraints matrix row by row */
  for(i=0, matrow=mat, bp=b; i<npts; ++i, matrow+=2*NUM_HAFFPARAMS){
    coefs=globs.cfs[ptsidx[i]];
    for(j=0; j<2*NUM_HAFFPARAMS; ++j) // 2 rows for each point pair
      matrow[j]=coefs[j];

    /* 2 right hand sides */
    j=ptsidx[i]<<1; // 2*ptsidx[i]
    *bp++=globs.rhs[j]; // b[2*i]
    *bp++=globs.rhs[j+1]; // b[2*i+1]
  }

  /* solve min |mat*h - b| */
  if((*solver)(mat, b, 2*npts, NUM_HAFFPARAMS, ah)!=1)
    return 0;

  return 1;
}

/* compute the (squared) algebraic residuals corresponding to ahomog in the second image */
static void affhomogResidualsAlg(double *ahomog, int numres, double *resid)
{
register int i, j;
double *cf1, *cf2, *rhs, sum1, sum2;

  for(i=0; i<numres; ++i){
    cf1=globs.cfs[i];
    cf2=cf1+NUM_HAFFPARAMS;
    rhs=globs.rhs+(i<<1); // 2*i
    sum1=-rhs[0];
    sum2=-rhs[1];
    for(j=0; j<NUM_HAFFPARAMS; ++j){
      sum1+=cf1[j]*ahomog[j];
      sum2+=cf2[j]*ahomog[j];
    }
    resid[i]=sum1*sum1 + sum2*sum2;
  }
}

/* robust, non-linear affine homography estimation from "nmatches" matched point features, possibly
 * including outliers. "inpts0", "inpts1" contain the matched image point coordinates, "inlPcent"
 * is the expected percentage of inliers (>=0.5), "H01" contains the estimated affine homography upon
 * return, "normalize" is a flag specifing whether input coordinates should be normalized according
 * to Hartley's suggestion, "idxOutliers" points to sufficiently large memory which upon return is
 * set to the indices of the detected outlying points (pass NULL if don't care), "nbOutliers" contains
 * the number of outliers, "verbose" specifies the verbosity level. A non-linear refinement step is
 * not necessary since for affine homographies, the algebraic error equals the geometric one
 *
 * Note that H01 is a 3x3 matrix, whose last row is fixed to [0, 0, 1]
 */
HOMEST_API_MOD int HOMEST_CALL_CONV
homestaff(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
              int normalize, int *idxOutliers, int *nbOutliers, int verbose)
{
register int i, j;
int isSqr=1, maxNbSol=1;
double gate=2.0, premResid=-1.0, sampleProb=0.99, outlierThresh;
int *outliersMap, ret, **sets=NULL, nbSets=0;
double (*pts0)[2], (*pts1)[2];
double L0[9], L1[9];

  /* the normalization transformations have a third row of [0 0 1],
   * thus they are also applicable to the affine case
   */
  if(normalize){
    pts0=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    pts1=(double (*)[2])malloc(nmatches*sizeof(double[2]));
    if(!pts0 || !pts1){
      fprintf(stderr, "Memory allocation request failed in homestaff()\n");
      exit(1);
    }
    homest_normalizePts(inpts0, pts0, nmatches, L0);
    homest_normalizePts(inpts1, pts1, nmatches, L1);
  }
  else{
    pts0=inpts0;
    pts1=inpts1;
  }

  globs.cfs=(double **)malloc(nmatches*sizeof(double *));
  if(!globs.cfs){
    fprintf(stderr, "Error: not enough memory for 'globs.cfs' in homestaff()\n");
    exit(1);
  }

  /* one "big" malloc instead of several "small" ones */
  globs.cfs[0]=(double *)malloc(2*nmatches*(NUM_HAFFPARAMS + 1)*sizeof(double)); /* two equations per point + RHS */
  if(!globs.cfs[0]){
    fprintf(stderr, "Error: not enough memory for 'globs.cfs[0]' in homestaff()\n");
    exit(1);
  }
	for(i=1; i<nmatches; ++i)
    globs.cfs[i]=globs.cfs[i-1] + 2*NUM_HAFFPARAMS;
  globs.rhs=globs.cfs[0]+2*nmatches*NUM_HAFFPARAMS;

	for(i=0; i<nmatches; ++i)
    calc2DAffHomogLinCoeffs2(pts0[i], pts1[i], globs.cfs[i], globs.rhs+2*i);

  nbSets=lqs_numtries(MIN_HAFFMATCHED_PTS, inlPcent, sampleProb);
  sets=lqs_allocsets(MIN_HAFFMATCHED_PTS, nbSets);

#ifdef USE_BUCKETS
  nbSets=homest_genRandomSetsWithBuckets(pts1, MIN_HAFFMATCHED_PTS, nmatches, nbSets, sets);
#else
  nbSets=homest_genRandomSetsNoBuckets(MIN_HAFFMATCHED_PTS, nmatches, nbSets, sets);
#endif /* USE_BUCKETS */

  globs.pts0=pts0; globs.pts1=pts1;
  globs.M=NULL; globs.Mrows=0;

  if(!(outliersMap=(int *)malloc(nmatches*sizeof(int)))){
    fprintf(stderr, "Error: not enough memory for 'outliersMap' in homestaff()\n");
    exit(1);
  }
  verbose=verbose>1;
  j=lqsfit(nmatches, MIN_HAFFMATCHED_PTS, sets, nbSets, affhomogResidualsAlg, est2DPtAffHomog,
            isSqr, verbose, maxNbSol, gate, premResid, NUM_HAFFPARAMS, inlPcent, H01,
            NULL, outliersMap, nbOutliers, &outlierThresh);

  if(verbose){
    fprintf(stderr, "Outlier threshold: %g\n", outlierThresh);
    fprintf(stderr, "homestaff(): LQS fit returned %d, %d outliers [out of %d]\n", j, *nbOutliers, nmatches);
  }

  if(sets) lqs_freesets(sets);

  if(j!=0){
    globs.inliersidx=(int *)malloc((nmatches - *nbOutliers)*sizeof(int));
    if(!globs.inliersidx){
      fprintf(stderr, "Error: not enough memory for 'globs.inliersidx' in homestaff()\n");
      exit(1);
    }

    for(i=j=0; i<nmatches; ++i)
      if(!outliersMap[i]) globs.inliersidx[j++]=i;

    /* LS estimation on inliers */
    globs.numInliers=nmatches - *nbOutliers;
    est2DPtAffHomog(H01, globs.numInliers, globs.inliersidx);

    /* complete fixed last row */
    H01[6]=H01[7]=0.0; H01[8]=1.0;

    /* expose outliers */
    if(idxOutliers!=NULL)
      for(i=j=0; i<nmatches; ++i)
        if(outliersMap[i]) idxOutliers[j++]=i;

    ret=HOMEST_OK;

#if 0
    /* include the following code fragment to print the matching point pairs found to be inlying */
    for(i=0; i<globs.numInliers; ++i){
      printf("%.4lf %.4lf  %.4lf %.4lf\n", inpts0[globs.inliersidx[i]][0], inpts0[globs.inliersidx[i]][1],
                                          inpts1[globs.inliersidx[i]][0], inpts1[globs.inliersidx[i]][1]);
    }
#endif

  }
  else{ /* LQS failed */
    dblzero_(H01, NUM_HPARAMS);
    *nbOutliers=nmatches;
    globs.inliersidx=NULL;
    ret=HOMEST_ERR;
  }

  est2DPtAffHomog(NULL, 0, NULL); /* release memory */

  if(normalize){
    free(pts0);
    free(pts1);
    homest_denormalizeH(H01, L0, L1, H01);
  }

  /* the DLT estimate has now been computed */

  /* just in case ... */
  globs.pts0=globs.pts1=NULL;
  globs.numInliers=0;

  if(globs.inliersidx) free(globs.inliersidx);
  free(globs.cfs[0]);
	free(globs.cfs);
  free(outliersMap);

  globs.cfs=NULL;
  globs.rhs=NULL;
  globs.inliersidx=NULL;

  return ret;
}
