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

/******************************************************************************** 
 * homest demo. The program accepts a text file containing quadruples of matching
 * point coordinates (i.e., x1 y1  x2 y2 where x1 y1 is a (corner) point in the
 * 1st image and x2 y2 its corresponding one in the 2nd) and estimates the
 * homography mapping points in the first image to points in the second
 ********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "homest.h"

#define MAXSTRLEN	256

/* read pairs of matching points from a file */
static int readMatching2DPoints(char *fname, double (**pts0)[2], double (**pts1)[2])
{
register int i;
int ncoords, nmatches;
double coords[4];
FILE *fp;
char buf[MAXSTRLEN];

  if((fp=fopen(fname, "r"))==NULL){
    fprintf(stderr, "cannot open file %s\n", fname);
    exit(1);
  }

  fgets(buf, MAXSTRLEN, fp);
  if(ferror(fp)){
    fprintf(stderr, "File %s: error reading first line\n", fname);
    exit(1);
  }

  ncoords=sscanf(buf, "%lf%lf%lf%lf", coords, coords+1, coords+2, coords+3);
  if(ncoords==4){ /* no lines number */
    for(nmatches=1; !feof(fp); nmatches++){
      fscanf(fp, "%*g%*g%*g%*g\n");
      if(ferror(fp)){
        fprintf(stderr, "File %s: error reading 2D point coordinates, line %d\n", fname, nmatches + 1);
        exit(1);
      }
    }

    rewind(fp);
  }
  else{
    sscanf(buf, "%d", &nmatches);
  }

  *pts0=(double (*)[2])malloc(nmatches*sizeof(double[2]));
  *pts1=(double (*)[2])malloc(nmatches*sizeof(double[2]));
  if(!pts0 || !pts1){
    fprintf(stderr, "Memory allocation request failed in readMatching2DPoints()\n");
    exit(1);
  }

  /* read in points and store them */
  for(i=0; !feof(fp); i++){
    ncoords=fscanf(fp, "%lf%lf%lf%lf\n", (*pts0)[i], (*pts0)[i]+1, (*pts1)[i], (*pts1)[i]+1);
    if(ncoords==EOF) break;

    if(ncoords!=4){
      fprintf(stderr, "File %s: line %d contains only %d coordinates\n", fname, i + 1, ncoords);
      exit(1);
    }

    if(ferror(fp)){
      fprintf(stderr, "File %s: error reading 2D point coordinates, line %d\n", fname, i + 1);
      exit(1);
    }

  }
  fclose(fp);

  if(i!=nmatches){
    fprintf(stderr, "number of actuall points in file %s does not agree with that in first line (%d != %d)!\n",
                     fname, i, nmatches);
    exit(1);
  }

  return nmatches;
}

#define INL_PCENT 0.7

int main(int argc, char *argv[])
{
double (*pts0)[2], (*pts1)[2];
register int i;
int npts, donorm, noutl, *outidx=NULL;
char *matchesfile;
double H[NUM_HPARAMS], rms, rmeds;
int estAffine=0;

clock_t start_time, end_time;

  /* arguments parsing */
  if(argc!=2 && argc!=3){
    fprintf(stderr, "Usage: %s [-a] <matched points>\n", argv[0]);
    exit(1);
  }

  if(argc==3)
    if(!strcmp(argv[1], "-a")){
      estAffine=1;
      matchesfile=argv[2];
    }
    else{
      fprintf(stderr, "%s: unknown argument \"%s\" specified!\n", argv[0], argv[1]);
      exit(1);
    }
  else
    matchesfile=argv[1];

  npts=readMatching2DPoints(matchesfile, &pts0, &pts1);

#if 0
  for(i=0; i<npts; ++i){
    printf("%g %g  %g %g\n", pts0[i][0], pts0[i][1], pts1[i][0], pts1[i][1]);
  }
#endif

#ifdef NEED_OUTLIERS
  if((outidx=(int *)malloc(npts*sizeof(int)))==NULL){
    fprintf(stderr, "Memory allocation request failed in main()\n");
    exit(1);
  }
#endif /* NEED_OUTLIERS */

  donorm=1;

  fprintf(stdout, "%somography estimation using %d image matches\n", estAffine? "Affine h" : "H", npts);

  start_time=clock();
  if(!estAffine){
    int cstfunc;

    cstfunc=HOMEST_SYM_XFER_ERROR; // use the symmetric transfer error
    //cstfunc=HOMEST_XFER_ERROR; // use the transfer error in 2nd image
    //cstfunc=HOMEST_SAMPSON_ERROR; //use the Sampson error
    //cstfunc=HOMEST_REPR_ERROR; // use the reprojection error
    //cstfunc=HOMEST_NO_NLN_REFINE; // no refinement
    homest(pts0, pts1, npts, INL_PCENT, H, donorm, cstfunc, outidx, &noutl, 1);
  }
  else
    homestaff(pts0, pts1, npts, INL_PCENT, H, donorm, outidx, &noutl, 1);
  end_time=clock();

  fprintf(stdout, "\nEstimated %shomography [%d outliers, %.2lf%%]\n", estAffine? "affine " : "", noutl, (double)(100.0*noutl)/npts);
  for(i=0; i<NUM_HPARAMS; ++i){
    if(!(i%3)) fprintf(stdout, "\n");
    fprintf(stdout, "%.7g ", H[i]);
  }
  fprintf(stdout, "\n");
  fprintf(stdout, "\nElapsed time: %.2lf seconds, %.2lf msecs\n", ((double) (end_time - start_time)) / CLOCKS_PER_SEC,
                        ((double) (end_time - start_time)) / (CLOCKS_PER_SEC/1000.0));

  homest_RMS_RMedS(pts0, pts1, npts, H, &rms, &rmeds);
  fprintf(stdout, "\nHomography RMS and RMedS errors for input points: %g %g\n", rms, rmeds);
  fflush(stdout);

#ifdef NEED_OUTLIERS
  fprintf(stdout, "Indices of the %d outlier pairs:\n", noutl);
  for(i=0; i<noutl; ++i)
    fprintf(stdout, "%d ", outidx[i]);
  fputc('\n', stdout);

  if(outidx) free(outidx);
#endif /* NEED_OUTLIERS */

  free(pts0);
  free(pts1);

  return 0;
}
