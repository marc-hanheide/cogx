/*
/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-08  Manolis Lourakis (lourakis **at** ics.forth.gr)
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
*/

#ifndef _HOMEST_H
#define _HOMEST_H

/* define the following if you want to build a DLL with MSVC */
/**
#define DLL_BUILD 
**/

#ifdef __cplusplus
extern "C" {
#endif

#define HOMEST_VERSION    "1.3 (Jul. 2009)"

#define NUM_HPARAMS       9

/* non-linear refinement cost functions */
#define HOMEST_NO_NLN_REFINE    0 /* no non-linear refinement */
#define HOMEST_XFER_ERROR       1 /* non-linear refinement using 2nd image homographic transfer error (non-symmetric) */
#define HOMEST_SYM_XFER_ERROR   2 /* non-linear refinement using symmetric homographic transfer error */
#define HOMEST_SAMPSON_ERROR    3 /* non-linear refinement using Sampson error */
#define HOMEST_REPR_ERROR       4 /* non-linear refinement using reprojection error */
#define HOMEST_XFER_ERROR1      5 /* non-linear refinement using 1st image inverse homographic transfer error (non-symmetric) */
#define HOMEST_XFER_ERROR2      HOMEST_XFER_ERROR /* alias */

#if defined(DLL_BUILD) && defined(_MSC_VER) /* build DLLs with MSVC only! */
#define HOMEST_API_MOD    __declspec(dllexport)
#define HOMEST_CALL_CONV  __cdecl
#else /* define empty */
#define HOMEST_API_MOD 
#define HOMEST_CALL_CONV
#endif /* DLL_BUILD && _MSC_VER */

#define HOMEST_ERR     -1
#define HOMEST_OK       0

/* homest.c */
/* fully projective H */
extern HOMEST_API_MOD int HOMEST_CALL_CONV
  homest(double (*pts0)[2], double (*pts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
          int normalize, int NLrefine, int *idxOutliers, int *nbOutliers, int verbose);

/* affine H */
extern HOMEST_API_MOD int HOMEST_CALL_CONV
  homestaff(double (*pts0)[2], double (*pts1)[2], int nmatches, double inlPcent, double H01[NUM_HPARAMS],
          int normalize, int *idxOutliers, int *nbOutliers, int verbose);

/* RMS & RMedS errors for a H */
extern HOMEST_API_MOD void HOMEST_CALL_CONV
  homest_RMS_RMedS(double (*inpts0)[2], double (*inpts1)[2], int nmatches, double H[NUM_HPARAMS],
                    double *rms, double *rmeds);

/* H^-1 */
extern HOMEST_API_MOD int HOMEST_CALL_CONV
  homest_calcInv(double H[NUM_HPARAMS], double H1[NUM_HPARAMS]);

/* H^-t */
extern HOMEST_API_MOD int HOMEST_CALL_CONV
  homest_calcInvTrans(double H[NUM_HPARAMS], double H1T[NUM_HPARAMS]);

#ifdef __cplusplus
}
#endif

#endif /* _HOMEST_H */
