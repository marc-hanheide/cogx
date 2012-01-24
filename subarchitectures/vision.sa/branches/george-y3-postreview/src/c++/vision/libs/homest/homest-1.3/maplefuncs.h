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

#ifndef _MAPLE_FUNCS_H
#define _MAPLE_FUNCS_H

/* calc_2Dhomog_coeffs.c */
extern void calc2DHomogLinCoeffs2(double m1[2], double m2[2], double eq[18]);
extern void calc2DHomogLinCoeffs3(double n1[3], double n2[3], double eq[27]);
extern void calc2DHomogSymTransfer(double m1[2], double m2[2], double h[9], double xm12[4]);
extern void calc2DHomogSymTransferJac(double m1[2], double m2[2], double h[9], double xm12_jac[4][9]);
extern void calc2DHomogSampsonErr(double m1[2], double m2[2], double h[9], double *err);
extern void calc2DHomogSampsonErrGrads(double m1[2], double m2[2], double h[9], double grads[9]);
extern void calc2DHomogTransfer(double m1[2], double h[9], double xm1[4]);
extern void calc2DHomogTransferJac(double m1[2], double h[9], double xm1_jac[2][9]);
extern void calc2DHomogReprojection(double h[9], double pm1[2], double pm12[4]);
extern void calc2DHomogReprojectionJac(double h[9], double pm1[2], double hgrad2[9], double hgrad3[9],
                                        double pm1grad0[2], double pm1grad1[2], double pm1grad2[2], double pm1grad3[2]);

extern void calc2DAffHomogLinCoeffs2(double m1[2], double m2[2], double eq[12], double rh[2]);

#endif /* _MAPLE_FUNCS_H */
