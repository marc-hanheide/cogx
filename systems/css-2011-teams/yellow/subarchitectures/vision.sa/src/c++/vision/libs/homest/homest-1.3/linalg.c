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
#include <float.h>

#include "compiler.h"
#include "util.h"

/* lapack functions prototypes */

/* eigenvalues & eigenvectors */
extern int F77_FUNC(dspev)(char *jobz, char *uplo, int *n, double *ap, 
                            double *w, double *z, int *ldz, double *work, int *info);

/* SVD */
extern int F77_FUNC(dgesvd)(char *jobu, char *jobvt, int *m, int *n,
                            double *a, int *lda, double *s, double *u, int *ldu,
                            double *vt, int *ldvt, double *work, int *lwork, int *info);

/* lapack 3.0 routine, faster than dgesvd() */
extern int F77_FUNC(dgesdd)(char *jobz, int *m, int *n, double *a, int *lda,
                            double *s, double *u, int *ldu, double *vt, int *ldvt,
                            double *work, int *lwork, int *iwork, int *info);

/* LU decomposition, linear system solution and matrix inversion */
extern int F77_FUNC(dgetrf)(int *m, int *n, double *a, int *lda, int *ipiv, int *info); /* blocked LU */
extern int F77_FUNC(dgetf2)(int *m, int *n, double *a, int *lda, int *ipiv, int *info); /* unblocked LU */
extern int F77_FUNC(dgetri)(int *n, double *a, int *lda, int *ipiv, double *work, int *lwork, int *info);

/* solution of overdetermined systems */
/* SVD */
extern int F77_FUNC(dgelss)(int *m, int *n, int *nrhs, 
	double *a, int *lda, double *b, int *ldb, double *s, 
	double *rcond, int *rank, double *work, int *lwork,
	int *info);

/* QR */
extern int F77_FUNC(dgels)(char *trans, int *m, int *n, int *nrhs,
    double *a, int *lda, double *b, int *ldb,
    double *work, int *lwork, int *info);

/* Solve min |Ax| subject to |x|=1
 * The solution is the eigenvector of A^T A corresponding
 * to the smallest eigenvalue.
 *
 * A is mxn, x is nx1
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int homest_min_Ax_normEIG(double *A, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

register int i, j, k;
double *triang, *eigvals, *eigvecs, *work, thresh;
register double sum;
int info, tot_sz, triang_sz, eigvals_sz, eigvecs_sz, work_sz;

  if(!A){
    if(buf) free(buf);
    buf=NULL;
    buf_sz=0;
    return 1;
  }

  /* calculate required memory size */
  triang_sz=(n*(n+1))>>1; //n*(n+1)/2;
  eigvals_sz=n;
  eigvecs_sz=n*n;
  work_sz=3*n;
  tot_sz=triang_sz+eigvals_sz+eigvecs_sz+work_sz;

  if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
    if(buf) free(buf); /* free previously allocated memory */

    buf_sz=tot_sz;
    buf=(double *)malloc(buf_sz*sizeof(double));
    if(!buf){
      fprintf(stderr, "Memory allocation request for %d doubles in homest_min_Ax_normEIG() failed!\n", buf_sz);
      exit(1);
    }
  }

  triang=buf;
  eigvals=triang+triang_sz;
  eigvecs=eigvals+eigvals_sz;
  work=eigvecs+eigvecs_sz;

  /* triang = A^t * A */
  for(j=0; j<n; j++)
    for(i=0; i<=j; i++){
      for(k=0, sum=0.0; k<m; k++)
        sum+=A[k*n+i]*A[k*n+j];
      triang[i + ((j*(j+1))>> 1)]=sum; //i + j*(j+1)/2
    }

  F77_FUNC(dspev)("V", "U", &n, triang, eigvals, eigvecs, &n, work, &info);

  if(info<0){
    fprintf(stderr, "LAPACK error: illegal value for argument %d of dspev in homest_min_Ax_normEIG()\n", -info);
    exit(1);
  }
  else if(info>0){
    fprintf(stderr, "LAPACK error: dspev failed to converge in homest_min_Ax_normEIG();\n%d %s", info,
        "off-diagonal elements of an intermediate tridiagonal form did not converge to zero\n");
    return 0;
  }

  for(i=n-1, j=0, thresh=eigvals[n-1]*DBL_EPSILON; i>0; --i, ++j)
    if(eigvals[i]<=thresh) break; /* remaining eigenvalues are smaller than this */


  if(j!=n-1){ /* matrix is not of rank n-1! */
    //fprintf(stderr, "Unacceptable rank %d in homest_min_Ax_normEIG\n", rank);
    return 0;
  }

  /* min eigenvalue is the first (they are returned in ascending order), return the first eigenvector */
  for(i=0; i<n; i++)
    x[i]=eigvecs[i];

  return 1;
}


/* Solve min |Ax| subject to |x|=1
 * The solution is the right singular vector (Vt) corresponding to A's
 * minimum singular value: A=U*D*V^T ==> A^T*A=V*D^2*V^T
 *
 * A is mxn, x is nx1
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int homest_min_Ax_normSVD(double *A, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

register int i, j;
double *a, *u, *s, *vt, *work, thresh;
int info, worksz, *iwork, iworksz;
int a_sz, u_sz, s_sz, vt_sz, tot_sz;

  if(!A){
    if(buf) free(buf);
    buf=NULL;
    buf_sz=0;
    return 1;
  }

  /* calculate required memory size. Note that if dgesvd is used, the memory for u is not actually needed... */
  worksz=-1; // workspace query. Keep in mind that dgesdd requires more memory than dgesvd
  /* note that optimal work size is returned in thresh */
  //F77_FUNC(dgesdd)("A", (int *)&m, (int *)&n, NULL, (int *)&m, NULL, NULL, (int *)&m, NULL, (int *)&n, (double *)&thresh, (int *)&worksz, NULL, &info);
  F77_FUNC(dgesvd)("N", "A", (int *)&m, (int *)&n, NULL, (int *)&m, NULL, NULL, (int *)&m, NULL, (int *)&n, (double *)&thresh, (int *)&worksz, &info);
  worksz=(int)thresh;
  iworksz=8*n;
  a_sz=m*n;
  u_sz=m*m; s_sz=n; vt_sz=n*n;

  tot_sz=(a_sz + u_sz + s_sz + vt_sz + worksz)*sizeof(double) + iworksz*sizeof(int); /* should be arranged in that order for proper doubles alignment */

  if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
    if(buf) free(buf); /* free previously allocated memory */

    buf_sz=tot_sz;
    buf=(double *)malloc(buf_sz);
    if(!buf){
      fprintf(stderr, "Memory allocation request for %d bytes in homest_min_Ax_normSVD() failed!\n", buf_sz);
      exit(1);
    }
  }

  a=buf;
  u=a+a_sz;
  s=u+u_sz;
  vt=s+s_sz;
  work=vt+vt_sz;
  iwork=(int *)(work+worksz);

  /* store A (column major!) into a */
  for(i=0; i<m; i++)
    for(j=0; j<n; j++)
      a[i+j*m]=A[i*n+j];

  /* SVD decomposition of A */
  //F77_FUNC(dgesdd)("A", (int *)&m, (int *)&n, a, (int *)&m, s, u, (int *)&m, vt, (int *)&n, work, (int *)&worksz, iwork, &info);
  F77_FUNC(dgesvd)("N", "A", (int *)&m, (int *)&n, a, (int *)&m, s, NULL, (int *)&m, vt, (int *)&n, work, (int *)&worksz, &info);

  /* error treatment */
  if(info!=0){
    if(info<0){
      fprintf(stderr, "LAPACK error: illegal value for argument %d of dgesdd in homest_min_Ax_normSVD()\n", -info);
      exit(1);
    }
    else{
      fprintf(stderr, "LAPACK error: dgesdd (dbdsdc)/dgesvd (dbdsqr) failed to converge in homest_min_Ax_normSVD() [info=%d]\n", info);
      return 0;
    }
  }

  /* determine A's rank */
  for(i=0, thresh=DBL_EPSILON*s[0]; i<n; ++i)
    if(s[i]<=thresh) break; /* remaining singular values are smaller than this */

  if(i<n-1) 
    return 0; /* A should have rank n-1 */

  /* s[n-1] is the smallest singular value */
  vt+=n-1;
  for(j=0; j<n; ++j)
    x[j]=vt[j*n]; //vt[n-1+j*n];

  return 1;
}

/*
 * This function computes the inverse of a square matrix A into B
 * using LU decomposition
 *
 * The function returns 0 in case of error (e.g. A is singular),
 * 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int homest_matinvLU(double *A, double *B, int m)
{
static double *buf=NULL;
static int buf_sz=0, nb=0;

int a_sz, ipiv_sz, work_sz, tot_sz;
register int i, j;
int info, *ipiv;
double *a, *work;
   
  if(A==NULL){
    if(buf) free(buf);
    buf=NULL;
    buf_sz=0;

    return 1;
  }

  /* calculate required memory size */
  ipiv_sz=m;
  a_sz=m*m;
  if(!nb){
    double tmp;

    work_sz=-1; // workspace query; optimal size is returned in tmp
    F77_FUNC(dgetri)((int *)&m, NULL, (int *)&m, NULL, (double *)&tmp, (int *)&work_sz, (int *)&info);
    nb=((int)tmp)/m; // optimal worksize is m*nb
  }
  work_sz=nb*m;
  tot_sz=(a_sz + work_sz)*sizeof(double) + ipiv_sz*sizeof(int); /* should be arranged in that order for proper doubles alignment */

  if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
    if(buf) free(buf); /* free previously allocated memory */

    buf_sz=tot_sz;
    buf=(double *)malloc(buf_sz);
    if(!buf){
      fprintf(stderr, "memory allocation in homest_matinvLU() failed!\n");
      exit(1);
    }
  }

  a=buf;
  work=a+a_sz;
  ipiv=(int *)(work+work_sz);

  /* store A (column major!) into a */
	for(i=0; i<m; ++i)
		for(j=0; j<m; ++j)
			a[i+j*m]=A[i*m+j];

  /* LU decomposition for A */
	//F77_FUNC(dgetrf)((int *)&m, (int *)&m, a, (int *)&m, ipiv, (int *)&info);  
	F77_FUNC(dgetf2)((int *)&m, (int *)&m, a, (int *)&m, ipiv, (int *)&info);  
	if(info!=0){
		if(info<0){
			fprintf(stderr, "argument %d of dgetf2/dgetrf illegal in homest_matinvLU()\n", -info);
			exit(1);
		}
		else{
			fprintf(stderr, "singular matrix A for dgetf2/dgetrf in homest_matinvLU()\n");
			return 0;
		}
	}

  /* (A)^{-1} from LU */
	F77_FUNC(dgetri)((int *)&m, a, (int *)&m, ipiv, work, (int *)&work_sz, (int *)&info);
	if(info!=0){
		if(info<0){
			fprintf(stderr, "argument %d of dgetri illegal in homest_matinvLU()\n", -info);
			exit(1);
		}
		else{
			fprintf(stderr, "singular matrix A for dgetri in homest_matinvLU()\n");
			return 0;
		}
	}

	/* store (A)^{-1} in B */
	for(i=0; i<m; ++i)
		for(j=0; j<m; ++j)
      B[i*m+j]=a[i+j*m];

	return 1;
}




/* NOTE: the current implementation of SVD LS relies on routine dgelss; for large problems
 * it is preferable to use dgelsd which uses a divide and conquer approach for computing
 * the SVD
 */

/* minimum norm solutions of linear least squares systems */

/*
 * This function computes the solution of min_x ||Ax - b|| where
 * || . || is the second order norm. It is a least squares technique
 * based on SVD
 *
 * In terms of stability (i.e. condition number), this SVD-based method
 * is preferable to that involving the normal equations which computes x
 * as (A^TA)^(-1)A^Tb. Note that b is assumed to be nonzero; in the
 * opposite case x should be computed with another function as the
 * eigenvector of A^T A corresponding to the smallest eigenvalue.
 *
 * A is mxn, x nx1 and b mx1 
 *
 * The function returns 0 in case of error, -1 if input matrix is rank
 * deficient and 1 otherwise. Note that a solution is still computed in
 * the rank deficient case
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int homest_min_AxbSVD(double *A, double *B, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

int a_sz, b_sz, s_sz, work_sz, tot_sz;
register int i, j;
int info, rank, nrhs=1;
double *a, *b, *s, *work;
double aux, rcond=-1.0; // use machine precision
//double condnum;

    if(!A){
      if(buf) free(buf);
        buf=NULL;
        buf_sz=0;
        return 1;
    }

    if(n>m){
      fprintf(stderr, "Fewer equations than unknowns in homest_min_AxbSVD()!\n");
      exit(1);
    }

    /* calculate required memory size */
    a_sz=m*n;
    b_sz=m;
    s_sz=n;
    work_sz=-1; // workspace query
    /* optimal work size is returned in aux */
    F77_FUNC(dgelss)(&m, &n, &nrhs, NULL, &m, NULL, &m, NULL, NULL, NULL, &aux, &work_sz, &info);
    work_sz=(int)aux;
    tot_sz=a_sz + b_sz + s_sz + work_sz;

    if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
      if(buf) free(buf); /* free previously allocated memory */

      buf_sz=tot_sz;
      buf=(double *)malloc(buf_sz*sizeof(double));
      if(!buf){
        fprintf(stderr, "memory allocation in homest_min_AxbSVD() failed!\n");
        exit(1);
      }
    }

    a=buf;
    b=a+a_sz;
    s=b+b_sz;
    work=s+s_sz;

    /* copy the matrix A into a, vector B into b */
    for(i=0; i<m; ++i){
      for(j=0; j<n; ++j)
        a[j*m+i]=A[i*n+j];

      b[i]=B[i];
    }
  
    /* call LAPACK */
    F77_FUNC(dgelss)(&m, &n, &nrhs, a, &m, b, &m, s, &rcond, &rank, work, &work_sz, &info);
    /* error treatment */
    if(info!=0){
		  if(info<0){
	      fprintf(stderr, "LAPACK error: illegal value for argument %d of dgelss in homest_min_AxbSVD()\n", -info);
		    exit(1);
		  }
		  else{
			  fprintf(stderr, "LAPACK error: dgelss failed to converge in homest_min_AxbSVD();\n%d %s", info,
				  "off-diagonal elements of an intermediate bidiagonal form did not converge to zero\n");
			  return 0;
		  }
    }
  
    /* copy solution to x */
    for(i=0; i<n; ++i)
      x[i]=b[i];

    /* condition number: ratio of largest singular value over the smallest  */
    //condnum=s[0]/s[n-1];
  
    if(n!=rank){
      /*
      fprintf(stderr, "Warning: input matrix in homest_min_AxbSVD() is not of full column rank! [%d != %d]\n", rank, n);
      */
      return -1;
    }
    else
      return 1;
}


/*
 * This function computes the solution of min_x ||Ax - b|| where
 * || . || is the second order norm. It is a least squares technique
 * based on QR decomposition: An orthogonal matrix preserves vector norm,
 * therefore ||Ax - b||=||Q^T Ax - Q^T b||=||Rx - Q^T b||. Thus, QR
 * decomposition allows the solution to be obtained by solving this last
 * triangular system
 *
 * In terms of stability (i.e. condition number), this QR-based method
 * is preferable to that involving the normal equations. Note that b
 * is assumed to be nonzero; in the opposite case x should be computed
 * with another function as the eigenvector of A^T A corresponding to
 * the smallest eigenvalue.
 *
 * A is mxn, x nx1 and b mx1 
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
int homest_min_AxbQR(double *A, double *B, int m, int n, double *x)
{
static double *buf=NULL;
static int buf_sz=0;

int a_sz, b_sz, work_sz, tot_sz;
register int i, j;
int info, nrhs=1;
double *a, *b, *work;
double aux;

    if(!A){
      if(buf) free(buf);
        buf=NULL;
        buf_sz=0;
        return 1;
    }

    if(n>m){
      fprintf(stderr, "Fewer equations than unknowns in homest_min_AxbQR()!\n");
      exit(1);
    }

    /* calculate required memory size */
    a_sz=m*n;
    b_sz=m;
    work_sz=-1; // workspace query
    /* optimal work size is returned in aux */
    F77_FUNC(dgels)("N", &m, &n, &nrhs, NULL, &m, NULL, &m, &aux, &work_sz, &info);
    work_sz=(int)aux;
    tot_sz=a_sz + b_sz + work_sz;

    if(tot_sz>buf_sz){ /* insufficient memory, allocate a "big" memory chunk at once */
      if(buf) free(buf); /* free previously allocated memory */

      buf_sz=tot_sz;
      buf=(double *)malloc(buf_sz*sizeof(double));
      if(!buf){
        fprintf(stderr, "memory allocation in homest_min_AxbQR() failed!\n");
        exit(1);
      }
    }

    a=buf;
    b=a+a_sz;
    work=b+b_sz;

    /* copy the matrix A into a, vector B into b */
    for(i=0; i<m; ++i){
      for(j=0; j<n; ++j)
        a[j*m+i]=A[i*n+j];

      b[i]=B[i];
    }
  
    /* call LAPACK */
    F77_FUNC(dgels)("N", &m, &n, &nrhs, a, &m, b, &m, work, &work_sz, &info);
    /* error treatment */
    if(info!=0){
		  if(info<0){
	      fprintf(stderr, "LAPACK error: illegal value for argument %d of dgels in homest_min_AxbQR()\n", -info);
		    exit(1);
		  }
		  else{
			  fprintf(stderr, "LAPACK error: unknown dgels error %d\n", info);
			  return 0;
		  }
    }

    /* copy solution to x */
    for(i=0; i<n; ++i)
      x[i]=b[i];

    return 1;
}
