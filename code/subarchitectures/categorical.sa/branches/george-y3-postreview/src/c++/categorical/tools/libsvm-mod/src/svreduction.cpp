/*
* This file implements the algorithm decreasing the number of support
* vectors by reducing those that are linearly depentent in feature space.
*
* References:    (TO BE UPDATED)
*   - T. Downs, K. E. Gates and A. Masters. Exact Simplification of
*     Support Vector Solutions. Journal of Machine Learning Research 2,
*     2001, 293-297
*   - C. D. Mayer. Matrix Analysis and Applied Linear Algebra. SIAM, 2000
*   - B. Noble, J. W. Daniel. Applied Linear Algebra. Third Edition. Prentice-Hall, 1988
*
* Author:
*   Andrzej Pronobis
*   pronobis@nada.kth.se
*   pronobis@o2.pl
*/

#include <limits.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "svm.h"


/* Uncomment the following line if you want additional debug messages. */
// #define DEBUG_MESSAGES

/* Malloc */
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

/* Memcpy */
#define Memcpy(dest, src, type, n) memcpy((dest), (src), (n)*sizeof(type))

/* Fill with zeros */
#define Memclear(ptr, type, n) memset((ptr), 0, (n)*sizeof(type))

/* One against all for original libsvm */
#ifndef ONE_AGAINST_ALL
  #define ONE_AGAINST_ALL 5
#endif

/*
* For testing purposes only.
*/
void printKSV(double **KSV, int *P, bool *LI, int nrSV, int l, int pRow=-1, int pCol=-1)
{
  // Pivots marked with *
  // Checked rows and l. i. vectors marked with +

  fprintf(stdout, "--------------------\n");
  fprintf(stdout,"KSV Matrix:\n\n");
  fprintf(stdout,"\t|");
  for (int j=0 ; j<nrSV ; ++j)
    if (LI[j])
      fprintf(stdout,"%2d +            |", j);
    else
      fprintf(stdout,"%2d              |", j);

  for (int i=0 ; i<l ; ++i)
  {
    if (P[i]<0)
      fprintf(stdout,"\n%d\t|", i);
    else
      fprintf(stdout,"\n%d +\t|", i);

    for (int j=0 ; j<nrSV ; ++j)
      if ((i==pRow)&&(j==pCol))
      {
        if (KSV[j][i]<0)
          fprintf(stdout,"*%15.3f|", KSV[j][i]);
        else
          fprintf(stdout,"*%15.3f|", KSV[j][i]);
      }
      else
        fprintf(stdout,"%16.3f|", KSV[j][i]);
  }

  fprintf(stdout,"\n--------------------\n\n");
  fflush(stdout);
} // printKSV



void printMatrix(double **M, int s1, int s2, char *title)
{
  // M is s1 x s2

  printf("--------------------\n");
  printf(title);
  printf(":\n\n");
  printf("  \t|");
  for (int j=0 ; j<s2 ; ++j)
      printf("%2d              |", j);

  for (int i=0 ; i<s1 ; ++i)
  {
    printf("\n%d\t|", i);

    for (int j=0 ; j<s2 ; ++j)
        printf("%16.10f|", M[j][i]);
  }

  printf("\n--------------------\n\n");
  fflush(stdout);
} // printKSV



void printVector(double *V, int s, char *title)
{
  printf("--------------------\n");
  printf(title);
  printf(":\n\n|");
  for (int j=0 ; j<s ; ++j)
      printf("%2d              |", j);

  printf("\n|");

  for (int j=0 ; j<s ; ++j)
      printf("%16.10f|", V[j]);

  printf("\n--------------------\n\n");
  fflush(stdout);
} // printKSV



void printVectorPtr(double **V, int s, char *title)
{
  printf("--------------------\n");
  printf(title);
  printf(":\n\n|");
  for (int j=0 ; j<s ; ++j)
      printf("%2d              |", j);

  printf("\n|");

  for (int j=0 ; j<s ; ++j)
      printf("%16.10f|", *V[j]);

  printf("\n--------------------\n\n");
  fflush(stdout);
} // printKSV



void printPivot(double pVal, int pRow, int pCol)
{
  fprintf(stdout,"--------------------\n");
  fprintf(stdout,"Pivot value: %f\n", pVal);
  fprintf(stdout,"Pivot row: %i\n", pRow);
  fprintf(stdout,"Pivot col: %i\n", pCol);
  fprintf(stdout,"--------------------\n\n");
  fflush(stdout);
} // printPivot

void printAlpha(double **alphaSV, int nrSV)
{
  fprintf(stdout,"--------------------\n");
  fprintf(stdout,"Alpha: \n");

  fprintf(stdout,"|");
  for (int j=0 ; j<nrSV ; ++j)
    fprintf(stdout,"%2d              |", j);

  fprintf(stdout,"\n|");
  for (int j=0 ; j<nrSV ; ++j)
    fprintf(stdout,"%16.3f|", (*alphaSV[j]));

  fprintf(stdout,"\n--------------------\n\n");
  fflush(stdout);
} // printAlpha



/*
* Function allocates memory and prepares input data.
* Parameters:
*   - x (in) - Array of all input vectors (Array of arrays of nodes)
*   - alpha (in) - Array of all alpha coefficients
*   - l (in) - Number of all input vectors in problem
*   - param (in) - Model parameters
*   - KSV (out) - Gram matrix for SVs only
*   - alphaSV (out) - Array of pointers to original SV alpha
*   - nrSV (out) - Number of SV
*/
void prepareData(svm_node **x, double *alpha, int l, const svm_parameter* param,
                 double **&KSV, double **&alphaSV, int &nrSV)
{
    // Count SV
    nrSV = 0;
    for (int i=0; i<l; ++i)
      if (fabs(alpha[i]) > 0)
        ++nrSV;

    if (nrSV==0) return;

    // Create matrix
    KSV = Malloc(double *, nrSV);
    alphaSV = Malloc(double *, nrSV);



    for (int i=0,j=0; j<nrSV; ++i)
    {
      if(fabs(alpha[i]) > 0)
      {
        alphaSV[j] = &alpha[i];
        KSV[j] = Malloc(double, l);

        for (int k=0; k<l; ++k )
        {
          KSV[j][k] = Kernel::k_function(x[i],x[k],*param);
          if (isnan(KSV[j][k])) KSV[j][k]=0.0;
        }

        ++j;
      } // if
    } // for

}


/*
* Function releases previously allocated memory.
* Parameters:
*   - KSV (in) - Gram matrix for SVs only
*   - alphaSV (in) - Array of pointers to original SVs alpha
*   - nrSV (in) - Number of SV
*/
void releaseMemory(double **&KSV, double **&alphaSV, int nrSV)
{
  if (KSV)
  {
    for (int i=0; i<nrSV; ++i)
      free(KSV[i]);
    free(KSV);
    KSV=0;
  }
  if (alphaSV)
  {
    free(alphaSV);
    alphaSV=0;
  }
}



/*
* Function releases previously allocated memory.
* Parameters:
*   - model (in) - Model
*   - problem (in) - Problem
*   - x_space (in) - X Space
*/
void releaseMemory(svm_model *&model, svm_problem *&problem, svm_node *&x_space)
{
  if (model)
  {
    svm_destroy_model(model);
    model=0;
  }
  if (problem)
  {
    free(problem->x);
    free(problem->y);
    free(problem);
    problem=0;
  }
  if (x_space)
  {
    free(x_space);
    x_space=0;
  }
}



/*
* Function checks which of SVs are linearly dependent in the feature space
* and updates alpha for all linearly dependent and independent SVs.
* Function uses the Gauss-Jordan Elimination algorithm with partial pivoting.
*   - KSV (in) - Gram matrix for SVs only
*   - alphaSV (in,out) - Array of pointers to original SV alpha
*   - nrSV (in) - Number of SV
*   - l (in) - Number of all input vectors
*   - param (in) - Model parameters
*/
int RLDSVcoreGJE(double **KSV, double **alphaSV, int nrSV, int l, const svm_parameter* param)
{
  // -------------------
  //  Allocating memory
  // -------------------

  // Pivot position in each row of SV matrix
  int *P = Malloc(int, l);
  for (int i=0 ; i<l ; ++i) P[i]=-1;
  // Indicates whether SV is linearly independent
  bool *LI = Malloc(bool, nrSV);
  memset(LI, 0, nrSV*sizeof(bool));


  // --------------
  //  Finding LDSV
  // --------------

  // Find next pivot
  double pVal;  // Pivot value
  int pCol = -1;  // Pivot position
  int pRow;

  // Outer loop (one pivot per loop)
  do
  {
    // Find pivot
    pRow = -1;
    pVal = param->RLDSVthreshold;
    for (int j=pCol+1 ; (pRow<0) && (j<nrSV) ; ++j)
    {
      for (int i=0 ; i<l ; ++i)
      {
        if (P[i] < 0)
        {
          if (fabs(KSV[j][i])>fabs(pVal))
          {
            pVal = KSV[j][i];
            pCol = j;
            pRow = i;
          } // if
        } // if
      } // for
    } // for


    // Debug info
    #ifdef DEBUG_MESSAGES
      printKSV(KSV, P, LI, nrSV, l, pRow, pCol);
      printPivot(pVal, pRow, pCol);
    #endif

    // If another pivot found
    if (pRow!=-1)
    {
      P[pRow] = pCol;   // Mark row as checked and remember the pivot column for that row
      LI[pCol] = true;   // Mark column (support vector) as linearly independent

      // Divide the pivotal row by the value of the pivot
      for (int j=pCol; j<nrSV; ++j)
        KSV[j][pRow]/=pVal;

      // Substract the pivotal row from other rows
      for (int i=0; i<l; ++i)
      {
        if (i!=pRow)
        {
          double reduceVal = KSV[pCol][i];
          KSV[pCol][i] = 0;
          for (int j=pCol+1; j<nrSV; ++j)
          {
            KSV[j][i]-=KSV[j][pRow]*reduceVal;
          }
        }
      } // for

      // Debug info
      #ifdef DEBUG_MESSAGES
        printKSV(KSV, P, LI, nrSV, l, pRow, pCol);
      #endif

    } // if

  } while ((pRow!=-1) && (pCol<nrSV));



  // ----------------
  //  Updating Alpha
  // ----------------

  // Debug info
  #ifdef DEBUG_MESSAGES
    printAlpha(alphaSV, nrSV);
  #endif

  // Check all rows
  for(int i=0; i<l; ++i)
  {
    int iSV = P[i];
    // Take into consideration only nonzero rows
    if (iSV!=-1)
    {
      double sum=0;  // SUM(j, j!=LI)(Alpha_j*y_j*c_ji)
      for(int j=iSV+1; j<nrSV; ++j)
        if (!LI[j])  // Linearly dependent only
          sum+=(*alphaSV[j])*KSV[j][i];

      (*alphaSV[iSV])+=sum;
    } // if
  } // for


  // Set all alpha belonging to lin. dep. SV to zero
  int nrSVReduced = 0;
  for (int j=0; j<nrSV; ++j)
    if (!LI[j])
    {
      (*alphaSV[j])=0;
      ++nrSVReduced;
    }

  // Debug info
  #ifdef DEBUG_MESSAGES
    printAlpha(alphaSV, nrSV);
  #endif


  // ------------------
  //  Releasing memory
  // ------------------
  if (P)
  {
    free(P);
    P=0;
  }
  if (LI)
  {
    free(LI);
    LI=0;
  }


  return nrSVReduced;

} // RLDSVcore




/*
* Function computes the Euclidean norm of a vector.
*/
double euclideanNorm(double *V, int start, int end)
{
  double result = 0.0;
  for (int i=start; i<=end; ++i)
  {
    result+=pow(V[i], 2.0);
  }
  return sqrt(result);
}


/*
* Function divides a vector by a given value.
*/
void divideVector(double *V, int start, int end, double value)
{
  for (int i=start; i<=end; ++i)
  {
    V[i]/=value;
  }
}


/*
* Function multiplies a vector by a given value.
*/
void multiplyVector(double *V, int start, int end, double value)
{
  for (int i=start; i<=end; ++i)
  {
    V[i]*=value;
  }
}


/*
* Function computes the dot product of two vectors.
*/
double dotProduct(double *V1, double *V2, int start, int end)
{
  double result = 0.0;
  for (int i=start; i<=end; ++i)
  {
    result+=V1[i]*V2[i];
  }
  return result;
}

/*
*  Function adds a vector multiplied by a scalar to another vector.
*/
void addMultipliedVector(double *V1, double *V2, double value, int start, int end)
{
  for (int i=start; i<=end; ++i)
  {
    V1[i]+=value*V2[i];
  }
}



/*
* Function computes the maximum absolute column sum norm of an upper
* triangular matrix (||M||1).
* The dimensionality of the matrix is: (end1-start1)x(end2-start2).
*/
double maxAbsColSumNormUT(double **M, int start, int end)
{
  double result = 0.0;
  double sum = 0.0;
  double *curCol;

  for (int j=start; j<=end; ++j)
  {
    curCol = M[j];
    sum = 0.0;
    for (int i=start; i<=j; ++i)
      sum+=fabs(curCol[i]);
    if (sum>result)
      result=sum;
  }

  return result;
}


/*
* Function computes the maximum absolute row sum norm of an upper
* triangular matrix (||M||oo).
* The dimensionality of the matrix is: (end1-start1)x(end2-start2).
*/
double maxAbsRowSumNormUT(double **M, int start, int end)
{
  double result = 0.0;
  double sum = 0.0;

  for (int i=start; i<=end; ++i)
  {
    sum = 0.0;
    for (int j=i; j<=end; ++j)
      sum+=fabs(M[j][i]);
    if (sum>result)
      result=sum;
  }

  return result;
}



/*
* Function computes one step of the algorithm inverting an upper
* triangular matrix. The first element of matrix is located at
* M[0][0] and the last one at M[size-1][size-1]. Only elements on
* and above diagonal are modified.
*/
void invertUpperTriangularStep(double **M, int colNr, int size)
{
  // Invert diagonal element (we are sure that it's not equal to zero)
  M[colNr][colNr]=1.0/M[colNr][colNr];

  // Compute elements above diagonal in column colNr
  if (colNr>0)
    multiplyVector(M[colNr], 0, colNr-1, -M[colNr][colNr]);

  // Compute the remaining columns
  double temp;
  for (int i=colNr+1; i<size; ++i)
  {
    temp = M[i][colNr];
    M[i][colNr] = 0.0;
    addMultipliedVector(M[i], M[colNr], temp, 0, colNr);
  }
}



/*
* Function checks which of SVs are linearly dependent in the feature space
* and updates alpha for all linearly dependent and independent SVs.
* Function uses the QR factorization algorithm with column pivoting.
*   - KSV (in) - Gram matrix for SVs only
*   - alphaSV (in,out) - Array of pointers to original SV alpha
*   - n (in) - Number of SV
*   - m (in) - Number of all input vectors
*   - param (in) - Model parameters
*/
int RLDSVcoreQR(double **KSV, double **alphaSV, int n, int m, const svm_parameter* param)
{
  // ----------------------------------
  // Allocating memory and prepare data
  // ----------------------------------

    // Prepare a copy of K to be modified by QR factorization
    double **R = Malloc(double *, n);
    for (int i=0; i<n; ++i)
    {
      R[i]=Malloc(double, m);
      Memcpy(R[i], KSV[i], double, m);
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printMatrix(KSV, m, n, "Matrix KSV==R");
    #endif

    // Helper variables
    int lastRow = m-1;
    int lastCol = n-1;
    double threshold = param->RLDSVthreshold;

    // Temporary variables
    double *tmpDblPtr;
    double tmpDbl1;
    double tmpDbl2;

    // Vectors containing euclidean norms of columns
    double *scaleNorms = Malloc(double, n);
    double *testNorms = Malloc(double, n);
    // Vector containing norms and later first elements of Householder vectors divided by norm.
    double *updNormsHhV1 = Malloc(double, n);


  // ----------------------------------------------------------
  // Computing Euclidean norms of vectors and normalize vectors
  // ----------------------------------------------------------
    for (int i=0; i<n; ++i)
    {
      scaleNorms[i]=euclideanNorm(KSV[i], 0, lastRow);
      if (param->RLDSVnormalize == 1)
      { // Don't normalize
        updNormsHhV1[i]=scaleNorms[i];
        testNorms[i]=scaleNorms[i];
      }
      else
      { // Normalize
        divideVector(R[i], 0, lastRow, scaleNorms[i]);
        updNormsHhV1[i]=1.0;
        testNorms[i]=1.0;
      }
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printVector(scaleNorms, n, "Scale norms");
    #endif

    // Debug info
    #ifdef DEBUG_MESSAGES
      printMatrix(R, m, n, "Normalized matrix R before factorization");
      printf("--------------------\nColumn numbers: ");
    #endif


  // -----------------------------------------------
  // Computing QR factorization with column pivoting
  // -----------------------------------------------

    // Variables
    double maxNorm;
    int maxPos;
    double curNorm;

    // Loop through all columns
    for (int i=0; i<n; ++i)
    {

      // Find column of largest norm
      maxNorm = 0.0;
      maxPos = i;
      for (int j=i; j<n; ++j)
      {
        if (updNormsHhV1[j]>maxNorm)
        {
          maxNorm = updNormsHhV1[j];
          maxPos = j;
        }
      }
/*    (TEST4)
      tmpDbl1 = 0.0;
      tmpDbl2 = maxNorm*0.9999;
      for (int j=i; j<n; ++j)
      {
        if (updNormsHhV1[j]>=tmpDbl2)
        if ((*alphaSV[j])>tmpDbl1)
        {
          tmpDbl1 = *alphaSV[j];
          maxPos = j;
        }
      }

      maxNorm = updNormsHhV1[maxPos];
*/

      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("%d ", maxPos);
      #endif

      // Exchange columns and their data
      if (maxPos!=i)
      {
        tmpDblPtr = R[i];                       // Columns of R
        R[i] = R[maxPos];
        R[maxPos] = tmpDblPtr;

        tmpDblPtr = KSV[i];                       // Columns of KSV
        KSV[i] = KSV[maxPos];
        KSV[maxPos] = tmpDblPtr;

        tmpDbl1 = scaleNorms[i];                // scaleNorms
        scaleNorms[i] = scaleNorms[maxPos];
        scaleNorms[maxPos] = tmpDbl1;

        tmpDblPtr = alphaSV[i];                 // alphaSV
        alphaSV[i] = alphaSV[maxPos];
        alphaSV[maxPos] = tmpDblPtr;

        updNormsHhV1[maxPos] = updNormsHhV1[i]; // updNormsHhV1
        testNorms[maxPos] = testNorms[i];       // testNorms
      }

      updNormsHhV1[i]=0.0;

      // Compute and apply Housholder Reflection to column i
      if ((curNorm = euclideanNorm(R[i], i, lastRow))!=0.0)
      {
        if (R[i][i]<0.0)  curNorm = -curNorm;   // Change sign of norm
        divideVector(R[i], i, lastRow, curNorm);
        R[i][i] += 1.0;

        // Apply Housholder Reflection to remaining columns >i
        for (int j=i+1; j<n; ++j)
        {
          addMultipliedVector(R[j], R[i], -dotProduct(R[i], R[j], i, lastRow)/R[i][i], i, lastRow);

          // Update norms
          if (updNormsHhV1[j] != 0.0)
          {
            tmpDbl1 = 1.0 - pow(fabs(R[j][i])/updNormsHhV1[j],2.0);
            if (tmpDbl1<0.0) tmpDbl1=0.0;
            tmpDbl2 = 1.0 + 0.05*tmpDbl1*pow(updNormsHhV1[j]/testNorms[j], 2.0);

            if (tmpDbl2 == 1.0)
            {
              updNormsHhV1[j] = euclideanNorm(R[j], i+1, lastRow);
              testNorms[j] = updNormsHhV1[j];
            }
            else
            {
              updNormsHhV1[j]*=sqrt(tmpDbl1);
            } // if
          } // if
        } // for

        // Save the first element of Householder Vector divided by the norm
        updNormsHhV1[i] = R[i][i];
        R[i][i] = -curNorm;

      } // if
    } // Main loop

    // Debug info
    #ifdef DEBUG_MESSAGES
      printf("\n--------------------\n\n");
      printMatrix(R, m, n, "Normalized matrix R after factorization");
    #endif


  // ----------------
  // Releasing memory
  // ----------------
    free(testNorms);
    testNorms = 0;


  // --------------------------------------------------------------------
  // Computing the rank r of the matrix KSV with respect to the threshold
  // --------------------------------------------------------------------
    int r;  // Numberical rank with respect to the threshold

    if (param->RLDSVrankdef == 1)
    {
      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("--------------------\nNumerical rank calculations:\n");
        printf("\t - threshold: %16.10f\n", threshold);
        printf("\t - estimated ||R22||2: ");
      #endif

      // ||R22||2<threshold
      r = n-1;
      while ( (r>0) &&
              (sqrt(maxAbsColSumNormUT(R, r, lastCol)*maxAbsRowSumNormUT(R, r, lastCol))<threshold) )
      {
        #ifdef DEBUG_MESSAGES
          printf("%16.10f ", sqrt(maxAbsColSumNormUT(R, r, lastCol)*maxAbsRowSumNormUT(R, r, lastCol)));
        #endif
        --r;
      }
      ++r;

      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("%16.10f ", sqrt(maxAbsColSumNormUT(R, r-1, lastCol)*maxAbsRowSumNormUT(R, r-1, lastCol)));
        printf("\n\t - numerical rank: %d\n", r);
        printf("--------------------\n\n");
        fflush(stdout);
      #endif

    }
    else if (param->RLDSVrankdef == 2)
    {
      // R(r+1, r+1)<threshold
      r = n-1;
      while ( (r>0)&&(fabs(R[r][r])<threshold) )
        --r;
      ++r;

      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("--------------------\nNumerical rank calculations:\n");
        printf("\t - threshold: %16.10f\n", threshold);
        printf("\t - numerical rank: %d\n", r);
        printf("--------------------\n\n");
        fflush(stdout);
      #endif
    }
    else
    {
      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("--------------------\nNumerical rank calculations:\n");
        printf("\t - threshold: %16.10f\n", threshold);
        printf("\t - estimated infR11: ");
      #endif

      // inf(R11)>=threshold
      double infR11=0.0;

      invertUpperTriangularStep(R, 0, n);   // always leave at lest 1 vector

      for (r=1; r<n; ++r)
      {
        if (R[r][r]==0) break;

        invertUpperTriangularStep(R, r, n);
        infR11 = 1.0/sqrt(maxAbsColSumNormUT(R, 0, r)*maxAbsRowSumNormUT(R, 0, r));
        #ifdef DEBUG_MESSAGES
          printf("%16.10f ", infR11);
        #endif

        if (infR11<threshold) break;
      }


      // Debug info
      #ifdef DEBUG_MESSAGES
        printf("\n\t - numerical rank: %d\n", r);
        printf("--------------------\n\n");
        fflush(stdout);
      #endif
    }


  // -----------------------------------
  // Computing the inverse of R11 matrix
  // -----------------------------------

    if (param->RLDSVrankdef !=3)
    {
      for (int i=0; i<r; ++i)
        invertUpperTriangularStep(R, i, r);
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printMatrix(R, r, r, "Matrix R11 after inversion");
    #endif


  // -----------------
  // Allocating memory
  // -----------------
    // The result of multiplying transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n)
    double *Q1T_KSV2_alphaSV2 = Malloc(double, m);
    Memclear(Q1T_KSV2_alphaSV2, double, m);


  // ------------------------------------------
  // Computing KSV(1:m, r+1:n) * alphaSV(r+1:n)
  // ------------------------------------------
    for (int j=r; j<n; ++j)
    {
      tmpDblPtr = KSV[j];
      tmpDbl1 = *(alphaSV[j]);
      for (int i=0; i<m; ++i)
        Q1T_KSV2_alphaSV2[i]+=tmpDblPtr[i]*tmpDbl1;
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printVectorPtr(alphaSV, n, "Vector alphaSV");
      printVector(Q1T_KSV2_alphaSV2, m, "Vector KSV(1:m, r+1:n)*alphaSV(r+1:n)");
    #endif



  // ---------------------------------------------------------------------------------
  // Computing transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n) using Householder vectors
  // ---------------------------------------------------------------------------------
    // It's only necessary to multiply r Householder Matrices
    for (int i=0; i<r; ++i)
    {
      if (i<lastRow)
      {
        tmpDbl1 = -(updNormsHhV1[i]*Q1T_KSV2_alphaSV2[i]+dotProduct(R[i], Q1T_KSV2_alphaSV2, i+1, lastRow)) / updNormsHhV1[i];

        Q1T_KSV2_alphaSV2[i]+=tmpDbl1*updNormsHhV1[i];
        addMultipliedVector(Q1T_KSV2_alphaSV2, R[i], tmpDbl1, i+1, lastRow);
      }
      else
      {
        tmpDbl1 = -(updNormsHhV1[i]*Q1T_KSV2_alphaSV2[i]) / updNormsHhV1[i];
        Q1T_KSV2_alphaSV2[i]+=tmpDbl1*updNormsHhV1[i];
      }
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printVector(Q1T_KSV2_alphaSV2, r, "Vector transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n)");
    #endif


  // ----------------
  // Releasing memory
  // ----------------
    free(updNormsHhV1);
    updNormsHhV1 = 0;


  // -----------------
  // Allocating memory
  // -----------------
    // The result of multiplying inv(R11) * transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n)
    double *R11I_Q1T_KSV2_alphaSV2 = Malloc(double, r);
    Memclear(R11I_Q1T_KSV2_alphaSV2, double, r);


  // ------------------------------------------------------------------
  // Computing inv(R11) * transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n)
  // ------------------------------------------------------------------
    for (int j=0; j<r; ++j)
    {
      tmpDblPtr = R[j];
      tmpDbl1 = Q1T_KSV2_alphaSV2[j];
      for (int i=0; i<=j; ++i)
        R11I_Q1T_KSV2_alphaSV2[i]+=tmpDblPtr[i]*tmpDbl1;
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printVector(R11I_Q1T_KSV2_alphaSV2, r, "Vector inv(R11) * transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n)");
    #endif

  // ----------------
  // Releasing memory
  // ----------------
    free(Q1T_KSV2_alphaSV2);
    Q1T_KSV2_alphaSV2 = 0;


  // ---------------------------------------------
  // Scaling the result if columns were normalized
  // ---------------------------------------------
    if (param->RLDSVnormalize != 1)
    { // Normalization enabled
      for (int i=0; i<r; ++i)
        R11I_Q1T_KSV2_alphaSV2[i]/=scaleNorms[i];
    }

    // Debug info
    #ifdef DEBUG_MESSAGES
      printVector(R11I_Q1T_KSV2_alphaSV2, r, "Vector inv(R11) * transp(Q1) * KSV(1:m, r+1:n) * alphaSV(r+1:n) after scaling");
    #endif


  // ----------------
  // Updating alphaSV
  // ----------------

    // alphaSV1
    for(int i=0; i<r; ++i)
      *alphaSV[i]+=R11I_Q1T_KSV2_alphaSV2[i];

    // alphaSV2
    for(int i=r; i<n; ++i)
      *alphaSV[i]=0.0;


  // ----------------
  // Releasing memory
  // ----------------

    // Matrix R
    for (int i=0; i<n; ++i)
      free(R[i]);
    free(R);
    R = 0;

    // Vector norms
    free(scaleNorms);
    scaleNorms = 0;

    // R11I_Q1T_KSV2_alphaSV2
    free(R11I_Q1T_KSV2_alphaSV2);
    R11I_Q1T_KSV2_alphaSV2 = 0;


  return n-r;
} // RLDSVcoreQR




/*
* Main function. Reduces the support vectors that are linearly dependent
* in the feature space. Should be called after training.
*/
void reduceLinDepSV(decision_function *df, const svm_problem *prob, const svm_parameter* param)
{
  double **KSV = 0;      // Gram matrix for SVs only, copy of data
  double **alphaSV =0; // Array of SV alpha, pointers to data
  int nrSV; // Number of SVs
  int nrSVReduced; // Number of SVs after reduction

  #ifdef DEBUG_MESSAGES
    fprintf(stdout, "\n----------- SV Reduction -----------\n\n");
  #endif

  prepareData(prob->x, df->alpha, prob->l, param, KSV, alphaSV, nrSV);

  if (nrSV==0)
  {
    printf("Error! Zero support vectors!!");
    fflush(stdout);
    // Memory not allocated, no need to release
    return;
  }

  if (param->RLDSValgorithm == 2)
    nrSVReduced = RLDSVcoreGJE(KSV, alphaSV, nrSV, prob->l, param);
  else
    nrSVReduced = RLDSVcoreQR(KSV, alphaSV, nrSV, prob->l, param);

  releaseMemory(KSV, alphaSV, nrSV);

  // Printf results
  fprintf(stdout,"RLDSV results: [Before: %d SVs] [After: %d SVs] [Reduced: %.3f%%]\n", nrSV, nrSV-nrSVReduced, 100.0*((float)nrSVReduced/(float)nrSV));
  fflush(stdout);

  #ifdef DEBUG_MESSAGES
    fprintf(stdout, "\n\n");
  #endif
}



/*
* Function loads training set from file and creates the problem in
* memory. Labels are class numbers (generated based on model)
* instead of original labels!
*/
bool loadTrainingSet(char *training_set, svm_model *model, svm_problem *&problem, svm_node *&x_space)
{
  // Open file
  FILE *f = fopen(training_set, "r");
  if (f == 0) return false;


  // Count elements in file
  int l = 0;
  int elem = 0;
  int c;

  while ( (c=fgetc(f)) != EOF )
  {
    if (c==':')
    {
      ++elem;
    }
    else if (c=='\n')
    {
      ++l;
      ++elem;
    }
  }


  // Allocate memory
  problem = Malloc(svm_problem, 1);
  problem->y = Malloc(double, l);
  problem->x = Malloc(svm_node*, l);
  problem->l = l;
  x_space = Malloc(svm_node, elem);


  // Rewind file
  rewind(f);
  double label;
  int j=0;
  int k=0;
  for (int i=0; i<l; ++i)
  {
    // Remember pointer
    problem->x[i] = &x_space[j];

    // Read label
    fscanf(f, "%lf", &label);

    // Find class number
    for(k=0; (k < model->nr_class) && (model->label[k] != label); ++k) ;
    if (k==model->nr_class)
    {
      fclose(f);
      return false;   // Class label not found
    }
    else
      problem->y[i] = k;

    // Read nodes
    do
    {
      do
      {
        c=fgetc(f);
      } while ((isspace(c)) && (c!='\n'));

      if (c=='\n')
      {
        x_space[j].index=-1;
        ++j;
      }
      else
      {
        ungetc(c,f);
        fscanf(f,"%d:%lf",&(x_space[j].index),&(x_space[j].value));
        ++j;
      }
    } while (c!='\n');
  } // for (int i=0; i<l; ++i)

  // Close file
  fclose(f);

  return true;
}



/*
* Function allocates memory and prepares input data for reduction
* algorithm. Function handles only one against one. Extracts vectors
* belonging only to two classes given as parameters.
* Parameters:
*   - model (in) - Input model
*   - problem (in) - Problem containing training data
*   - class1 (in) - Class number
*   - class2 (in) - Class number
*   - KSV (out) - Gram matrix for SVs only
*   - alphaSV (out) - Array of pointers to original SV alpha
*   - l (out) - Number of vectors (training data) belonging to given classes
*   - nrSV (out) - Number of SV belonging to given classes
*/
void prepareDataFromModelOaO(svm_model *model, svm_problem *problem, int class1, int class2,
                             double **&KSV, double **&alphaSV, int &l, int &nrSV)
{
  // Initialization
  // Index of the first SV falling into the class
  int start1=0;
  int start2=0;
  for (int i=0; i<class1; ++i) start1+=model->nSV[i];
  for (int i=0; i<class2; ++i) start2+=model->nSV[i];
  // Amount of SVs falling into the class
  int count1 = model->nSV[class1];
  int count2 = model->nSV[class2];
  int end1 = start1+count1;    // The last index is end1-1
  int end2 = start2+count2;    // The last index is end2-1
  // Parts od coeff table in which to look for support vectors
  double *alpha1 = model->sv_coef[class2-1];  // indexes from start1 to start1+count1-1
  double *alpha2 = model->sv_coef[class1];    // indexes from start2 to start2+count2-1


  // Count SV
  nrSV=0;
  for(int i=start1; i<end1; ++i)
    if (fabs(alpha1[i])>0) ++nrSV;
  for(int i=start2; i<end2; ++i)
    if (fabs(alpha2[i])>0) ++nrSV;


  // Count vectors belonging to given classes
  l=0;
  int prob_l = problem->l;
  for(int i=0; i<prob_l; ++i)
    if ( (problem->y[i]==class1) | (problem->y[i]==class2) ) ++l;

  // Allocate memory
  KSV = Malloc(double *, nrSV);
  alphaSV = Malloc(double *, nrSV);

  // Loop for class1
  int j=0;
  for (int i=0; i<count1; ++i)
  {
    if (fabs(alpha1[start1+i])>0)
    {
      alphaSV[j] = &alpha1[start1+i];
      KSV[j] = Malloc(double, l);

      for (int a=0, b=0; b<l; ++a )
        if ( (problem->y[a]==class1) | (problem->y[a]==class2) )
        {
            KSV[j][b] = Kernel::k_function(model->SV[start1+i],problem->x[a],model->param);
            if (isnan(KSV[j][b])) KSV[j][b]=0.0;
            ++b;
        }

      ++j;
    }
  }


  // Loop for class2
  for (int i=0; i<count2; ++i)
  {
    if (fabs(alpha2[start2+i])>0)
    {
      alphaSV[j] = &alpha2[start2+i];
      KSV[j] = Malloc(double, l);

      for (int a=0, b=0; b<l; ++a )
        if ( (problem->y[a]==class1) | (problem->y[a]==class2) )
        {
            KSV[j][b] = Kernel::k_function(model->SV[start2+i],problem->x[a],model->param);
            if (isnan(KSV[j][b])) KSV[j][b]=0.0;
            ++b;
        }

      ++j;
    }
  }

}



/*
* Function allocates memory and prepares input data for reduction
* algorithm. Function handles only one against all case.
* Parameters:
*   - model (in) - Input model
*   - problem (in) - Problem containing training data
*   - dfNr (in) - Decision functio number
*   - KSV (out) - Gram matrix for SVs only
*   - alphaSV (out) - Array of pointers to original SV alpha
*   - l (out) - Number of all vectors (training data)
*   - nrSV (out) - Number of SV for the given decision function
*/
void prepareDataFromModelOaA(svm_model *model, svm_problem *problem, int dfNr,
                             double **&KSV, double **&alphaSV, int &l, int &nrSV)
{
  // Idea to make the whole process faster: Compute the whole gram matrix for
  // all SVs (for all decision functions) and then just copy those that are needed

  // Initialization
  double *alpha = model->sv_coef[dfNr];
  l = problem->l;
  int model_l = model->l;

  // Count SVs
  nrSV=0;
  for (int i=0; i<model_l; ++i)
    if (fabs(alpha[i])>0) ++nrSV;

  // Allocate memory
  KSV = Malloc(double *, nrSV);
  alphaSV = Malloc(double *, nrSV);

  // Fill KSV and alphaSV
  int j=0;
  double *curCol;
  for (int i=0; i<model_l; ++i)
  {
    if (fabs(alpha[i])>0)
    {
      alphaSV[j] = &alpha[i];
      curCol = Malloc(double, l);
      KSV[j] = curCol;

      for (int k=0; k<l; ++k)
      {
        curCol[k] = Kernel::k_function(model->SV[i],problem->x[k],model->param);
        if (isnan(curCol[k])) curCol[k]=0.0;
      }

      ++j;
    }
  }

}



/*
* Function removes vectors that are no longer support vectors for any of
* the decision functions. Handles only one against one case.
* Parameters:
*   - model (in) - Model after reduction of lin. dep. SV
*/
void removeReducedVectorsOaO(svm_model *model)
{
  // Initialization
  int old_l = model->l;
  int nr_class = model->nr_class;
  int *start = Malloc(int, nr_class);
  int *end = Malloc(int, nr_class);
  start[0] = 0;
  for(int i=1; i<nr_class; ++i)   start[i] = start[i-1] + model->nSV[i-1];
  for(int i=0; i<nr_class; ++i)   end[i] = start[i] + model->nSV[i];
  bool *remove = Malloc(bool, old_l);

  // Find the size of xspace
  int old_xSpaceSize=0;
  svm_node *old_xSpace=model->SV[0];
  {
    int j=0;
    for(int i=0; i<old_l; ++i)
    {
      for( ;old_xSpace[j].index!=-1; ++j)
        ++old_xSpaceSize;
      ++old_xSpaceSize; //-1 elem
      ++j;
    }
  }

  // Count vectors that are reduced
  for(int i=0; i<nr_class; ++i)
  {
    for(int j=start[i]; j<end[i]; ++j)
    {
      // Check if vector nr j belonging to class nr. i is no longer support vector
      remove[j] = true;
      for(int k=0; k<(nr_class-1); ++k)
        if (fabs(model->sv_coef[k][j])>0)
        {
          remove[j] = false;
          break;
        }

      // Update model
      if (remove[j])
      {
        --model->nSV[i];
        --model->l;
      }
    } // for(int j=start[i]; j<end[i]; ++j)
  } // for(int i=0; i<nr_class; ++i)

  // At this point model->l and model->nSV are already updated

  // Remove unnecessary vectors
  svm_node *new_xSpace = Malloc(svm_node, old_xSpaceSize);
  svm_node **new_SV = Malloc(svm_node *, model->l);
  double **new_coef = Malloc(double *, nr_class-1);
  for(int i=0; i<nr_class-1; ++i)
    new_coef[i] = Malloc(double, model->l);

  int xSpacePtr=0;
  for(int i=0,j=0; i<old_l; ++i)
  {
    if (!remove[i])
    {
      // Update pointers
      new_SV[j] = new_xSpace+xSpacePtr;
      // Copy SV to new xSpace
      for(int k=0; model->SV[i][k].index!=-1; ++k)
      {
        new_xSpace[xSpacePtr]=model->SV[i][k];
        ++xSpacePtr;
      }
      new_xSpace[xSpacePtr].index=-1;
      ++xSpacePtr;
      // Update coefs
      for(int k=0; k<(nr_class-1); ++k)
        new_coef[k][j] = model->sv_coef[k][i];
      ++j;
    }
  }

  // Exchange old data with new data
  free(model->SV);
  free(old_xSpace);
  model->SV = new_SV;

  for(int i=0; i<(nr_class-1); ++i)
    free(model->sv_coef[i]);
  free(model->sv_coef);
  model->sv_coef = new_coef;

  // Free allocated memory
  free(start);
  free(end);
  free(remove);
}



/*
* Function removes vectors that are no longer support vectors for any of
* the decision functions. Handles only one against all case
* Parameters:
*   - model (in) - Model after reduction of lin. dep. SV
*/
void removeReducedVectorsOaA(svm_model *model)
{
  // Initialization
  int old_l = model->l;
  int nr_class = model->nr_class;
  bool *remove = Malloc(bool, old_l);

  int *start = Malloc(int, nr_class);
  int *end = Malloc(int, nr_class);
  start[0] = 0;
  for(int i=1; i<nr_class; ++i)   start[i] = start[i-1] + model->nSV[i-1];
  for(int i=0; i<nr_class; ++i)   end[i] = start[i] + model->nSV[i];

  // Find the size of xspace
  int old_xSpaceSize=0;
  svm_node *old_xSpace=model->SV[0];
  {
    int j=0;
    for(int i=0; i<old_l; ++i)
    {
      for( ;old_xSpace[j].index!=-1; ++j)
        ++old_xSpaceSize;
      ++old_xSpaceSize; //-1 elem
      ++j;
    }
  }

  // Count vectors that are reduced
  for (int i=0; i<old_l; ++i)
  {
    // Check if vector nr i is no longer support vector
    remove[i]=true;
    for (int j=0; j<nr_class; ++j)
    {
      if (fabs(model->sv_coef[j][i])>0)
      {
        remove[i]=false;
        break;
      }
    }

    // Update model
    if (remove[i])
      --model->l;
  }

  // At this point model->l is already updated

  // Decrease nSV
  for(int i=0; i<nr_class; ++i)
  {
    for(int j=start[i]; j<end[i]; ++j)
    {
      if (remove[j])
      {
        --model->nSV[i];
      }
    }
  }

// old broken version: for (int i=0; i<nr_class; ++i) model->nSV[i]=0;

  // Remove unnecessary vectors and count SVs for each decision function
  svm_node *new_xSpace = Malloc(svm_node, old_xSpaceSize);
  svm_node **new_SV = Malloc(svm_node *, model->l);
  double **new_coef = Malloc(double *, nr_class);
  for(int i=0; i<nr_class; ++i)
    new_coef[i] = Malloc(double, model->l);

  int xSpacePtr=0;
  int j=0;
  for (int i=0; i<old_l; ++i)
    if (!remove[i])
    {
      // Update pointers
      new_SV[j] = new_xSpace+xSpacePtr;
      // Copy SV to new xSpace
      for(int k=0; model->SV[i][k].index!=-1; ++k)
      {
        new_xSpace[xSpacePtr]=model->SV[i][k];
        ++xSpacePtr;
      }
      new_xSpace[xSpacePtr].index=-1;
      ++xSpacePtr;
      // Update coefs
      for (int k=0; k<nr_class; ++k)
      {
//        if (fabs(new_coef[k][j] = model->sv_coef[k][i])>0) ++(model->nSV[k]);
        new_coef[k][j] = model->sv_coef[k][i];
      }
      ++j;
    }

  // Exchange old data with new data
  free(model->SV);
  free(old_xSpace);
  model->SV = new_SV;

  for(int i=0; i<nr_class; ++i)
    free(model->sv_coef[i]);
  free(model->sv_coef);
  model->sv_coef = new_coef;

  // Free allocated memory
  free(remove);
  free(start);
  free(end);

}



/*
* Main function. Reduces the support vectors from given model that are
* linearly dependent in the feature space.
*/
bool reduceModel(char *training_set, char *input_model, char *output_model,
                 double threshold, char algorithm, char rankdef, char normalize)
{
  svm_model *model = 0;
  svm_problem *problem = 0;
  svm_node *x_space = 0;
  int nrSVBefore1, nrSVBefore2, nrSVAfter1, nrSVAfter2;  // Number of SVs before and after r.

  // ------------
  //  Read model
  // ------------
  printf("Reading model...");
  fflush(stdout);

    if ((model=svm_load_model(input_model))==0)
    {
      printf(" Error! Cannot load model file!\n");
      // No need to release memory, svm_load_model takes care of it in case of error.
      return false;
    }

    // Check model type and params (we work only with C_SVC)
    if ((model->param.svm_type!=C_SVC) && (model->param.svm_type!=ONE_AGAINST_ALL))
    {
      printf(" Error! Wrong SVM type.\nSvm-reduce works only with one against one C_SVC and one against all C_SVC.\n");
      releaseMemory(model, problem, x_space);  // Now we have to release model
      return false;
    }

    // Initialize variables
    int nrClass = model->nr_class; // Number of classes
    model->param.RLDSV = 1;
    model->param.RLDSVthreshold = threshold;
    model->param.RLDSValgorithm = algorithm;
    model->param.RLDSVrankdef = rankdef;
    model->param.RLDSVnormalize = normalize;
    nrSVBefore1 = model->l;  // Real number of SVs
    nrSVBefore2 = 0;         // Sum over all d.f.
    nrSVAfter1 = 0;
    nrSVAfter2 = 0;


  printf(" Done!\n\n");
  fflush(stdout);


  // -------------------
  //  Read training set
  // -------------------
  printf("Reading training set...");
  fflush(stdout);

  if (!loadTrainingSet(training_set, model, problem, x_space))
  {
    printf(" Error! Cannot load training set file!\n");
    releaseMemory(model, problem, x_space);
    return false;
  }

  printf(" Done!\n\n");
  fflush(stdout);


  // -------------------------
  //  Run reduction algorithm
  // -------------------------
  printf("Reducing support vectors...\n");
  fflush(stdout);

    // Reduction algorithm
    double **KSV = 0;      // Gram matrix for SVs only, copy of data
    double **alphaSV = 0; // Array of SV alpha, pointers to data
    int nrSV; // Number of SV falling into considered classes
    int nrSVReduced; // Number of SV after reduction
    int l; // Number of vectors (training data) falling into considered classes

    if (model->param.svm_type == ONE_AGAINST_ALL)
    { // One against all

      for(int i=0; i<nrClass; ++i)
      {
          // Reduce decision function
          prepareDataFromModelOaA(model, problem, i, KSV, alphaSV, l, nrSV);

          if (nrSV>0)
          {
            if (model->param.RLDSValgorithm == 2)
              nrSVReduced=RLDSVcoreGJE(KSV, alphaSV, nrSV, l, &(model->param));
            else
              nrSVReduced=RLDSVcoreQR(KSV, alphaSV, nrSV, l, &(model->param));
          }
          else
          {
            printf("Error! Zero support vectors!!");
            fflush(stdout);
            nrSVReduced=0;
          }

          releaseMemory(KSV, alphaSV, nrSV);

          // Printf results
          nrSVBefore2+=nrSV;
          nrSVAfter2+=nrSV-nrSVReduced;
          fprintf(stdout,"DF nr. %d: [Before: %d SVs] [After: %d SVs] [Reduced: %.3f%%]\n",
                  i, nrSV, nrSV-nrSVReduced, 100.0*((float)nrSVReduced/(float)nrSV));
          fflush(stdout);
      } // END: for

    } // END: One against all
    else
    { // One against one

      int dfNum = 0; // Decision function number
      dfNum = 0;
      for(int i=0; i<nrClass; ++i)
        for(int j=i+1; j<nrClass; ++j)
        {
            // Reduce decision function
            prepareDataFromModelOaO(model, problem, i, j, KSV, alphaSV, l, nrSV);

            if (nrSV>0)
            {
              if (model->param.RLDSValgorithm == 2)
                nrSVReduced=RLDSVcoreGJE(KSV, alphaSV, nrSV, l, &(model->param));
              else
                nrSVReduced=RLDSVcoreQR(KSV, alphaSV, nrSV, l, &(model->param));
            }
            else
            {
              printf("Error! Zero support vectors!!");
              fflush(stdout);
              nrSVReduced=0;
            }

            releaseMemory(KSV, alphaSV, nrSV);

            // Printf results
            nrSVBefore2+=nrSV;
            nrSVAfter2+=nrSV-nrSVReduced;
            ++dfNum;
            fprintf(stdout,"DF nr. %d: [Before: %d SVs] [After: %d SVs] [Reduced: %.3f%%]\n",
                    dfNum, nrSV, nrSV-nrSVReduced, 100.0*((float)nrSVReduced/(float)nrSV));
            fflush(stdout);
        } // END: for

    } // END: One against one

  printf("Done!\n\n");
  fflush(stdout);



  // --------------------
  //  Save reduced model
  // --------------------
  printf("Saving reduced model...");
  fflush(stdout);

    // If a SV[nr] for all coeff[i][nr] has alpha=0 it may be totaly removed from model.
    // Therefore we investigate coeff to find such vectors and then we remove them
    // from SV and coeff. We also have to update nSV for class to which this vector
    // belonged.
    if (model->param.svm_type == ONE_AGAINST_ALL)
      removeReducedVectorsOaA(model);
    else
      removeReducedVectorsOaO(model);

    nrSVAfter1 = model->l;

    if (svm_save_model(output_model, model)<0)
    {
      printf(" Error! Cannot save model!\n");
      releaseMemory(model, problem, x_space);
      return false;
    }

  printf(" Done!\n\n");
  fflush(stdout);



  // ----------------
  //  Release memory
  // ----------------
  releaseMemory(model, problem, x_space);



  // --------------------
  // Print final results
  // --------------------
  fprintf(stdout,"Final results:\n");
  fprintf(stdout,"\t* Overall: [Before: %d SVs] [After: %d SVs] [Reduced: %.3f%%]\n", nrSVBefore1, nrSVAfter1, 100.0*((float)(nrSVBefore1-nrSVAfter1)/(float)nrSVBefore1));
  fprintf(stdout,"\t* Summed up: [Before: %d SVs] [After: %d SVs] [Reduced: %.3f%%]\n\n", nrSVBefore2, nrSVAfter2, 100.0*((float)(nrSVBefore2-nrSVAfter2)/(float)nrSVBefore2));
  fflush(stdout);

  return true;
}


