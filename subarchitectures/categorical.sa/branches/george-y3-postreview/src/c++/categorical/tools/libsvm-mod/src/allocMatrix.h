#ifndef _ALLOCMATRIX_H
#define _ALLOCMATRIX_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

double** allocDoubleArray(int dim1, int dim2);
int** allocIntArray(int dim1, int dim2);
int** reallocIntArray(int** voteMat, int allocatedSpaceForVotes, int dim2, int n_new);

void initIntMatrix(int** mat, int dim1, int dim2, int value);

#ifdef __cplusplus
}
#endif

#endif
