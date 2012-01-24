#include "allocMatrix.h"

double** allocDoubleArray(int dim1, int dim2) {
	int i; 
	double** array= (double**) malloc(sizeof(double)*dim1);
	for(i=0;i<dim1;i++)
	{
		array[i]= (double*) malloc(sizeof(double)*dim2);
	}
	return array;
}

int** allocIntArray(int dim1, int dim2) {
	int i; 
	int** array= (int**) malloc(sizeof(int)*dim1);
	for(i=0;i<dim1;i++)
	{
		array[i]= (int*) malloc(sizeof(int)*dim2);
	}
	return array;
}

int** reallocIntArray(int** voteMat, int allocatedSpaceForVotes, int dim2, int n_new) 
{
	voteMat= (int**) realloc(voteMat, sizeof(int*)*(allocatedSpaceForVotes+n_new));
	for(int i= allocatedSpaceForVotes; i<allocatedSpaceForVotes+n_new; i++)
	{
		voteMat[i]= (int*) malloc(sizeof(int)*dim2);
	}
	return voteMat;
}


void initIntMatrix(int** mat, int dim1, int dim2, int value)
{
	for(int i= 0; i<dim1; i++)
	{
		for(int j= 0; j<dim2; j++)
		{
			mat[i][j]= value;
		}
	}
}
