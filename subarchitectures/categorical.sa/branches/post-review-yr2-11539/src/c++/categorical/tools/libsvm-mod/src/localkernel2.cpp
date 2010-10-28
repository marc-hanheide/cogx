/**
* Speeded up local kernel.
* \file localkernel2.cpp
* \author Andrzej Pronobis
*/

#include "localkernel2.h"
#include <stdio.h>
#include <math.h>
#include <algorithm>
using namespace std;

#define LARGE_VAL 100000000.0


/** Matrix storing the local features in a dense format. */
static double **_xDense=0;

/** Matrix storing the local features in a dense format. */
static double **_yDense=0;

/** Signs of laplacian. */
static int *_xLaplaceSigns=0;

/** Signs of laplacian. */
static int *_yLaplaceSigns=0;

/** Matrix storing the calculated distances. */
static double **_distances=0;

/** Matrix storing the no of the distance metric used to calculate
 * the distance in _distances. */
static int **_metrics=0;

/** Matches that were found. */
static double *_matches=0;

/** Number of local features in the vector x. */
static int _xCount;

/** Number of local features in the vector y. */
static int _yCount;

/** Maximal number of features. */
static int _maxFeatures;

/** Number of considered matches. */
static int _maxMatches;

/** Size of a single local descriptor vector. */
static int _featureSize;

/** Use enhancements for SURF features. */
static bool _surfEnhancements;

/** Assume that there are laplacian signs in the data. */
static bool _surfLaplacians;

/** Pointer to the params structure. */
static const svm_parameter *_params;

/** Factors by which the distance is multiplied. */
static double _distFactors[4];

/** Statistics for debugging. */
static long _stat[4] = {0,0,0,0};
static long _statSum = 0;


/** Measures CPU time. */
//long long rdtsc()
//{
//  asm("RDTSC");
//}


/** Allocates a matrix of doubles. */
double** allocateDoubleMatrix(int dim1, int dim2)
{
  double** matrix = (double**)malloc(sizeof(double*)*dim1);
  matrix[0] = (double*) malloc(sizeof(double)*dim1*dim2);
  for(int i=1; i<dim1; ++i)
    matrix[i]= *matrix + dim2 * i;
  return matrix;
}


/** Frees a matrix of doubles. */
void freeDoubleMatrix(double **matrix)
{
  free(matrix[0]);
  free(matrix);
}


/** Allocates a matrix of integers. */
int** allocateIntMatrix(int dim1, int dim2)
{
  int** matrix = (int**)malloc(sizeof(int*)*dim1);
  matrix[0] = (int*) malloc(sizeof(int)*dim1*dim2);
  for(int i=1; i<dim1; ++i)
    matrix[i]= *matrix + dim2 * i;
  return matrix;
}


/** Frees a matrix of integers. */
void freeIntMatrix(int **matrix)
{
  free(matrix[0]);
  free(matrix);
}


/** Generates statistics of distance measures. */
void getMetricsStat()
{
  if (_xCount<=_yCount)
  {
    for (int i=0; i<_xCount; ++i)
    {
      for (int j=0; j<_yCount; ++j)
      {
        _stat[_metrics[i][j]]++;
        _statSum++;
      }
    }
  }
  else
  {
    for (int i=0; i<_yCount; ++i)
    {
      for (int j=0; j<_xCount; ++j)
      {
        _stat[_metrics[i][j]]++;
        _statSum++;
      }
    }
  }

  printf("Metrics stats: %ld %ld %ld %ld, xc=%d, yc=%d\n", (_stat[0]*100)/_statSum, (_stat[1]*100)/_statSum, (_stat[2]*100)/_statSum, (_stat[3]*100)/_statSum, _xCount, _yCount);
}


/** Prints a matrix, for debugging. */
void printDoubleMatrix(double**m, int s1, int s2)
{
  printf("====MATRIX====\n");
  for (int i=0; i<s1; ++i)
  {
    for (int j=0; j<s1; ++j)
    {
      printf("%d:%d:%g ", i, j, m[i][j]);
    }
    printf("\n");
  }
  printf("====MATRIX END====\n");
}



/** Initializes parameters and allocates static memory. */
void initParams(const svm_parameter &param)
{
  // Check if the memory was allocated
  if (!_xDense)
  {
    // Save the important params
    _maxFeatures=param.n_features;
    _maxMatches=param.nummax;
    _featureSize=param.featuredim;
    _surfEnhancements=(param.surfEnhancements==1);
    _surfLaplacians=(param.surfLaplacians==1);
    _params=&param;

    // Allocate the memory
    _xDense=allocateDoubleMatrix(_maxFeatures, _featureSize);
    _yDense=allocateDoubleMatrix(_maxFeatures, _featureSize);
    _xLaplaceSigns=(int*)malloc(sizeof(int)*_maxFeatures);
    _yLaplaceSigns=(int*)malloc(sizeof(int)*_maxFeatures);
    _distances=allocateDoubleMatrix(_maxFeatures, _maxFeatures);
    _metrics=allocateIntMatrix(_maxFeatures, _maxFeatures);
    _matches=(double*)malloc(sizeof(double)*_maxFeatures);

    // Calculate prediction factors ( 1 + (0.5*(4-(l+1))) / (l+1) )
    _distFactors[0]=( 1 + (param.distCompensationCoeff*(4-(0+1))) / (0+1) ); // for a=0.5: 1+0.5*3/1=2.5
    _distFactors[1]=( 1 + (param.distCompensationCoeff*(4-(1+1))) / (1+1) ); // 1+0.5*2/2 = 1.5
    _distFactors[2]=( 1 + (param.distCompensationCoeff*(4-(2+1))) / (2+1) ); // 1+0.5*1/3 = 1.16
    _distFactors[3]=( 1 + (param.distCompensationCoeff*(4-(3+1))) / (3+1) ); // 1
  }
}


/** Calculates initial distance (the most coarse one) for two local features. */
inline double calculateInitDistance(double *xDense, double *yDense, double distFactor)
{
  // Calculate the distance
  double dist=0;
  for (int i=0; i<_featureSize; i+=4)
  {
    double diff = xDense[i] - yDense[i];
    dist += diff * diff;
  }
  dist*=distFactor;

  return dist;
}



/** Calculates distance using metric l for two local features. */
inline double calculateDistance(double *xDense, double *yDense, int l, double dist, double distFactors[])
{
  // Calculate the distance
  dist/=distFactors[l-1];
  for (int i=l; i<_featureSize; i+=4)
  {
    double diff = xDense[i] - yDense[i];
    dist += diff * diff;
  }
  dist*=distFactors[l];

  return dist;
}


/** Sets the variables for two new feature vectors. */
void setFeatures(const svm_node *x, const svm_node *y)
{
  int maxFeatures=_maxFeatures;

  // Convert the feature vectors to a dense format
  // and get the dimensions
  int skip=(_surfLaplacians)?3:2;
  int xCount;
  for (xCount=0; (xCount<maxFeatures) && (x->index!=-1); ++xCount)
  {
    // Skip the position information and update the counters
    ++x; ++x;

    // Read the sign of laplacian
    if (_surfLaplacians)
    {
      ++x;
    }

    // Initalize some temp value
    int tmp=skip + xCount*(_featureSize+skip) + 1; // skip because of position info, 1 because counting starts from 1

    // Read local features
    for (int i=0; i<_featureSize; ++i)
    {
      if (x->index == tmp + i )
      {
        _xDense[xCount][i] = x->value;
        ++x;
      }
      else
        _xDense[xCount][i] = 0.0;
    }
  }
  _xCount=xCount;

  int yCount;
  for (yCount=0; (yCount<maxFeatures) && (y->index!=-1); ++yCount)
  {
    // Skip the position information and update the counters
    ++y; ++y;

    // Read the sign of laplacian
    if (_surfLaplacians)
    {
      ++y;
    }

    // Initalize some temp value
    int tmp=skip + yCount*(_featureSize+skip) + 1; // skip because of position info, 1 because counting starts from 1

    // Read local features
    for (int i=0; i<_featureSize; ++i)
    {
      if (y->index == tmp + i )
      {
        _yDense[yCount][i] = y->value;
        ++y;
      }
      else
        _yDense[yCount][i] = 0.0;
    }
  }
  _yCount=yCount;

  // Initalize the memory (distances and matches)
  double **distances=_distances;
  int **metrics=_metrics;
  double *matches=_matches;
  double **xDense=_xDense;
  double **yDense=_yDense;
  double distFactor=_distFactors[0];
  if (xCount<=yCount)
  {
    for (int i=0; i<xCount; ++i)
    {
      double *distancesRow=distances[i];
      int *metricsRow=metrics[i];
      matches[i]=-LARGE_VAL;
      double *xDenseI=xDense[i];
      for (int j=0; j<yCount; ++j)
      {
        distancesRow[j] = calculateInitDistance(xDenseI, yDense[j], distFactor);
        metricsRow[j] = 0;
      }
    }
  }
  else
  {
    for (int i=0; i<yCount; ++i)
    {
      double *distancesRow=distances[i];
      int *metricsRow=metrics[i];
      matches[i]=-LARGE_VAL;
      double *yDenseI=yDense[i];
      for (int j=0; j<xCount; ++j)
      {
        distancesRow[j] = calculateInitDistance(xDense[j], yDenseI, distFactor);
        metricsRow[j] = 0;
      }
    }
  }
}



/** Sets the variables for two new feature vectors. Uses the laplacian sign to 
    calculate initial distance. */
void setFeaturesLaplace(const svm_node *x, const svm_node *y)
{
  int maxFeatures=_maxFeatures;
  int *xLaplaceSigns=_xLaplaceSigns;
  int *yLaplaceSigns=_yLaplaceSigns;

  // Convert the feature vectors to a dense format
  // and get the dimensions
  int skip=(_surfLaplacians)?3:2;
  int xCount;
  for (xCount=0; (xCount<maxFeatures) && (x->index!=-1); ++xCount)
  {
    // Skip the position information and update the counters
    ++x; ++x;

    // Read the sign of laplacian
    if (_surfLaplacians)
    {
      xLaplaceSigns[xCount]=(int)(x->value);
      ++x;
    }

    // Initalize some temp value
    int tmp=skip + xCount*(_featureSize+skip) + 1; // skip because of position info, 1 because counting starts from 1

    // Read local features
    for (int i=0; i<_featureSize; ++i)
    {
      if (x->index == tmp + i )
      {
        _xDense[xCount][i] = x->value;
        ++x;
      }
      else
        _xDense[xCount][i] = 0.0;
    }
  }
  _xCount=xCount;

  int yCount;
  for (yCount=0; (yCount<maxFeatures) && (y->index!=-1); ++yCount)
  {
    // Skip the position information and update the counters
    ++y; ++y;

    // Read the sign of laplacian
    if (_surfLaplacians)
    {
      yLaplaceSigns[yCount]=(int)(y->value);
      ++y;
    }

    // Initalize some temp value
    int tmp=skip + yCount*(_featureSize+skip) + 1; // skip because of position info, 1 because counting starts from 1

    // Read local features
    for (int i=0; i<_featureSize; ++i)
    {
      if (y->index == tmp + i )
      {
        _yDense[yCount][i] = y->value;
        ++y;
      }
      else
        _yDense[yCount][i] = 0.0;
    }
  }
  _yCount=yCount;

  // Initalize the memory (distances and matches)
  double **distances=_distances;
  int **metrics=_metrics;
  double *matches=_matches;
  double **xDense=_xDense;
  double **yDense=_yDense;
  double distFactor=_distFactors[0];
  if (xCount<=yCount)
  {
    for (int i=0; i<xCount; ++i)
    {
      double *distancesRow=distances[i];
      int *metricsRow=metrics[i];
      matches[i]=-LARGE_VAL;
      double *xDenseI=xDense[i];
      int xLaplace=xLaplaceSigns[i];
      for (int j=0; j<yCount; ++j)
      {
        if (xLaplace!= yLaplaceSigns[j])
        {
          distancesRow[j] = LARGE_VAL/10;
          metricsRow[j] = 3;
        }
        else
        {
          distancesRow[j] = calculateInitDistance(xDenseI, yDense[j], distFactor);
          metricsRow[j] = 0;
        }
      }
    }
  }
  else
  {
    for (int i=0; i<yCount; ++i)
    {
      double *distancesRow=distances[i];
      int *metricsRow=metrics[i];
      matches[i]=-LARGE_VAL;
      double *yDenseI=yDense[i];
      int yLaplace=yLaplaceSigns[i];
      for (int j=0; j<xCount; ++j)
      {
        if (xLaplaceSigns[j]!=yLaplace)
        {
          distancesRow[j] = LARGE_VAL/10;
          metricsRow[j] = 3;
        }
        else
        {
          distancesRow[j] = calculateInitDistance(xDense[j], yDenseI, distFactor);
          metricsRow[j] = 0;
        }
      }
    }
  }

}


/** Compare function for qsort. */
int qsortCompare(const void *a, const void *b)
{
  double c = *(double*)a;
  double d = *(double*)b;

  if(c < d)
    return(-1);
  return(1);
}


/** Calculates the kernel value for two identical vectors (x and x). */
double matchIdentical(const svm_node *x)
{
  int maxMatches=_maxMatches;

  // Check if the dimension exceeds _maxMatches
  int skip=(_surfLaplacians)?3:2;
  int xCount;
  for (xCount=0; (xCount<maxMatches) && (x->index!=-1); ++xCount)
  {
    // Skip the position information
    ++x; ++x;

    // Skip the laplacian sign
    if (_surfLaplacians)
      ++x;

    // Initalize some temp value
    int tmp=skip + xCount*(_featureSize+skip) + 1; // 2 because of position info, 1 because counting starts from 1

    // Skip the values
    for (int i=0; i<_featureSize; ++i)
    {
      if (x->index == tmp + i )
        ++x;
    }
  }

  return ((double)xCount)/maxMatches;
}


/** Performs matching of two sets of local features. */
double doMatching()
{
  double **distances = _distances;
  int **metrics = _metrics;
  double *matches = _matches;
  int xCount=_xCount;
  int yCount=_yCount;
  double **xDense=_xDense;
  double **yDense=_yDense;
  double distFactors[4];
  distFactors[0]=_distFactors[0];
  distFactors[1]=_distFactors[1];
  distFactors[2]=_distFactors[2];
  distFactors[3]=_distFactors[3];

  if (xCount<=yCount)
  { // -------------------------------------------------------
    // distances [xCount, yCount]
    // metrics [xCount, yCount]
    // matches [xCount]
    // We look for xCount matches

    // Iterate until all matches found
    int matchesLeft=xCount;
    while(matchesLeft>0)
    {
      // Go through all the rows of the distances matrix
      for (int i=0; i<xCount; ++i)
      {
        // Cache pointers to the rows
        double *distancesRow=distances[i];
        int *metricsRow=metrics[i];
        double &match=matches[i];
        double *xDenseI=xDense[i];

        // Check if match for this row was already found
        if (match<0)
        { // No, match not found yet
          double min=LARGE_VAL;
          int minId=0;

          // Find min value in the row i
          // Iterate through all elements of the row
          for (int j=0; j<yCount; ++j)
          {
            /* NICE BUT SLOW VERSION
            double &distance = distancesRow[j];
            int &metric = metricsRow[j];
            // Check if this element is a possible minimum
            while(distance<min)
            {
              if (metric==PARTITION_COUNT-1)
              { //  This is a new minimum
                min = distance;
                minId = j;
                break;
              }
              // We don't know yet, get better distance
              ++metric;
              distance = calculateDistance(xDenseI, yDense[j], metric, distance);
            } // while(distance<min)
            */
            if (distancesRow[j]>=min)
              continue;
            if (metricsRow[j]==3)
            {
              min = distancesRow[j];
              minId = j;
            }
            else if (metricsRow[j]==2)
            {
              double distance;
              distance = calculateDistance(xDenseI, yDense[j], 3, distancesRow[j], distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            }
            else if (metricsRow[j]==1)
            {
              double distance;
              distance = calculateDistance(xDenseI, yDense[j], 2, distancesRow[j], distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=2;
                continue;
              }
              distance = calculateDistance(xDenseI, yDense[j], 3, distance, distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            }
            else
            {
              double distance;
              distance = calculateDistance(xDenseI, yDense[j], 1, distancesRow[j], distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=1;
                continue;
              }
              distance = calculateDistance(xDenseI, yDense[j], 2, distance, distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=2;
                continue;
              }
              distance = calculateDistance(xDenseI, yDense[j], 3, distance, distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            } // if
          } // for (int j=0; j<yCount; ++j)

          // Check if this is also a minimum for the column
          bool stillMin=true;
          double *yDenseMinId=yDense[minId];
          for(int j=0; (j<xCount) && (stillMin); ++j)
          {
            /* NICE BUT SLOW VERSION
            // Skip rows that were matched already
            if (matches[j]<0)
            {
              // Initialize temp. variables
              double &distance = distances[j][minId];
              int &metric = metrics[j][minId];
              // Check if this element can be smaller than minimum
              while(distance<min)
              {
                //                printf("A2\n");
                if (metric==PARTITION_COUNT-1)
                { //  Yes, there is a smaller element in the column
                  stillMin=false;
                  break;
                }
                // We don't know yet, get better distance
                ++metric;
                distance = calculateDistance(xDense[j], yDenseMinId, metric, distance);
              } // while(distance<min)
            */
            if (matches[j]>=0)
              continue;
            if (distances[j][minId]>=min)
              continue;
            if (metrics[j][minId]==3)
            {
              stillMin=false;
              // break;
            }
            else if (metrics[j][minId]==2)
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseMinId, 3, distances[j][minId], distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
                //break;
              }
            }
            else if (metrics[j][minId]==1)
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseMinId, 2, distances[j][minId], distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=2;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseMinId, 3, distance, distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
                //break;
              }
            }
            else
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseMinId, 1, distances[j][minId], distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=1;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseMinId, 2, distance, distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=2;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseMinId, 3, distance, distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
                // break;
              }
            } // if
            //}
          } // for(int j=0; (j<xCount) && (stillMin); ++j)

          // So, have we found a match?
          if (stillMin)
          { // Yes, it's a new match!
            match=min; // Remember the value
            for (int j=0; j<xCount; ++j) // Mark the column as already used
            {
              distances[j][minId]=LARGE_VAL;
              metrics[j][minId]=3;
            }
            --matchesLeft; // Decrese the number of matches to be found
          } //if (stillMin)

        } // if (matches[i]<0)
      } // for
    } // while

    // Get the number of matches we should consider
    int m = _maxMatches;
    if (m>xCount)
      m=xCount;

    // Sort the matches
    //    qsort((void*)matches, xCount, sizeof(double), qsortCompare);
    sort(matches,  matches+xCount); // Sort seems to be faster

    // Get the final sum
    double sum=0.0;
    for (int i=0; i<m; ++i)
    {
      if (matches[i]<LARGE_VAL/10.0)
      {
        sum+=exp(-_params->gamma*matches[i]);
      }
    }
    return sum/_maxMatches;
  }
  else
  {
    // -------------------------------------------------------
    // distances [yCount, xCount]
    // metrics [yCount, xCount]
    // matches [yCount]
    // We look for yCount matches

    // Iterate until all matches found
    int matchesLeft=yCount;
    while(matchesLeft>0)
    {
      // Go through all the rows of the distances matrix
      for (int i=0; i<yCount; ++i)
      {
        // Cache pointers to the rows
        double *distancesRow=distances[i];
        int *metricsRow=metrics[i];
        double &match=matches[i];
        double *yDenseI=yDense[i];

        // Check if match for this row was already found
        if (match<0)
        { // No, match not found yet
          double min=LARGE_VAL;
          int minId=0;

          // Find min value in the row i
          // Iterate through all elements of the row
          for (int j=0; j<xCount; ++j)
          {
            /* NICE BUT SLOW VERSION
            double &distance = distancesRow[j];
            int &metric = metricsRow[j];
            // Check if this element is a possible minimum
            while(distance<min)
            {
              //              printf("A1 %f %f\n", distance, min);
              if (metric==PARTITION_COUNT-1)
              { //  This is a new minimum
                min = distance;
                minId = j;
                break;
              }
              // We don't know yet, get better distance
              ++metric;
              distance = calculateDistance(xDense[j], yDenseI, metric, distance);
            } // while(distance<min)
            */
            if (distancesRow[j]>=min)
              continue;
            if (metricsRow[j]==3)
            {
              min = distancesRow[j];
              minId = j;
            }
            else if (metricsRow[j]==2)
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseI, 3, distancesRow[j], distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            }
            else if (metricsRow[j]==1)
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseI, 2, distancesRow[j], distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=2;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseI, 3, distance, distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            }
            else
            {
              double distance;
              distance = calculateDistance(xDense[j], yDenseI, 1, distancesRow[j], distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=1;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseI, 2, distance, distFactors);
              if (distance>=min)
              {
                distancesRow[j]=distance;
                metricsRow[j]=2;
                continue;
              }
              distance = calculateDistance(xDense[j], yDenseI, 3, distance, distFactors);
              distancesRow[j]=distance;
              metricsRow[j]=3;
              if (distance<min)
              {
                min = distance;
                minId = j;
              }
            }// if
          } // for (int j=0; j<xCount; ++j)

          // Check if this is also a minimum for the column
          bool stillMin=true;
          double *xDenseMinId=xDense[minId];
          for(int j=0; (j<yCount) && (stillMin); ++j)
          {
            /* NICE BUT SLOW VERSION
            // Skip rows that were matched already
            if (matches[j]<0)
            {
              // Initialize temp. variables
              double &distance = distances[j][minId];
              int &metric = metrics[j][minId];
              // Check if this element can be smaller than minimum
              while(distance<min)
              {
                //printf("A2\n");
                if (metric==PARTITION_COUNT-1)
                { //  Yes, there is a smaller element in the column
                  stillMin=false;
                  break;
                }
                // We don't know yet, get better distance
                ++metric;
                distance = calculateDistance(xDenseMinId, yDense[j], metric, distance);
              } // while(distance<min)
            */
            if (matches[j]>=0)
              continue;
            if (distances[j][minId]>=min)
              continue;
            if (metrics[j][minId]==3)
            {
              stillMin=false;
              //break;
            }
            else if (metrics[j][minId]==2)
            {
              double distance;
              distance = calculateDistance(xDenseMinId, yDense[j], 3, distances[j][minId], distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
               // break;
              }
            }
            else if (metrics[j][minId]==1)
            {
              double distance;
              distance = calculateDistance(xDenseMinId, yDense[j], 2, distances[j][minId], distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=2;
                continue;
              }
              distance = calculateDistance(xDenseMinId, yDense[j], 3, distance, distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
                //break;
              }
            }
            else
            {
              double distance;
              distance = calculateDistance(xDenseMinId, yDense[j], 1, distances[j][minId], distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=1;
                continue;
              }
              distance = calculateDistance(xDenseMinId, yDense[j], 2, distance, distFactors);
              if (distance>=min)
              {
                distances[j][minId]=distance;
                metrics[j][minId]=2;
                continue;
              }
              distance = calculateDistance(xDenseMinId, yDense[j], 3, distance, distFactors);
              distances[j][minId]=distance;
              metrics[j][minId]=3;
              if (distance<min)
              {
                stillMin=false;
                //break;
              }
            } // if
            // }
          } // for(int j=0; (j<yCount) && (stillMin); ++j)

          // So, have we found a match?
          if (stillMin)
          { // Yes, it's a new match!
            match=min; // Remember the value
            for (int j=0; j<yCount; ++j) // Mark the column as already used
            {
              distances[j][minId]=LARGE_VAL;
              metrics[j][minId]=3;
            }
            --matchesLeft; // Decrese the number of matches to be found
          } //if (stillMin)

        } // if (matches[i]<0)
      } // for
    } // while

    // Get the number of matches we should consider
    int m = _maxMatches;
    if (m>yCount)
      m=yCount;

    // Sort the matches
    //qsort((void*)matches, yCount, sizeof(double), qsortCompare);
    sort(matches, matches+yCount);

    // Get the final sum
    double sum=0.0;
    for (int i=0; i<m; ++i)
    {
      if (matches[i]<LARGE_VAL/10.0)
      {
        sum+=exp(-_params->gamma*matches[i]);
      }
    }
    return sum/_maxMatches;
  } // -------------------------------------------------------

} // function



double localkernel2(const svm_node *x, const svm_node *y, const svm_parameter& param)
{
  // Check if the feature sets are empty
  if(x->index == -1)
    return(exp(param.gamma));
  if(y->index == -1)
    return(exp(param.gamma));

  // Use the local kernel
  initParams(param);
  double val;
  if (x==y)
    val=matchIdentical(x);
  else
  {
    if ((_surfLaplacians) && (_surfEnhancements))
      setFeaturesLaplace(x, y);
    else
      setFeatures(x, y);
    val=doMatching();
  }

  return val;
}

