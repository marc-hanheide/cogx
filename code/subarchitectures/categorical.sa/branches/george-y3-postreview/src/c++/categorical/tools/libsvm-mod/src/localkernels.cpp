#include "localkernels.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int kernelEvals= 0;

/* memory pointers for localkernel */
static double* af= NULL;
static double* bf= NULL;
static double* amean= NULL;
static double* bmean= NULL;
static double** avec= NULL;
static double** bvec= NULL;
static double** apos= NULL;
static double** bpos= NULL;
static double** corr= NULL;
static int *alaplace = NULL;
static int *blaplace = NULL;

const double SMALL = -10000000.0;


/* memory pointer for histogram distance extension */
static double* distances= NULL;
static double** distHistsA= NULL;
static double** distHistsB= NULL;

/* memory pointer for speed up for maximum localization */

// a flat copy of the correlation values including the corresponding indices i and j
static double* correlation_flat= NULL;

// pointers to the correlationvalues in correlation_flat;
// used to initialize corr_ptrs
// (which gets messed by the sorting algoeithm)
static double** corr_ptrs_init;

// vector with sorted pointers which point to the correlation values in correlation_flat
static double** corr_ptrs= NULL;

// stores the used indices to achieve one-to-one matching
static int* used_links_i= NULL;

// stores the used indices to achieve one-to-one matching
static int* used_links_j= NULL;

// allocate an array
double** allocDoubleArray(int dim1, int dim2) {
  int i;
  double** array= (double**) malloc(sizeof(double)*dim1);
  for(i=0;i<dim1;i++)
    {
      array[i]= (double*) malloc(sizeof(double)*dim2);
    }
  return array;
}

// allocate an array with continious memory mapping
double** allocContiniousDoubleArray(int dim1, int dim2) {
  int i;
  double** array= (double**) malloc(sizeof(double)*dim1);
  array[0]= (double*) malloc(sizeof(double)*dim1*dim2);
  for(i=1;i<dim1;i++)
    {
      array[i]= *array + dim2 * i;
    }
  return array;
}

// compute the corration matrix (corr) of the local features
inline void rbfOfCorrelation(double** corr, double** avec, double** bvec, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double dist;

  // compute variances
  for (int i=0;i<countera;i++){
    af[i]=0.0;
    for (int k=0;k<FEATUREDIM;k++) af[i]+=(avec[i][k]-amean[i])*(avec[i][k]-amean[i]);
  }
  for (int j=0;j<counterb;j++){
    bf[j]=0.0;
    for (int k=0;k<FEATUREDIM;k++) bf[j]+=(bvec[j][k]-bmean[j])*(bvec[j][k]-bmean[j]);
  }

  // compute correlation from mean and variances
  for (int i=0;i<countera;i++){
    for (int j=0;j<counterb;j++){
      if (af[i]!=0.0 && bf[j]!=0.0){
	dist=0.0;
	for (int k=0;k<FEATUREDIM;k++) dist+=(avec[i][k]-amean[i])*(bvec[j][k]-bmean[j]);
	dist/=sqrt(af[i]*bf[j]);
      }
      else dist=-1.0;
      // total similarity!, RBF kernel!
      corr[i][j]=param.gamma*(dist-1);
      //printf("%d %d -> %f %f at %.1f,%.1f %.1f,%.1f: %f\n",i,j,dist,pixdist,apos[i][0],apos[i][1],bpos[j][0],bpos[j][1],corr[i][j]);
    }
  }
}

// add the "feature distances" position information
inline void addPositionInfoSimple(double** corr, double** apos, double** bpos, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double gamma= 1.0 / (2*param.sigmaPos*param.sigmaPos);
  double xdiff, ydiff;
  double dist;

  for (int i=0;i<countera;i++)
    {
      for (int j=0;j<counterb;j++)
	{
	  xdiff= apos[i][0]-bpos[j][0];
	  ydiff= apos[i][1]-bpos[j][1];
	  dist= xdiff*xdiff + ydiff*ydiff;
	  corr[i][j]+= -gamma * dist;
	}
    }
}

// compare dereferenced elements
inline int compareElements(const void* a, const void* b)
{
  double A= *((double*) a);
  double B= *((double*) b);

  if (A<B) return -1;
  else return 1;
}

// add "distance histogram" position information
inline void addPositionInfoDistHist(double** corr, double** apos, double** bpos, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double xdiff, ydiff, maximum, binSize, accBin;
  int binIndex;

  // computing distance histograms for x
  for (int ref=0;ref<countera;ref++)
    {
      for(int i= 0;i<countera;i++)
	{
	  // compute feature distance
	  xdiff= apos[i][0]-apos[ref][0];
	  ydiff= apos[i][1]-apos[ref][1];
	  distances[i]= sqrt(xdiff*xdiff + ydiff*ydiff);
	}
      qsort((void*) distances, countera, sizeof(double), compareElements);

      // restrict to the param.distHistNeighbourhood nearest features
      // (make the constraint local)
      int possibleNeighbourhood= param.distHistNeighbourhood;
      if (possibleNeighbourhood>countera)
	possibleNeighbourhood= countera;

      // compute histograms
      maximum= distances[possibleNeighbourhood-1];
      binSize= maximum/param.distHistBins;
      accBin= binSize;
      binIndex= 0;
      for(int i= 0;i<possibleNeighbourhood;i++)
	{
	  if(distances[i]>accBin)
	    {
	      distHistsA[ref][binIndex]= i+1;
	      while(distances[i]>accBin)
		{
		  accBin+= binSize;
		  binIndex++;
		}
	    }
	}
      distHistsA[ref][param.distHistBins-1]= possibleNeighbourhood;
    }

  // computing distance histograms for y
  // (similar to computation above for x)
  for (int ref=0;ref<counterb;ref++)
    {
      for(int i= 0;i<counterb;i++)
	{
	  xdiff= bpos[i][0]-bpos[ref][0];
	  ydiff= bpos[i][1]-bpos[ref][1];
	  distances[i]= sqrt(xdiff*xdiff + ydiff*ydiff);
	}
      qsort((void*) distances, counterb, sizeof(double), compareElements);


      int possibleNeighbourhood= param.distHistNeighbourhood;
      if (possibleNeighbourhood>counterb)
	possibleNeighbourhood= counterb;

      maximum= distances[possibleNeighbourhood-1];
      binSize= maximum/param.distHistBins;
      accBin= binSize;
      binIndex= 0;
      for(int i= 0;i<possibleNeighbourhood;i++)
	{
	  if(distances[i]>accBin)
	    {
	      distHistsB[ref][binIndex]= i+1;
	      while(distances[i]>accBin)
		{
		  accBin+= binSize;
		  binIndex++;
		}
	    }
	}
      distHistsB[ref][param.distHistBins-1]= possibleNeighbourhood;
    }

  double chiSquared, diff;

  // compare the histograms with chi-square kernel
  for (int i=0;i<countera;i++)
    {
      for (int j=0;j<counterb;j++)
	{
	  chiSquared= 0;
	  for(int bin=0;bin<param.distHistBins;bin++)
	    {
	      diff= distHistsA[i][bin] - distHistsB[j][bin];
	      chiSquared+= diff*diff/(distHistsA[i][bin] + distHistsB[j][bin]);
	    }
	  corr[i][j]+= -param.sigmaPos * chiSquared;
	}
    }
}


// add "distance profile" position information
inline void addPositionInfoDistHist_L2(double** corr, double** apos, double** bpos, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double xdiff, ydiff;

  // computing distances for x
  for (int ref=0;ref<countera;ref++)
    {
      for(int i= 0;i<countera;i++)
	{
	  // compute feature distances
	  xdiff= apos[i][0]-apos[ref][0];
	  ydiff= apos[i][1]-apos[ref][1];
	  distances[i]= sqrt(xdiff*xdiff + ydiff*ydiff);
	}
      qsort((void*) distances, countera, sizeof(double), compareElements);

      // normalize distances with the largest distance
      // to obtain more scale robustness
      for(int i=0; i<countera; i++)
	{
	  distances[i]/= distances[counterb-1];
	}

      for(int i= 0;i<countera;i++)
	{
	  distHistsA[ref][i]= distances[i];
	}

    }

  // computing distances for y
  for (int ref=0;ref<counterb;ref++)
    {
      for(int i= 0;i<counterb;i++)
	{
	  xdiff= bpos[i][0]-bpos[ref][0];
	  ydiff= bpos[i][1]-bpos[ref][1];
	  distances[i]= sqrt(xdiff*xdiff + ydiff*ydiff);
	}
      qsort((void*) distances, counterb, sizeof(double), compareElements);

      // normalize distances with the largest distance
      // to obtain more scale robustness
      for(int i=0; i<counterb; i++)
	{
	  distances[i]/= distances[counterb-1];
	}


      for(int i= 0;i<counterb;i++)
	{
	  distHistsB[ref][i]= distances[i];
	}

    }

  // make the constraint local by only considering the
  // param.distHistNeighbourhood nearest feature
  int commonSupport= param.distHistNeighbourhood;
  if (countera<commonSupport)
    commonSupport= countera;
  if (counterb<commonSupport)
    commonSupport= counterb;

  double sumOfSqDiff, diff;

  // compare the distance profiles with an rbf-kernel
  for (int i=0;i<countera;i++)
    {
      for (int j=0;j<counterb;j++)
	{
	  sumOfSqDiff= 0;
	  for(int bin=0;bin<commonSupport;bin++)
	    {
	      diff= distHistsA[i][bin] - distHistsB[j][bin];
	      sumOfSqDiff+= diff*diff;
	    }
	  corr[i][j]+= -param.sigmaPos * sumOfSqDiff;
	}
    }
}

// chi-square metric for the local kernel (not used, but usefull for local histograms)
inline void rbfOfChiSquared(double** corr, double** avec, double** bvec, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double dist;

  for (int i=0;i<countera;i++)
    {
      for (int j=0;j<counterb;j++)
	{
	  dist=0.0;
	  for (int k=0;k<FEATUREDIM;k++)
	    {
	      double sum= fabs(avec[i][k] + bvec[j][k]);
	      double diff= avec[i][k] - bvec[j][k];
	      dist+= (sum != 0.0) * (diff*diff/(sum + (sum == 0) ));
	    }
	  corr[i][j]=-param.gamma * dist;
	}
    }
}


void printDoubleMatrix2(double**m, int s1, int s2)
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


// local kernel with L2 metrix; used for SIFT and SURF features
inline void rbfOfSquaredDist(double** corr, double** avec, double** bvec, double* amean, double* bmean, double* af, double* bf, int FEATUREDIM, const svm_parameter& param, int countera, int counterb)
{
  double dist;
  double diff;

  for (int i=0;i<countera;i++)
    {
      for (int j=0;j<counterb;j++)
	{
	  dist=0.0;
	  for (int k=0;k<FEATUREDIM;k++)
	    {
	      diff= avec[i][k] - bvec[j][k];
	      dist+= diff*diff;
	    }
	  corr[i][j]=-param.gamma * dist;
	}
    }
}


// local kernel with L2 metrix and fast matching due to laplacian sign; used for SURF features
inline void rbfOfSquaredDistLaplace(double** corr, double** avec, double** bvec, double* amean, 
                                    double* bmean, double* af, double* bf, int FEATUREDIM, 
                                    const svm_parameter& param, int countera, int counterb,
                                    int *alap, int *blap)
{
  double dist;
  double diff;

  for (int i=0;i<countera;i++)
  {
    for (int j=0;j<counterb;j++)
    {
      if (alap[i]!=blap[j])
      {
        corr[i][j]= SMALL/10.0;
      }
      else
      {
        dist=0.0;
        for (int k=0;k<FEATUREDIM;k++)
        {
          diff= avec[i][k] - bvec[j][k];
          dist+= diff*diff;
        }
        corr[i][j]=-param.gamma * dist;
      }
    }
  }
}


int mySort(const void *a, const void *b) {
  double c = *(double*)a;
  double d = *(double*)b;

  if(c > d) return(-1);
  return(1);
}

// speed up for the features matching (by Christian Schuld)
inline double getMaxSum_ninjaStyle(int NUMMAX, double SIMTHRESH, double** corr, int countera, int counterb, const svm_parameter& param) {
  int i, j, k, maxj, nomax, corrcount;
  double max;

  // number of rows < number of columns? (or was it vice versa? :-)

  if(countera <= counterb) {

    for(i=0;i<counterb;i++)
      correlation_flat[i] = SMALL;

    corrcount = countera;
    while(corrcount > 0) {

      for(i=0;i<countera;i++) {

	// already calculated?

	if(correlation_flat[i] == SMALL) {

	  // nope, find max for this feature

	  max = SMALL;
	  maxj = 0;
	  for(j=0;j<counterb;j++) {
	    if(corr[i][j] > max) {
	      max = corr[i][j];
	      maxj = j;
	    }
	  }

	  // found maximum correlation for this feature.
	  // but is it a maximum for the other feature also?

	  nomax = false;
	  for(j=0;j<countera;j++) {
	    if(corr[j][maxj] > max) {
	      nomax = true;
	      break;
	    }
	  }

	  if(!nomax) {

	    // woah! it was a maximum!
	    correlation_flat[i] = max;

	    // mark both as used
	    for(k=0;k<countera;k++) corr[k][maxj] = SMALL;
	    for(j=0;j<counterb;j++) corr[i][j] = SMALL;
	    corrcount--;
	  }

	}
      }
    }

  } else {

    for(i=0;i<countera;i++)
      correlation_flat[i] = SMALL;

    corrcount = counterb;
    while(corrcount > 0) {

      for(i=0;i<counterb;i++) {

	// already calculated?

	if(correlation_flat[i] == SMALL) {

	  // nope, find max for this feature

	  max = SMALL;
	  maxj = 0;
	  for(j=0;j<countera;j++) {
	    if(corr[j][i] > max) {
	      max = corr[j][i];
	      maxj = j;
	    }
	  }

	  // found maximum correlation for this feature.
	  // but is it a maximum for the other feature also?

	  nomax = false;
	  for(j=0;j<counterb;j++) {
	    if(corr[maxj][j] > max) {
	      nomax = true;
	      break;
	    }
	  }

	  if(!nomax) {

	    // woah! it was a maximum!
	    correlation_flat[i] = max;

	    // mark both as used
	    for(k=0;k<counterb;k++) corr[maxj][k] = SMALL;
	    for(j=0;j<countera;j++) corr[j][i] = SMALL;
	    corrcount--;
	  }

	}
      }
    }

  }

  double correlationscore = 0.0;
  if (countera<counterb)
    qsort((void*)correlation_flat, countera, sizeof(double), mySort);
  else
    qsort((void*)correlation_flat, counterb, sizeof(double), mySort);

  int oldNUMMAX= NUMMAX;

  if (countera<NUMMAX)
    NUMMAX= countera;
  if (counterb<NUMMAX)
    NUMMAX= counterb;

  int n_summed_elements= 0;
  for(i=0;i<NUMMAX;i++)
    if(correlation_flat[i] >= SIMTHRESH)
      {
	correlationscore += exp(correlation_flat[i]);
	n_summed_elements++;
      }

  if (oldNUMMAX!=0)
    return(correlationscore/oldNUMMAX);
  else
    return(0.0);
}

// feature matching as it was implemented by Christian Wallraven
inline double getMaxSum_oldStyle (int NUMMAX, double SIMTHRESH, double** corr, int countera, int counterb, const svm_parameter& param)
{
  double max;
  int maxi, maxj;

  double correlationscore=0.0;
  int counter=0;
  for (int k=0;k<NUMMAX;k++){
    max=-10000000.0;
    maxi=maxj=0;
    int i= 0;
    for(double* it= corr[0]; it<corr[0]+param.n_features*param.n_features; it++)
      {
	if(*it > max)
	  {
	    max= *it;
	    maxi= i/param.n_features;;
	    maxj= i%param.n_features;
	  }
	++i;
      }

    // we have found a maximum in the correlation matrix,
    // now remove the corresponding feature pair from further consideration
    // (ONE-TO-ONE matching!!)
    for (int i=0;i<countera;i++) corr[i][maxj]=-1000000.0;
    for (int j=0;j<counterb;j++) corr[maxi][j]=-1000000.0;
    // again, this threshold does not seem to have much influence
    if (max>=SIMTHRESH){
      //	  printf("Maximum distance: %f -> %f at %d %d\n", (float) max, (float) exp(param.gamma*max), maxi, maxj);
      correlationscore+=exp(max);
      counter++;
    }
  }
  //printf("%f: %d -> %f\n",param.gamma,counter, correlationscore);

  //	printf("oldStyle found %d elements\n", counter);

  // RETURN similarity values!!
  if (counter!=0) return(correlationscore/counter);
  else return(0.0);
}

inline int compareCorrElements(const void* a, const void* b)
{
  double corrA= **((double**) a);
  double corrB= **((double**) b);

  if (corrA>corrB) return -1;
  else return 1;
}

// speed up of the matching by using quicksort (not used and not extensively tested)
inline double getMaxSum_quickSortStyle (double** corr, int countera, int counterb, const svm_parameter& param)
{
  int FEATURES= param.n_features;
  int NUMMAX= param.nummax;
  double SIMTHRESH= param.simthresh;

  // initializing helping vectors
  for(int i= 0; i<FEATURES*FEATURES; i++)
    {
      corr_ptrs[i]= corr_ptrs_init[i];
    }

  for(int i= 0; i<FEATURES; i++)
    {
      for(int j= 0; j<FEATURES; j++)
	{
	  correlation_flat[i*FEATURES*3 + j*3 ]= corr[i][j];
	}
    }

  for(int i= 0; i<FEATURES; i++)
    {
      used_links_i[i]= 0;
      used_links_j[i]= 0;
    }

  qsort((void*) corr_ptrs, FEATURES*FEATURES, sizeof(double*), compareCorrElements);

  double correlationscore=0.0;
  int counter=0;
  int skipped= 0;
  double** act_ptr= corr_ptrs;
  for (int k=0;k<NUMMAX;k++){
    // skip all entries with already matched features
    while(used_links_i[(int)(*act_ptr)[1]] || used_links_j[(int)(*act_ptr)[2]])
      {
	++act_ptr;
	skipped++;
      }

    // we have found a maximum in the correlation matrix,
    // now remove the corresponding feature pair from further consideration
    // (ONE-TO-ONE matching!!)
    used_links_i[(int)(*act_ptr)[1]]= 1;
    used_links_j[(int)(*act_ptr)[2]]= 1;

    // again, this threshold does not seem to have much influence
    if ((*act_ptr)[0] >=SIMTHRESH){

      correlationscore+=exp((*act_ptr)[0]);
      counter++;
    }
  }
  if (counter!=0) return(correlationscore/counter);
  else return(0.0);
}

double localkernel1(const svm_node *x, const svm_node *y,
		    const svm_parameter& param)
{

  double k1=localkernel1_hat(x, y, param);
  //if (kernelEvals/100 == 1.0*kernelEvals/100) printf("Kernelevaluations: %d\n", kernelEvals);
  kernelEvals++;

  return k1;
}

double localkernel1_hat(const svm_node *x, const svm_node *y,
			const svm_parameter& param)
{
  const svm_node *ai,*bj;
  int i,j,counter,countera,counterb;

  /* read parameters from struct */
  int FEATURES= param.n_features;
  int FEATUREDIM= param.featuredim;
  int NUMMAX= param.nummax;
  double SIMTHRESH= param.simthresh;
  int surfLaplacians = (param.surfLaplacians==1);

  if (af == NULL)
    {
      // Allocating temporary memory for local kernel evaluations
      af= (double*) malloc(sizeof(double)*FEATURES);
      bf= (double*) malloc(sizeof(double)*FEATURES);
      amean= (double*) malloc(sizeof(double)*FEATURES);
      bmean= (double*) malloc(sizeof(double)*FEATURES);
      avec= allocDoubleArray(FEATURES, FEATUREDIM);
      bvec= allocDoubleArray(FEATURES, FEATUREDIM);
      apos= allocDoubleArray(FEATURES, 2);
      bpos= allocDoubleArray(FEATURES, 2);
      corr= allocContiniousDoubleArray(FEATURES, FEATURES);
      alaplace= (int*) malloc(sizeof(int)*FEATURES);
      blaplace= (int*) malloc(sizeof(int)*FEATURES);

      // Allocating temporary memory for histogram distance extension
      distances= (double*) malloc(sizeof(double)*param.n_features);
      distHistsA= allocDoubleArray(FEATURES, FEATURES);
      distHistsB= allocDoubleArray(FEATURES, FEATURES);

      // Allocationg temporary memory for speed-ups
      correlation_flat= (double*) malloc(sizeof(double)*FEATURES*FEATURES*3);
      corr_ptrs_init= (double**) malloc(sizeof(double*)*FEATURES*FEATURES);
      corr_ptrs= (double**) malloc(sizeof(double*)*FEATURES*FEATURES);
      used_links_i=  (int*) malloc(sizeof(int)*FEATURES);
      used_links_j=  (int*) malloc(sizeof(int)*FEATURES);

      // Initializing temporary memory for speed-ups
      for(int i= 0; i<FEATURES; i++)
	{
	  for(int j= 0; j<FEATURES; j++)
	    {
	      correlation_flat[i*FEATURES*3 + j*3 + 1]= (double) i;
	      correlation_flat[i*FEATURES*3 + j*3 + 2]= (double) j;
	      corr_ptrs_init[i*FEATURES + j]= correlation_flat + i*FEATURES*3 + j*3;
	    }
	}
    }

  // initial poisioning of correlation array
  for(double* it= corr[0]; it<corr[0]+FEATURES*FEATURES; it++)
    {
      *it= -99999;
    }

  ai=x;
  bj=y;
  counter=0;

  // short-cut for empty feature sets
  if(ai->index == -1) return(exp(param.gamma));
  if(bj->index == -1) return(exp(param.gamma));

  //read in position and feature vectors and compute mean
  ai=x;
  bj=y;

  // short-cut for identical feature sets
  // REMOVED BECAUSE THE KERNEL WEIGHTS THE OUTPUT VALUE DEPENDING
  // ON THE NUMBER OF INTEREST POINTS IN THE FEATURE VECTORS
  // AS A RESULT, K(x,x) IS NOT ALWAYS 1!
  //  if (ai==bj) return(1.0);

  countera=counterb=0;

  // read in sparse data structur
  // (to speed up computation, the sparse data structure
  // is temporarily transfered in a dense one)
  int skip=(surfLaplacians)?3:2;
  for (i=0;i<FEATURES;i++)
  {
    // read in positioni of feature
    if (ai->index!=-1)
    {
      countera++;
      apos[i][0]=ai->value;ai++;
      apos[i][1]=ai->value;ai++;
      if (surfLaplacians) // Read the sign of laplacian
      {
        alaplace[i]=(int)(ai->value);
        ai++;
      }
    }
    if (bj->index!=-1)
    {
      counterb++;
      bpos[i][0]=bj->value;bj++;
      bpos[i][1]=bj->value;bj++;
      if (surfLaplacians) // Read the sign of laplacian
      {
        blaplace[i]=(int)(bj->value);
        bj++;
      }
    }

    // read feature information
    amean[i]=bmean[i]=0.0;
    for ( j=0;j<FEATUREDIM;j++ )
    {
      if ( ai->index == skip + j + ( countera-1 ) * ( FEATUREDIM+skip ) + 1 )
      {
        avec[i][j]= ai->value;
        ++ai;
      }
      else
        avec[i][j]= 0;

      if ( bj->index == skip + j + ( counterb-1 ) * ( FEATUREDIM+skip ) + 1 )
      {
        bvec[i][j]=bj->value;
        ++bj;
      }
      else
        bvec[i][j]= 0;

      if ( ai->index != -1 )
      {
        amean[i]+=avec[i][j];
      }
      if ( bj->index != -1 )
      {
        bmean[i]+=bvec[i][j];
      }
    }

    // already compute mean of local feature vector
    amean[i]/=FEATUREDIM;
    bmean[i]/=FEATUREDIM;
  }


  // choose the selected inner local kernel type
  // (which should be chosen according to the local feature type
  // (jet, histograms, sift)
  if ( param.localKernelType == 0 )
  {
    rbfOfCorrelation ( corr,
                      avec, bvec,
                      amean, bmean,
                      af, bf,
                      FEATUREDIM,
                      param,
                      countera, counterb );
  }
  else if ( param.localKernelType == 1 )
  {
    rbfOfChiSquared ( corr,
                      avec, bvec,
                      amean, bmean,
                      af, bf,
                      FEATUREDIM,
                      param,
                      countera, counterb );
  }
  else if ( param.localKernelType == 2 )
  {
    if ((surfLaplacians) && (param.surfEnhancements==1))
    {
      rbfOfSquaredDistLaplace ( corr,
                                avec, bvec,
                                amean, bmean,
                                af, bf,
                                FEATUREDIM,
                                param,
                                countera, counterb,
                                alaplace, blaplace );
    }
    else
    {
      rbfOfSquaredDist ( corr,
                        avec, bvec,
                        amean, bmean,
                        af, bf,
                        FEATUREDIM,
                        param,
                        countera, counterb );
    }
  }

  // add position information to corr if desired
  if (param.posType == 0)
    {
      addPositionInfoSimple(corr,
			    apos, bpos,
			    amean, bmean,
			    af, bf,
			    FEATUREDIM,
			    param,
			    countera, counterb);
    }
  else if (param.posType == 1)
    {
      addPositionInfoDistHist(corr,
			      apos, bpos,
			      amean, bmean,
			      af, bf,
			      FEATUREDIM,
			      param,
			      countera, counterb);
    }
  else if (param.posType == 2)
    {
      addPositionInfoDistHist_L2(corr,
				 apos, bpos,
				 amean, bmean,
				 af, bf,
				 FEATUREDIM,
				 param,
				 countera, counterb);
    }

  // compute max and sum for NUMMAX most important feature
  double kernelValue= getMaxSum_ninjaStyle(NUMMAX,
					   SIMTHRESH,
					   corr,
					   countera, counterb,
					   param);

  //  double kernelValueOld= getMaxSum_oldStyle(NUMMAX, SIMTHRESH, ncorr, countera, counterb, param);
  //  double kernelValue= getMaxSum_quickSortStyle (corr, countera, counterb, param);

  return kernelValue;
}

