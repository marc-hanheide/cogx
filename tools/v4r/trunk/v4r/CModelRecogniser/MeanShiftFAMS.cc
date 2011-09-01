/**
 * $Id$
 */


#include "MeanShiftFAMS.hh"

#define DEBUG



namespace P 
{



/********************** MeanShiftFAMS ************************
 * Constructor/Destructor
 */
MeanShiftFAMS::MeanShiftFAMS(Parameter _param)
 : param(_param), pilotfile(0)
{
  nDim = 0;
  nModes = 0;
}

MeanShiftFAMS::~MeanShiftFAMS()
{
}

/**
 * Convert data to FAMS structure
 */
void MeanShiftFAMS::ConvertData(FAMS *fams, double *data, unsigned num, unsigned dim)
{
  nDim = dim;
  int i;
  fams->n_ = (int)num;
  fams->d_ = (int)dim;

  fams->CleanPoints();
  
  // allocate and convert to integer
  double min, max;
  for (i=0, min=data[0], max=data[0]; i<(int)(num*dim); i++)
  {
     if (min > data[i])
        min = data[i];
     else if (max < data[i])
        max = data[i];
  }
  fams->minVal_ = (float)min;
  fams->maxVal_ = (float)max;

  fams->data_ = new unsigned short[num*dim];
  fams->rr_ = new double[dim];
  fams->hasPoints_ = 1;
  double deltaVal = max-min;

  if (deltaVal == 0) deltaVal = 1;
  for (i=0; i<(int)(num*dim); i++)
     fams->data_[i] = (unsigned short) (65535.0*(data[i]-min)/deltaVal);

  fams->dataSize_ = dim*sizeof(unsigned short);
  fams->points_ = new fams_point[num];
  unsigned short* dtempp;
  for (i=0, dtempp=fams->data_; i<(int)num; i++, dtempp+=dim)
  {
     fams->points_[i].data_ = dtempp;
     fams->points_[i].usedFlag_ = 0;
  }
}

void MeanShiftFAMS::CopyModes(FAMS *fams, unsigned &numModes)
{
  if (fams->nsel_ < 1)
  {
    modes.clear();
    nModes = numModes=0;
    return;
  }

  int i,j,idx;
  idx = 0;
  modes.resize(fams->npm_*fams->d_);
  for (i=0; i<fams->npm_; i++)
  {
     for (j=0; j<fams->d_; j++, idx++)
     {
        modes[idx] = fams->prunedmodes_[idx]*(fams->maxVal_-fams->minVal_)/65535.0 + fams->minVal_;
     }
  }

  nModes = numModes = fams->npm_;
}






/****************************** PUBLIC *************************/
/**
 * Mean shift clustering
 */
double *MeanShiftFAMS::Operate(double *data, unsigned num, unsigned dim, unsigned &numModes, unsigned *labels)
{
  if (num<=1)
  {
    modes.clear();
    numModes=0;
    return 0;
  }
    
  FAMS cfams(param.no_lsh);

  // load points
  ConvertData(&cfams, data,num,dim);

  // find K L (if necessary)
  if (param.find_kl)
  {
    cfams.FindKL(param.Kmin, param.Kmax, param.Kjump, param.Lmax, param.k_neigh, param.width, param.epsilon, param.K, param.L);
    #ifdef DEBUG
    printf("Found K = %d L = %d (write them down)\n", param.K, param.L);
    #endif
  }

  cfams.RunFAMS(param.K, param.L, param.k_neigh, param.percent, param.jump, param.width, pilotfile);

  // save the pruned modes 
  CopyModes(&cfams,numModes);

  // assign labels
  if (labels!=0 && nModes>0 && num > 0)
  {
    unsigned idx;
    double distsqr, minsqr;
    double *m, *d = data;

    for (unsigned i=0; i<num; i++, d+=dim)
    {
      minsqr=DBL_MAX;
      m = &modes[0];

      for (unsigned j=0; j<nModes; j++,m+=dim)
      {
        distsqr = DistSqr(d,m,dim);
        if (distsqr < minsqr)
        {
          minsqr = distsqr;
          idx = j;
        }
      }
      
      labels[i] = idx;
    }
  }

  return &modes[0];
}

/**
 * FAMS mean shift clustering
 */
void MeanShiftFAMS::Cluster(cv::Mat_<double> &samples, vector<vector<unsigned> > &clusters)
{
  if (samples.rows<=1)
  {
    if (samples.rows>0)
    {
      clusters.resize(1);
      for (unsigned i=0; i<samples.rows; i++)
        clusters[0].push_back(i);
    }
    return;
  }
  
  cv::Mat data; 
  FAMS cfams(param.no_lsh);

  // align points and convert
  if (samples.isContinuous())
    data = samples;
  else
    samples.copyTo(data);
   
  ConvertData(&cfams, data.ptr<double>(0), data.rows, data.cols);

  // find K L (if necessary)
  if (param.find_kl)
  {
    cfams.FindKL(param.Kmin, param.Kmax, param.Kjump, param.Lmax, param.k_neigh, param.width, param.epsilon, param.K, param.L);
    #ifdef DEBUG
    printf("Found K = %d L = %d (write them down)\n", param.K, param.L);
    #endif
  }

  cfams.RunFAMS(param.K, param.L, param.k_neigh, param.percent, param.jump, param.width, pilotfile);

  // save the pruned modes 
  CopyModes(&cfams,nModes);

  // assign labels
  if (nModes>0)
  {
    unsigned idx;
    double distsqr, minsqr;
    double *m, *d = data.ptr<double>(0);
    clusters.resize(nModes);

    for (unsigned i=0; i<data.rows; i++, d+=data.cols)
    {
      idx=UINT_MAX;
      minsqr=DBL_MAX;
      m = &modes[0];

      for (unsigned j=0; j<nModes; j++,m+=data.cols)
      {
        distsqr = DistSqr(d,m,data.cols);
        if (distsqr < minsqr)
        {
          minsqr = distsqr;
          idx = j;
        }
      }
      
      if (idx!=UINT_MAX) clusters[idx].push_back(i); 
    }
  }

}


}

