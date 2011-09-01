/**
 * $Id$
 */


#include "CBEntry.hh"




namespace P 
{


/************************************************************************************
 * Constructor/Destructor
 */
CBEntry::CBEntry(unsigned &_N, float _sqrThr)
 : N(_N), wi(0), sqrThr(_sqrThr)
{
}

CBEntry::CBEntry(unsigned &_N, float *d, unsigned size, unsigned vidx, unsigned kidx, float _sqrThr)
 : N(_N), sqrThr(_sqrThr)
{
  data.resize(size);

  for (unsigned i=0; i<size; i++)
    data[i] = d[i];

  idxViewKeys[vidx].push_back(kidx);

  wi = log((float)N/(float)Ni());
}

CBEntry::~CBEntry()
{
}


/****************************************************************************************/

/**
 * Insert and mean shift update
 */
void CBEntry::InsertMeanShift(vector<cv::Ptr<View> > &views, unsigned vidx, unsigned kidx, float invSqrSigma)
{  
  map<unsigned, vector<unsigned> >::iterator it;
  float weight, sumWeight=0, *d;
  vector<float> tmp = data;
  SetZero(); 

  // add new keypoint
  idxViewKeys[vidx].push_back(kidx);

  // update mean shift
  for (it=idxViewKeys.begin(); it!=idxViewKeys.end(); it++)
  {
    for (unsigned i=0; i<it->second.size(); i++)
    {
      d = &views[it->first]->descriptors(it->second[i],0);
      weight = exp(-invSqrSigma * DistSqr(&tmp[0], d, data.size()) );
      AddMul(d, weight);
      sumWeight += weight;
    }
  }  

  if (sumWeight > PMath::eps)
    Mul(1./sumWeight);

  // update threshold
  float cthr;
  for (it=idxViewKeys.begin(); it!=idxViewKeys.end(); it++)
  {
    for (unsigned i=0; i<it->second.size(); i++)
    {
      cthr = DistSqr( &data[0], &views[it->first]->descriptors(it->second[i],0), data.size() ) ;
      if (cthr > sqrThr) sqrThr = cthr;
    }
  }

  // update weight
  wi = log((float)N/(float)Ni());
  if (wi<0.001) wi=0.001;       // just a small offset for normalization
}


}












