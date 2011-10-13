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
 : sqrThr(_sqrThr), sqrSigma(0.), N(_N), wi(0)
{
}

CBEntry::CBEntry(unsigned &_N, float *d, unsigned size, unsigned idxObject, unsigned idxView,
                 unsigned idxKey, float _sqrThr)
 : sqrThr(_sqrThr), sqrSigma(0.), N(_N)
{
  data.resize(size);

  for (unsigned i=0; i<size; i++)
    data[i] = d[i];

  occurrences.push_back(Occurrence(idxObject,idxView,idxKey));
  objects.insert(idxObject);

  wi = log((float)N/(float)Ni());
  if (wi<0.001) wi=0.001;       // just a small offset for normalization
}

CBEntry::~CBEntry()
{
}


/****************************************************************************************/

/**
 * Insert and mean shift update
 */
void CBEntry::InsertMeanShift(std::vector<cv::Ptr<CModel> > &objs, unsigned oidx, unsigned vidx, unsigned kidx, float invSqrSigma)
{  
  float weight, sumWeight=0, *d;
  std::vector<float> tmp = data;
  SetZero(); 

  // add new keypoint
  objects.insert(oidx);
  occurrences.push_back( Occurrence(oidx,vidx,kidx) );

  // update mean shift
  for (unsigned i=0; i<occurrences.size(); i++)
  {
    Occurrence &occ = occurrences[i];
    d = &objs[occ.object]->views[occ.view]->descriptors(occ.key,0);
    weight = exp(-invSqrSigma * DistSqr(&tmp[0], d, data.size()) );
    AddMul(d, weight);
    sumWeight += weight;
  }  

  if (sumWeight > PMath::eps)
    Mul(1./sumWeight);

  // update threshold
  float cthr;
  for (unsigned i=0; i<occurrences.size(); i++)
  {
    Occurrence &occ = occurrences[i];
    cthr= DistSqr(&data[0], &objs[occ.object]->views[occ.view]->descriptors(occ.key,0), data.size());
    if (cthr > sqrThr) sqrThr = cthr;
  }

  // update weight
  wi = log((float)N/(float)Ni());
  if (wi<0.001) wi=0.001;       // just a small offset for normalization
}


/**
 * InsertMean
 */
bool CBEntry::InsertMean(std::vector<cv::Ptr<CModel> > &objs, unsigned oidx, unsigned vidx, unsigned kidx, float thrSqrSigma)
{
  if (occurrences.size()==0)
    return false;

  float combSqrSigma = CombinedSqrSigma(&objs[oidx]->views[vidx]->descriptors(kidx, 0), data.size());

  if (combSqrSigma < thrSqrSigma)
  {
    objects.insert(oidx);

    sqrSigma = combSqrSigma;

    Mul(occurrences.size());
    Add(&objs[oidx]->views[vidx]->descriptors(kidx, 0));
    Mul( 1./(occurrences.size()+1.) );

    occurrences.push_back( Occurrence(oidx,vidx,kidx) );


    // update weight
    wi = log((float)N/(float)Ni());
    if (wi<0.001) wi=0.001;       // just a small offset for normalization
  
    return true;
  }

  return false;

}



}












