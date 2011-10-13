/**
 * $Id$
 * Johann Prankl, 2011-03-31
 * prankl@acin.tuwien.ac.at
 */


#include "MeanShiftCodebook.hh"

#define DEBUG


namespace P 
{

static const bool CmpMatchesDec(const cv::Ptr<ObjectMatches>& a, const cv::Ptr<ObjectMatches>& b)
{
  return (a->conf > b->conf);
}



MeanShiftCodebook::MeanShiftCodebook(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param)
 : optimized(false), N(0), param(_param)
{
  matcher = descMatcher;
}

MeanShiftCodebook::~MeanShiftCodebook()
{
}




/************************************** PRIVATE ************************************/





/************************************** PUBLIC ************************************/
/**
 * Clear the tree and delete all view links
 */
void MeanShiftCodebook::clear()
{ 
  cbEntries.clear();
  descriptors = cv::Mat();
}

/**
 * Optimize
 * for now just copy descriptors to a dense matrix
 */
void MeanShiftCodebook::Optimize()
{
  if (!descriptors.isContinuous())
  {
    cv::Mat_<float> tmp(descriptors.rows,descriptors.cols);
    descriptors.copyTo(tmp);
    descriptors = tmp;
  }

  if (param.useFlann)
  {
    if (flannMatcher.empty())
      flannMatcher = new cv::FlannBasedMatcher(
                     new cv::flann::KDTreeIndexParams(param.numTrees), 
                     new cv::flann::SearchParams(param.searchDepth) );
    
    vector<cv::Mat> descs;
    descs.push_back(descriptors);

    flannMatcher->clear();
    flannMatcher->add(descs);
    flannMatcher->train();

    optimized = true;
  }
}

/**
 * Insert a view to the meanshift codebook
 */
//static int cbcnt=0;
void MeanShiftCodebook::InsertView(unsigned idxObject,unsigned idxView, std::vector<cv::Ptr<CModel> > &objs)
{
  N=objs.size();
  PKeypoint::nbcnt++;
  float sqrThrDesc = PMath::Sqr(param.thrDesc);
  float invSqrSigmaDesc = 1./(PMath::Sqr(param.sigmaDesc));
  View &view = *objs[idxObject]->views[idxView];

  // match global with optimized matcher
  if (!descriptors.empty())
  {
    vector<cv::DMatch> matches;
    matcher->match(view.descriptors, descriptors, matches);

    // test and add to codebook
    for (unsigned i=0; i<matches.size(); i++)
    {
      const cv::DMatch &ma = matches[i];
      if (PMath::Sqr(ma.distance) <= cbEntries[ma.trainIdx]->sqrThr)
      {
        cbEntries[ma.trainIdx]->InsertMeanShift(objs, idxObject, idxView, ma.queryIdx, invSqrSigmaDesc);
        view.keys[ma.queryIdx]->nb = PKeypoint::nbcnt;
        cbEntries[ma.trainIdx]->copyTo(&descriptors(ma.trainIdx,0));
      } 
    }
  }

  // check new and add
  unsigned idx;
  float *d, dist, minDist;
  unsigned start = cbEntries.size();
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    if (view.keys[i]->nb != PKeypoint::nbcnt)
    {
      d = &view.descriptors(i,0);
      minDist = FLT_MAX, idx = UINT_MAX;
      for (unsigned j=start; j<cbEntries.size(); j++)
      {
        dist = CBEntry::DistSqr(&cbEntries[j]->data[0], d, view.descriptors.cols);
        if (dist < cbEntries[j]->sqrThr && dist < minDist)
        {
          minDist = dist;
          idx = j;
        }
      }
      if (idx!=UINT_MAX)    // found matching entry -> add
      {
        cbEntries[idx]->InsertMeanShift(objs, idxObject, idxView, i, invSqrSigmaDesc);
        cbEntries[idx]->copyTo(&descriptors(idx,0));
      }
      else
      {
        cbEntries.push_back(new CBEntry(N, d, view.descriptors.cols, idxObject, idxView, i, sqrThrDesc));
        if (descriptors.empty())
          descriptors = cv::Mat_<float>(1,view.descriptors.cols);
        else descriptors.push_back(cv::Mat_<float>(1,view.descriptors.cols));
        
        cbEntries.back()->copyTo(&descriptors(descriptors.rows-1,0));
      }
    }
  }

  optimized = false;
}

/**
 * Query an object view
 * @param matches <object_idx, matches>
 */
void MeanShiftCodebook::QueryObjects(cv::Mat_<float> &queryDescriptors, map<unsigned, vector<cv::DMatch> > &matches)
{
  matches.clear();
  if (descriptors.empty())
    return;

  // match
  vector<vector<cv::DMatch> > tmpMatches;

  if (param.useFlann && optimized)
    flannMatcher->knnMatch(queryDescriptors, tmpMatches, 2);
  else
    matcher->knnMatch(queryDescriptors, descriptors, tmpMatches, 2);

  // get object matches
  for (unsigned i=0; i<tmpMatches.size();i++)
  {
    if (tmpMatches[i].size()==1 || 
        (tmpMatches[i].size()==2 && tmpMatches[i][0].distance/tmpMatches[i][1].distance<param.nnRatio) )
    {
      CBEntry &entry = *cbEntries[tmpMatches[i][0].trainIdx];
      for (unsigned j=0; j<entry.occurrences.size(); j++)
      {
        Occurrence &occ = entry.occurrences[j];
        matches[occ.object].push_back(cv::DMatch(i, occ.key, occ.view, entry.wi));
      }
    }
  }

}





}







