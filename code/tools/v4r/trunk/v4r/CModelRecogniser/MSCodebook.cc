/**
 * $Id$
 * Johann Prankl, 2011-03-31
 * prankl@acin.tuwien.ac.at
 * TODO:
 * - speed up: e.g. do mean shift only in optimize and use flann for matching
 *                  or use gpu for matching
 */


#include "MSCodebook.hh"

#define DEBUG


namespace P 
{

static const bool CmpMatchesDec(const cv::Ptr<OVMatches>& a, const cv::Ptr<OVMatches>& b)
{
  return (a->conf > b->conf);
}



MSCodebook::MSCodebook(cv::Ptr<cv::DescriptorMatcher> &descMatcher, Parameter _param)
 : optimized(false), N(0), haveNorm(false), param(_param)
{
  matcher = descMatcher;
}

MSCodebook::~MSCodebook()
{
}




/************************************** PRIVATE ************************************/

/**
 * set ni=0
 */
void MSCodebook::Reset_ni()
{
  for (unsigned i=0; i<cbEntries.size(); i++)
  {
    cbEntries[i]->ni=0;
  }
}

/**
 * compute norm for n
 */
float MSCodebook::Norm2_n()
{
  float norm_n = 0.;

  for (unsigned i=0; i<cbEntries.size(); i++)
    norm_n += PMath::Sqr(cbEntries[i]->ni*cbEntries[i]->wi);

  return sqrt(norm_n);
}

/**
 * Normalize the weights
 */
void MSCodebook::PrecomputeNorm2_m()
{
  norm_m.resize(views.size());
  diff.resize(views.size());
  for (unsigned i=0; i<norm_m.size(); i++) norm_m[i]=0.;

  map<unsigned, vector<unsigned> >::iterator it;
  for (unsigned i=0; i<cbEntries.size(); i++)
  {
    for (it = cbEntries[i]->idxViewKeys.begin(); it != cbEntries[i]->idxViewKeys.end(); it++)
      norm_m[it->first] += PMath::Sqr(it->second.size()*cbEntries[i]->wi); // mi*wi
  }

  for (unsigned i=0; i<norm_m.size(); i++)
    if (!PMath::IsZero(norm_m[i])) norm_m[i] = sqrt(norm_m[i]);
  haveNorm=true;
}





/************************************** PUBLIC ************************************/
/**
 * Clear the tree and delete all view links
 */
void MSCodebook::clear()
{ 
  views.clear();
  cbEntries.clear();
  descriptors = cv::Mat();
}

/**
 * Optimize
 * for now just copy descriptors to a dense matrix
 */
void MSCodebook::Optimize()
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

  PrecomputeNorm2_m();
}

/**
 * Insert an object view to the tree structure
 * attention:
 * - we get inconsistency because of (mean shift) 
 * - and because of global match and then add new checks only new...
 */
//static int cbcnt=0;
void MSCodebook::InsertView(unsigned oidx, unsigned vidx, cv::Ptr<View> &view)
{
  view->id = vidx;        // that should not be here, but just to be sure...
  view->idx = oidx;
  views.push_back(view);
  N++;
  PKeypoint::nbcnt++;
  float sqrThrDesc = PMath::Sqr(param.thrDesc);
  float invSqrSigmaDesc = 1./(PMath::Sqr(param.sigmaDesc));

  /*cbcnt+=view->keys.size();
  cout<<"cbEntries.size()="<<cbEntries.size()<<" + "<<view->keys.size()
      <<" = "<<(cbEntries.size()+view->keys.size());
  //cv::Mat img(100,100,CV_8U);
  //cv::imshow("Image",img);*/

  // match global with optimized matcher
  if (!descriptors.empty())
  {
    vector<cv::DMatch> matches;
    matcher->match(view->descriptors, descriptors, matches);

    // test and add to codebook
    for (unsigned i=0; i<matches.size(); i++)
    {
      const cv::DMatch &ma = matches[i];
      if (PMath::Sqr(ma.distance) <= cbEntries[ma.trainIdx]->sqrThr)
      {
        cbEntries[ma.trainIdx]->InsertMeanShift(views, views.size()-1, ma.queryIdx, invSqrSigmaDesc);
        view->keys[ma.queryIdx]->nb = PKeypoint::nbcnt;
        cbEntries[ma.trainIdx]->copyTo(&descriptors(ma.trainIdx,0));
      } 
    }
  }

  // check new and add
  unsigned idx;
  float *d, dist, minDist;
  unsigned start = cbEntries.size();
  for (unsigned i=0; i<view->keys.size(); i++)
  {
    if (view->keys[i]->nb != PKeypoint::nbcnt)
    {
      d = &view->descriptors(i,0);
      minDist = FLT_MAX, idx = UINT_MAX;
      for (unsigned j=start; j<cbEntries.size(); j++)
      {
        dist = CBEntry::DistSqr(&cbEntries[j]->data[0], d, view->descriptors.cols);
        if (dist < cbEntries[j]->sqrThr && dist < minDist)
        {
          minDist = dist;
          idx = j;
        }
      }
      if (idx!=UINT_MAX)    // found matching entry -> add
      {
        cbEntries[idx]->InsertMeanShift(views, views.size()-1, i, invSqrSigmaDesc);
        cbEntries[idx]->copyTo(&descriptors(idx,0));
      }
      else
      {
        cbEntries.push_back(new CBEntry(N, d, view->descriptors.cols, views.size()-1, i, sqrThrDesc));
        if (descriptors.empty())
          descriptors = cv::Mat_<float>(1,view->descriptors.cols);
        else descriptors.push_back(cv::Mat_<float>(1,view->descriptors.cols));
        
        cbEntries.back()->copyTo(&descriptors(descriptors.rows-1,0));
      }
    }
  }
  //cout<<" -> "<<cbEntries.size()<< " / "<<cbcnt<<endl;
  //cv::waitKey(0);
  //Optimize();

  haveNorm=false;       // to PrecomputeNorm2_m of object views
  optimized = false;
}

/**
 * Query an object view
 * @param matches <object_idx, matches> --> sorted better first
 */
void MSCodebook::QueryObjects(cv::Mat_<float> &queryDescriptors, vector<cv::Ptr<OVMatches> > &matches)
{
  matches.clear();
  if (descriptors.empty())
    return;

  Reset_ni();
  if (!haveNorm) PrecomputeNorm2_m();

  // match
  vector<cv::DMatch> cbMatches;

  if (param.useFlann && optimized)
  {
    vector<vector<cv::DMatch> > tmpMatches;

    flannMatcher->knnMatch(queryDescriptors, tmpMatches, 1);

    for (unsigned i=0; i<tmpMatches.size();i++)
      for (unsigned j=0; j<tmpMatches[i].size(); j++)
        cbMatches.push_back(tmpMatches[i][j]);
  }
  else
  {
    matcher->match(queryDescriptors, descriptors, cbMatches);
  }

  for (unsigned i=0; i<cbMatches.size(); i++)
  {
    const cv::DMatch &ma = cbMatches[i];
    if (PMath::Sqr(ma.distance) <= cbEntries[ma.trainIdx]->sqrThr)
    {
      cbEntries[ma.trainIdx]->ni++;
    }
  }

  // sum up weights
  float norm_n = Norm2_n();
  map<unsigned, vector<unsigned> >::iterator it;

  if (!PMath::IsZero(norm_n))
  {
    for (unsigned i=0; i<diff.size(); i++) 
      diff[i]=pair<unsigned,float>(i,0.);

    for (unsigned i=0; i<cbEntries.size(); i++)
    {
      CBEntry &cbe = *cbEntries[i];
      float nqi = cbe.ni*cbe.wi/norm_n;

      for (it = cbe.idxViewKeys.begin(); it!=cbe.idxViewKeys.end(); it++)
        diff[it->first].second += nqi*(it->second.size()*cbe.wi / norm_m[it->first]);
    }

    for (unsigned i=0; i<diff.size(); i++) 
      diff[i].second = 2.- 2.*diff[i].second;

    // return matches and sort
    vector<cv::Ptr<OVMatches> > tmpMatches(views.size());
    for (unsigned i=0; i<views.size(); i++)
      tmpMatches[i] = new OVMatches(views[i]->idx, views[i]->id, 1.-(diff[i].second/2.));

    for (unsigned i=0; i<cbMatches.size(); i++)
    {
      const cv::DMatch &ma = cbMatches[i];
      if (cbEntries[ma.trainIdx]->ni > 0 && PMath::Sqr(ma.distance) <= cbEntries[ma.trainIdx]->sqrThr)
      {
        CBEntry &cbe = *cbEntries[ma.trainIdx];

        for (it = cbe.idxViewKeys.begin(); it!=cbe.idxViewKeys.end(); it++)
          for (unsigned j=0; j<it->second.size(); j++)
          {
            tmpMatches[it->first]->matches.push_back(cv::DMatch(ma.queryIdx, it->second[j], views[it->first]->id, ma.distance));
          } 
      }
    }

    //copy back result
    for (unsigned i=0; i<tmpMatches.size(); i++)
      if (tmpMatches[i]->matches.size() > 0 && tmpMatches[i]->conf>0.001)
      {
        matches.push_back(tmpMatches[i]);
      }

    sort(matches.begin(), matches.end(), CmpMatchesDec);    
  }
}





}







