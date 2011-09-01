/**
 * $Id$
 */


#include "MatchFilter.hh"




namespace P 
{

static const bool CmpIdxSize(const pair<unsigned,unsigned>& a, const pair<unsigned,unsigned>& b)
{
  return (a.second > b.second);
}


/************************************************************************************
 * Constructor/Destructor
 */

MatchFilter::MatchFilter()
{ 
}

MatchFilter::~MatchFilter()
{
}

/**
 * use object center voting to cluster keypoints
 */
void MatchFilter::Voting(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
         vector<cv::DMatch> &matches, cv::Point2d &center, vector<vector<cv::DMatch> > &filtMatches)
{
  double angle, scale;
  vector<cv::Point2d> votes;
  vector<vector<unsigned> > cls;
  vector<pair<unsigned,unsigned> > idxSize;

  MeanShiftFAMS mshift;

  votes.resize(matches.size());
  for (unsigned i=0; i<matches.size(); i++)
  {
    PKeypoint::Vote(*trainKeys[matches[i].trainIdx],*queryKeys[matches[i].queryIdx], center, 
                    votes[i], angle, scale);
  }

  cv::Mat_<double> samples(votes.size(),2,&votes[0].x);
  mshift.Cluster(samples,cls);

  idxSize.resize(cls.size());
  for (unsigned i=0; i<cls.size(); i++)
    idxSize[i] = pair<unsigned,unsigned>(i,cls[i].size());

  sort(idxSize.begin(), idxSize.end(), CmpIdxSize); 

  filtMatches.clear();
  filtMatches.resize(cls.size());
  for (unsigned i=0; i<idxSize.size(); i++)
  {
    for (unsigned j=0; j<cls[idxSize[i].first].size(); j++)
      filtMatches[i].push_back(matches[ cls[idxSize[i].first][j] ]);
  }
}


/**
 * Threshold the matches depending 3d backprojection
 */
void MatchFilter::ThrProjDist(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
         vector<cv::DMatch> &matches, Pose &pose, cv::Mat &cam, vector<cv::DMatch> &filtMatches, double thr)
{
  double dist, sqrInlDist = PMath::Sqr(thr);
  double pt[2], pos[3];

  filtMatches.clear();

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (trainKeys[matches[i].trainIdx]->Have3D())
    {
      PMat::MulAdd3(pose.R.ptr<double>(), &trainKeys[matches[i].trainIdx]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, cam.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&queryKeys[matches[i].queryIdx]->pt.x);
      if (dist < sqrInlDist)
      {
        filtMatches.push_back(matches[i]);
      }
    }
  }
}

/**
 * Filter depeding on absolut descriptor distance
 */
void MatchFilter::ThrDescriptor(const vector<cv::DMatch> &matches, vector<cv::DMatch> &filtMatches, double thr)
{
  tmpMatches.clear();

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (matches[i].distance < thr)
      tmpMatches.push_back(matches[i]);
  }

  filtMatches=tmpMatches;
}

}












