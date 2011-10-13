/**
 * $Id$
 * TODO:
 * - Compute prob values
 */


#include "ConfValues.hh"




namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */

ConfValues::ConfValues(const cv::Mat &cam, const cv::Mat &disto, double _inlDist)
  : intrinsic(cam), distCoeffs(disto), inlDist(_inlDist), sqrInlDist(inlDist*inlDist),
    constNormValue(20)
{ 
}

ConfValues::~ConfValues()
{
}

/**
 * SupportingPointsPerView
 * = num_supporting_points / num_points_of_recognised_view  (no matches used)
 */
double ConfValues::SupportingPointsPerView(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned inl=0, cnt=0;
  double pt[2], pos[3];
  double minDist, dist;

  for (unsigned i=0; i<trainKeys.size(); i++)
  {
    if (trainKeys[i]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      minDist = DBL_MAX;
      for (unsigned j=0; j<queryKeys.size(); j++)
      {
        dist = PVec::DistanceSqr2(pt,&queryKeys[j]->pt.x);
        if (dist  < minDist)
          minDist = dist;
      }

      if (minDist < sqrInlDist)
       inl++;
      cnt++;
    }
  }

  if (cnt>0)
    return ((double)inl)/(double)cnt;
  return 0.;
}

/**
 * WeightedSupportingPointsPerView
 * = num_supporting_points / num_points_of_recognised_view (no matches used)
 *   each supporting point is weighted with the inlier distance
 */
double ConfValues::WeightedSupportingPointsPerView(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned cnt=0;
  double sig=0.;
  double pt[2], pos[3];
  double minDist, dist;

  for (unsigned i=0; i<trainKeys.size(); i++)
  {
    if (trainKeys[i]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      minDist = DBL_MAX;
      for (unsigned j=0; j<queryKeys.size(); j++)
      {
        dist = PVec::DistanceSqr2(pt,&queryKeys[j]->pt.x);
        if (dist  < minDist)
          minDist = dist;
      }

      if (minDist < sqrInlDist)
        sig += inlDist-sqrt(minDist);
      cnt++;
    }
  }

  if (cnt>0)
    return (sig/inlDist) / (double)cnt;
  return 0.;
}

/**
 * MatchedPointsPerView
 * = num_inl_matched_points / num_points_of_recognised_view
 */
double ConfValues::MatchedPointsPerView(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, std::vector<cv::DMatch> &matches, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned inl=0, cnt=0;
  double pt[2], pos[3];
  double dist;

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[ma.trainIdx]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&queryKeys[ma.queryIdx]->pt.x);

      if (dist < sqrInlDist)
       inl++;

      cnt++;
    }
  }

  if (cnt>0)
    return ((double)inl)/(double)cnt;
  return 0.;
}

/**
 * WeightedMatchedPointsPerView
 * = num_inl_matched_points / num_points_of_recognised_view
 *   each supporting point is weighted with the inlier distance
 */
double ConfValues::WeightedMatchedPointsPerView(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, std::vector<cv::DMatch> &matches, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned cnt=0;
  double sig=0.;
  double pt[2], pos[3];
  double dist;

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[ma.trainIdx]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&queryKeys[ma.queryIdx]->pt.x);

      if (dist < sqrInlDist)
        sig += inlDist-sqrt(dist);

      cnt++;
    }
  }

  if (cnt>0)
    return (sig/inlDist) / (double)cnt;
  return 0.;
}

/**
 * WeightedMatchedPointsPerView
 * = num_inl_matched_points / num_points_of_recognised_view
 *   each supporting point is weighted with the inlier distance
 */
double ConfValues::WeightedMatchedPointsPerView(MatchPairs &mp, unsigned sizeView, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned cnt=0;
  double sig=0.;
  double pt[2], pos[3];
  double dist;

  for (unsigned j=0; j<mp.ptsImage.size(); j++)
  {
    for (unsigned i=0; i<mp.ptsImage[j].size(); i++)
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &mp.ptsModel[j][i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&mp.ptsImage[j][i]->pt.x);

      if (dist < sqrInlDist)
        sig += inlDist-sqrt(dist);

      cnt++;
    }
  }

  if (cnt>0 && inlDist>0 && sizeView > 0)
    return (sig/inlDist) / (double)sizeView;
  return 0.;
}



/**
 * SupportingPointsPerXX
 */
double ConfValues::SupportingPointsPerXX(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned inl=0;
  double pt[2], pos[3];
  double minDist, dist;

  for (unsigned i=0; i<trainKeys.size(); i++)
  {
    if (trainKeys[i]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      minDist = DBL_MAX;
      for (unsigned j=0; j<queryKeys.size(); j++)
      {
        dist = PVec::DistanceSqr2(pt,&queryKeys[j]->pt.x);
        if (dist  < minDist)
          minDist = dist;
      }

      if (minDist < sqrInlDist)
       inl++;
    }
  }

  double conf = ((double)inl)/constNormValue;
  return (conf > 1.? 1. : conf);
}

/**
 * WeightedSupportingPointsPerXX
 */
double ConfValues::WeightedSupportingPointsPerXX(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose)
{
  if (pose.empty())
    return 0.;

  double sig=0.;
  double pt[2], pos[3];
  double minDist, dist;

  for (unsigned i=0; i<trainKeys.size(); i++)
  {
    if (trainKeys[i]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[i]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      minDist = DBL_MAX;
      for (unsigned j=0; j<queryKeys.size(); j++)
      {
        dist = PVec::DistanceSqr2(pt,&queryKeys[j]->pt.x);
        if (dist  < minDist)
          minDist = dist;
      }

      if (minDist < sqrInlDist)
        sig += inlDist-sqrt(minDist);
    }
  }

  double conf = (sig/inlDist) / constNormValue;
  return (conf > 1.? 1. : conf);
}

/**
 * MatchedPointsPerXX
 */
double ConfValues::MatchedPointsPerXX(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, std::vector<cv::DMatch> &matches, Pose &pose)
{
  if (pose.empty())
    return 0.;

  unsigned inl=0;
  double pt[2], pos[3];
  double dist;

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[ma.trainIdx]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&queryKeys[ma.queryIdx]->pt.x);

      if (dist < sqrInlDist)
       inl++;
    }
  }

  double conf = ((double)inl)/constNormValue;
  return (conf > 1.? 1. : conf);
}

/**
 * WeightedMatchedPointsPerXX
 */
double ConfValues::WeightedMatchedPointsPerXX(std::vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    std::vector<cv::Ptr<PKeypoint> > &trainKeys, std::vector<cv::DMatch> &matches, Pose &pose)
{
  if (pose.empty())
    return 0.;

  double sig=0.;
  double pt[2], pos[3];
  double dist;

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (trainKeys[ma.trainIdx]->Have3D())
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &trainKeys[ma.trainIdx]->pos.x, pose.t.ptr<double>(), pos);
      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&queryKeys[ma.queryIdx]->pt.x);

      if (dist < sqrInlDist)
        sig += inlDist-sqrt(dist);
    }
  }

  double conf = (sig/inlDist) / constNormValue;
  return (conf > 1.? 1. : conf);
}

/**
 * WeightedMatchedPointsPerXX
 */
double ConfValues::WeightedMatchedPointsPerXX(MatchPairs &mp, Pose &pose)
{
  if (pose.empty())
    return 0.;

  double sig=0.;
  double pt[2], pos[3];
  double dist;

  for (unsigned j=0; j<mp.ptsImage.size(); j++)
  {
    for (unsigned i=0; i<mp.ptsImage[j].size(); i++)
    {
      PMat::MulAdd3( pose.R.ptr<double>(), &mp.ptsModel[j][i]->pos.x, pose.t.ptr<double>(), pos);

      ProjectPoint2Image(pos, intrinsic.ptr<double>(), pt);

      dist = PVec::DistanceSqr2(pt,&mp.ptsImage[j][i]->pt.x);

      if (dist < sqrInlDist)
        sig += inlDist-sqrt(dist);
    }
  }

  double conf = (sig/inlDist) / constNormValue;
  return (conf > 1.? 1. : conf);
}

/**
 * Transform the confidence value to a learned probability
 */
double ConfValues::ConfToProb(const double conf)
{
  return (conf>1. ? 1. : conf);
}

/**
 * Prdict the probability of a TP recognition result with a changed view point
 * @param angle angle between learnded view point and view point to predict [rad]
 */
double ConfValues::PredictProbFromAngle(const double angle)
{
  return 1-angle;
}


}  // -- THE END --












