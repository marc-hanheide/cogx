/**
 * $Id$
 */

#ifndef P_MATCH_FILTER_HH
#define P_MATCH_FILTER_HH

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "MeanShiftFAMS.hh"


namespace P
{

class MatchFilter
{
private:
  vector<cv::DMatch> tmpMatches;

public:
  MatchFilter();
  ~MatchFilter();

  void Voting(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
         vector<cv::DMatch> &matches, cv::Point2d &center, vector<vector<cv::DMatch> > &filtMatches);
  void ThrProjDist(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
         vector<cv::DMatch> &matches, Pose &pose, cv::Mat &cam, vector<cv::DMatch> &filtMatches, double thr=2.);
  void ThrDescriptor(const vector<cv::DMatch> &matches, vector<cv::DMatch> &filtMatches, double thr=.4);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

