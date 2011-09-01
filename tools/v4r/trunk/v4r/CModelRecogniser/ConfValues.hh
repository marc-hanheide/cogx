/**
 * $Id$
 */

#ifndef P_CONF_VALUES_HH
#define P_CONF_VALUES_HH

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "MatchPairs.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMatrix.hh"


namespace P
{

class ConfValues
{
private:
  const cv::Mat &intrinsic;
  const cv::Mat &distCoeffs;

  double inlDist, sqrInlDist;

public:
  double constNormValue;

  ConfValues(const cv::Mat &cam, const cv::Mat &disto, double _inlDist=1.);
  ~ConfValues();

  double SupportingPointsPerView(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose);
  double WeightedSupportingPointsPerView(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose);
  double MatchedPointsPerView(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, Pose &pose);
  double WeightedMatchedPointsPerView(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, Pose &pose);
  double WeightedMatchedPointsPerView(MatchPairs &mp, unsigned sizeView, Pose &pose);


  double SupportingPointsPerXX(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose);
  double WeightedSupportingPointsPerXX(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, Pose &pose);
  double MatchedPointsPerXX(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, Pose &pose);
  double WeightedMatchedPointsPerXX(vector<cv::Ptr<PKeypoint> > &queryKeys, 
                    vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, Pose &pose);
  double WeightedMatchedPointsPerXX(MatchPairs &mp, Pose &pose);


  double ConfToProb(double conf);
  double PredictProbFromAngle(double angle);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif

