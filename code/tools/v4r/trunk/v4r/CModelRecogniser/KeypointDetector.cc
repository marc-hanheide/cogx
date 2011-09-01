/**
 * $Id$
 */


#include "KeypointDetector.hh"
#include "v4r/PMath/PVector.hh"

namespace P 
{



/***
 * Draw keypoints
 */
void KeypointDetector::Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &keys, cv::Scalar col, unsigned detail)
{
  for (unsigned i=0; i<keys.size(); i++)
  {
    PKeypoint::Draw(img, *keys[i], col);
  }
}

/**
 * Draw matched keypoints
 */
void KeypointDetector::Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<cv::Ptr<PKeypoint> > &queryKeys, const vector<cv::DMatch> &matches, cv::Scalar col, unsigned detail)
{
  if (detail==2)
    Draw(img,queryKeys,col);

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (detail==0)
    {
      cv::circle(img,queryKeys[matches[i].queryIdx]->pt, 2,col);
    }
    else if (detail==1)
    {
      PKeypoint::Draw(img, *queryKeys[matches[i].queryIdx], col);
    }
    cv::line(img, trainKeys[matches[i].trainIdx]->pt, queryKeys[matches[i].queryIdx]->pt, CV_RGB(255,255,255));
  }
}

/**
 * Draw matched keypoints
 */
void KeypointDetector::Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<cv::Ptr<PKeypoint> > &queryKeys, const vector<vector<cv::DMatch> > &matches, cv::Scalar col, unsigned detail)
{
  if (detail==2)
    Draw(img,queryKeys,col);

  for (unsigned i=0; i<matches.size(); i++)
  {
    if (detail==0 && matches[i].size()>0)
    {
      cv::circle(img,queryKeys[matches[i][0].queryIdx]->pt, 2,col);
    }
    else if (detail==1 && matches[i].size()>0)
    {
      PKeypoint::Draw(img, *queryKeys[matches[i][0].queryIdx], col);
    }
    cv::line(img, trainKeys[matches[i][0].trainIdx]->pt, queryKeys[matches[i][0].queryIdx]->pt, CV_RGB(255,255,255));
  }
}


}

