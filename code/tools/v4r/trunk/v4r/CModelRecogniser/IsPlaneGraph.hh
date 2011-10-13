/**
 * $Id$
 */

#ifndef P_IS_PLANE_GRAPH_HH
#define P_IS_PLANE_GRAPH_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMath.hh"
#include "v4r/PCore/toString.hpp"
#include "PKeypoint.hh"
#include "Plane.hh"
#include "DistIdx.hh"
#include "SelectMatchesMRF.hh"
#include "GraphFrame.hh"
#include "MergePlanesHF.hh"
#include "svm/SVMPredictor.h"



namespace P
{

using namespace std;

/**
 * IsPlaneGraph
 */
class IsPlaneGraph
{
public:
  class Parameter
  {
  public:
    int width, height;        // image size
    double inlDist;
    int kMatches;
    float nnRatio;
    double thrDesc;
    double sigmaDistF;             // sigma for merging of planes depending on the fundamental matrix
    double sigmaDistH;             // homography of borders

    Parameter(int w=640, int h=480, double _inlDist=1., int k=3, float nnr=.8, double _thrDesc=.5,
       double sigmaf=3., double sigmah=2.) 
     : width(w), height(h), inlDist(_inlDist), kMatches(k), nnRatio(nnr), thrDesc(_thrDesc),
       sigmaDistF(sigmaf), sigmaDistH(sigmah) {}
  };

private:
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<SelectMatchesMRF> selmat;
  cv::Ptr<MergePlanesHF> hf;

  cv::Ptr<SVMPredictor> svm;

  vector<vector<cv::DMatch> > matches;
  vector<int> selectedMatches;

  vector<cv::Ptr<GraphFrame> > graphPlanes;

  void LinkPlanes(int idxSrcFrame, int idxDstFrame);
  void MatchPlanes(Plane &trainPlane, Plane &queryPlane, int &numMatches);
  void SetNNR(vector<vector<cv::DMatch> > &matches, vector<int> &selectedMatches);
  void GetPoints(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
        vector<vector<cv::DMatch> > &matches, vector<int> &selectedMatches, 
        vector<cv::Point2f> &queryPts, vector<cv::Point2f> &trainPts);
  void PrintInfo(GraphFrame &frame);

  inline void GetConf(unsigned numMatches, unsigned numLastKeys, unsigned numCurrentKeys, double &conf);
  inline void GetPartOf(double conf, unsigned numMatches, unsigned numLastKeys, unsigned numCurrentKeys, double &lPartofC, double &cPartofL);




public:
  Parameter param;
  cv::Mat dbg;

  IsPlaneGraph(cv::Ptr<cv::DescriptorMatcher> &descMatcher,
              Parameter _param=Parameter());
  ~IsPlaneGraph();

  void Create(vector< cv::Ptr<Plane> > &planes, unsigned idFrame);
  void Draw(cv::Mat &img, unsigned detail=0);
  void DrawText(cv::Mat &img, unsigned idx);
  void SetPlaneColour(GraphFrame &frame);
  void GTSaveTracks(cv::Mat &img, unsigned idx);
};




/*********************** INLINE METHODES **************************/
/**
 * ComputeConf
 */
inline void IsPlaneGraph::GetConf(unsigned numMatches, unsigned numLastKeys, unsigned numCurrentKeys, double &conf)
{
  conf =  ((double)numMatches) / (double)(numLastKeys+numCurrentKeys-numMatches);
}

/**
 * GetNumbers
 */
void IsPlaneGraph::GetPartOf(double conf, unsigned numMatches, unsigned numLastKeys, unsigned numCurrentKeys, double &lPartofC, double &cPartofL)
{
  if (numMatches>0)
  {
    cPartofL = conf / (((double)numMatches) / (double)(numLastKeys));
    lPartofC = conf / (((double)numMatches) / (double)(numCurrentKeys));
  }
  else
  {
    cPartofL = lPartofC = 0.;
  }
}



}

#endif

