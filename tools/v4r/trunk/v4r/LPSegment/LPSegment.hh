/**
 * $Id$
 * Johann Prankl, 20091118
 */

#ifndef P_LPSEGMENT2_HH
#define P_LPSEGMENT2_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include "v4r/CEdge/PNamespace.hh"
#include "LPDefinitions.hh"
#include "v4r/PMath/PMath.hh"
#include "v4r/CEdge/SDraw.hh"
#include "v4r/CEdge/Vector2.hh"
#include "LPNode.hh"
#include "v4r/CEdge/CEdge.hh"
#include "LogPolar.hh"
#include "RGBHistogram.hh"
#include "RGBColourProb.hh"
#include "UVHistogram.hh"
#include "UVColourProb.hh"
#include "v4r/CEdge/ImgUtils.hh"
#include "v4r/CEdge/Segment.hh"
#include "v4r/CEdge/Line.hh"
#include "v4r/CEdge/FormLines.hh"

namespace P
{

class CfgLPSegment
{
public:
  bool useColourEdges;
  bool useColourPrior;
  bool useWeightPrior;
  bool useMask;
  double magnitudeScale;        //...to create a LOG polar image
  double weightColour;
  double weightPrior;
  double weightEdge;
  double sigmaMask;
};

/**
 * Graph based segmentation using log-polar images
 */
class LPSegment
{
private:
  CfgLPSegment cfg;
  IplImage *dbg;
  IplImage *edge, *dx,*dy, *cannyImg;
  LPNode *nodes, s, t;
  int width,height;   //graph size = width*height
  float *expTab;
  unsigned expSize;

  RGBColourProb *prob;
  RGBHistogram *fg, *bg;
  UVColourProb *uvprob;
  UVHistogram *uvfg, *uvbg;

  CEdge computeEdge;
  FormLines formLines;
  P::Array<Segment *> segments;
  P::Array<Line *> lines;

  void CreateGraph(IplImage *lp);
  void FindPath(LPNode *nodes, LPNode *s, LPNode *t);
  double GetLPContour(LPNode *t, LPNode *s, P::Array<P::Vector2> &contour);
  void LPContour2Cart(P::Array<P::Vector2> &contour, Vector2 center, Vector2 size, double M);
  double ProcessGraph(IplImage *costLP, P::Array<P::Vector2> &contour);
  void RemoveSTEdges(LPNode *nodes, int width, int height, LPNode *s, LPNode *t);
  void SetSTEdges(LPNode *nodes, int width, int height, LPNode *s, LPNode *t, int r);
  void ComputeRGBColourPrior(IplImage *img, IplImage *mask, IplImage *prior);
  void ComputeUVColourPrior(IplImage *img, IplImage *mask, IplImage *prior);
  void Integral(IplImage *img, IplImage *sum);
  void AddColourPrior(IplImage *img, Array<Vector2> &contour, IplImage *edgeWeighted);
  void AddPrior(IplImage *edgeWeighted, IplImage *prior, double weight);
  void MulGaussMask(Array<Vector2> &contour, IplImage *edgeWeighted);
  void SetExpTab(float *tab, unsigned size, float sigma);
  void SpanRectangle(Array<Vector2> &vs, Vector2 &ul, Vector2 &lr);
  void CenterOfGravity(P::Array<Vector2> &contour, Vector2 &center);
  void DoSegmentation(IplImage *img, Vector2 center, double dist, double sigma, P::Array<Vector2> &segContour, IplImage *weight);
  void MulGaussMask(Vector2 &center, double dist, double sigma, IplImage *edgeWeighted);



  inline float CostFkt(float e, float norm);
  inline double GetWeight(IplImage *prior, int u, int v, int dx, int dy, double norm);

public:
  LPSegment();
  ~LPSegment();
  void ComputeEdges(IplImage *img);
  void DoSegmentation(IplImage *img, P::Array<Vector2> &contour, P::Array<Vector2> &segContour, IplImage *weight=0);

  void Operate(IplImage *img, P::Array<Vector2> &contour, P::Array<Vector2> &segContour, IplImage *weight=0);
  void Operate(cv::Mat &image, vector<cv::Point2d> &contour, vector<cv::Point2d> &segContour, cv::Mat weight=cv::Mat());
  void Operate(cv::Mat &image, vector<cv::Point2d> &segContour, cv::Point2d &center, double dist=50., double sigma=25., cv::Mat weight=cv::Mat());
  void SetSigmaMask(double sigma){cfg.sigmaMask=sigma;}
  void Draw(cv::Mat &image, vector<cv::Point2d> &contour);


  void SaveImage(const char *file, IplImage *img, float scale=FLT_MAX);
  void Graph2Img(LPNode *n, int width, int height, IplImage *img);
  void SaveGraphVisited(const char *file, LPNode *graph, int width, int height);
  void SetDebugImage(IplImage *img){dbg=img;}
  
};

/*********************** INLINE METHODES **************************/
inline float LPSegment::CostFkt(float e, float norm)
{
  float cost = e/norm;

  if (cost < LP_MASK_OFFSET)
  {
    cost = 1./cost * LP_MASK_PENALIZE;
  }
  else if (cost < LP_COST_OFFSET)
  {
    cost = 1./cost * LP_COST_PENALIZE;
  }
  else
  {
    cost = 1./cost;
  }

  if (LP_USE_SQR_COST)
    return Sqr(cost);

  return cost;
}


inline double LPSegment::GetWeight(IplImage *prior, int u, int v, int dx, int dy, double norm)
{
  Vector2 d = Vector2(dx,dy);

  if (dx==0 && dy==0)
    return 0.;

  d.Normalise();
  d *= LP_SAMPLE_DISTANCE;
  
  int w1 = GetPx8UC1(prior, u+d.x, v+d.y);
  int w2 = GetPx8UC1(prior, u-d.x, v-d.y);

  return (w1>w2?(w1-w2)/norm:(w2-w1)/norm);
}




}

#endif

