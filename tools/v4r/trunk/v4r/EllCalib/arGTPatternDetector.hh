/**
 * $Id$
 */

#ifndef P_AR_GT_PATTERN_DETECTOR_HH
#define P_AR_GT_PATTERN_DETECTOR_HH

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/elldetect/EDWrapper.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/armarker/armarker.h"


namespace P 
{

class EPNode;
class EllPattern;

/**
 * arGTPatternDetector
 */
class arGTPatternDetector
{
public:
  class Parameter
  {
  public:
    double xDist, yDist;          // distance of the ellipses in x an y direction
    int xOffs, yOffs;             // offset of the coordinate system (indices)
    int xMin, xMax, yMin, yMax;   // ellipse indices in x and y direction
    double dist;                  // threshold to accept ellipse location (affine model)
    bool filterHom;
    double distH;
    
    Parameter(double xd=30, double yd=30, int _xOffs=4, int _yOffs=0, 
       int _xMin=-2, int _xMax=3, int _yMin=-3, int _yMax=5, double _dist=10., bool filt=true,
       double dH=1.) 
     : xDist(xd), yDist(yd), xOffs(_xOffs), yOffs(_yOffs), 
       xMin(_xMin), xMax(_xMax), yMin(_yMin), yMax(_yMax), dist(_dist), filterHom(filt), distH(dH) {} 
  };

private:
  cv::Mat grayImage;
  vector<vector<EPNode> > graph;

  cv::Ptr<RTE::EDWrapper> elldetector;

  vector<RTE::CzEllipse> whiteElls;
  vector<RTE::CzEllipse> blackElls;
  vector<pair<unsigned, unsigned> > rings;

  int ids[2];
  V4R::MarkerDetection ardetector;

  bool haveModel;
  double sqrDist;

  void DetectCoordSys(const cv::Mat &img, vector<cv::Ptr<EllPattern> > &patterns);
  void Insert(EllPattern &pattern, int xIdx, int yIdx, RTE::CzEllipse &ell, Parameter &param);
  void AddSupportingEllipses(EllPattern &pat, vector<RTE::CzEllipse> &ells, Parameter &param);
  void InitGraph(EllPattern &pat, Parameter &param);
  void Mark(int x, int y, Parameter &param);
  void Look4EllsAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, Parameter &param);
  bool InsertEllipseAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, float H[9], Parameter &param);
  bool IsLine2(float p1[2], float p2[2], float p3[2]);
  bool GetModel(int x, int y, float H[9], Parameter &param);
  void FilterHom(const EllPattern &src, const Parameter &param, EllPattern &dst);

  inline EPNode& GetNode(int x, int y, Parameter &param);


public:

  string pat[2];
  Parameter param[2];
  cv::Mat dbg;

  arGTPatternDetector(const string &_pat1, const string &_pat2, Parameter p1=Parameter(), Parameter p2=Parameter());
  ~arGTPatternDetector();

  bool Detect(const cv::Mat &img, EllPattern &pattern);
  void Draw(cv::Mat &img, EllPattern &pattern);
};


/**************************** SMALL HELPER CLASSES *************************
 * The pattern to detect
 */
class EllPattern
{
public:
  double quotientRing;     // the origint is a ring => r_inner/r_outer
  int ids[2];              // marker code of x1 and y1 if available

  vector<cv::Point3f> objPoints;
  vector<cv::Point2f> imgPoints;
  vector<pair<int,int> > location;

  EllPattern() : quotientRing(1) {ids[0]=-1, ids[1]=-1;}
};

/**
 * ELPNode
 */
class EPNode
{
public:
  unsigned nb;
  cv::Point2f pt;
  cv::Point3f pos;
  EPNode() : nb(0) {}
};


/********************************* INLINE METHODES *********************************/

/**
 * Get a node from the graph
 */
inline EPNode& arGTPatternDetector::GetNode(int x, int y, Parameter &param)
{
  //return graph[y+abs(param.yMin)][x+abs(param.xMin)];
  return graph[y-param.yMin][x-param.xMin];
}


} //--END--

#endif

