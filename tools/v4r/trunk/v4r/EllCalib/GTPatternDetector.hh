/**
 * $Id$
 */

#ifndef P_GT_PATTERN_DETECTOR_HH
#define P_GT_PATTERN_DETECTOR_HH

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/elldetect/EDWrapper.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PGeometry/PHomography.hh"


namespace P 
{

class EPNode;
class EllPattern;

/**
 * GTPatternDetector
 */
class GTPatternDetector
{
public:
  class Parameter
  {
  public:
    int ids[2];                   // marker code of x1 and y1 if available
    double xDist, yDist;          // distance of the ellipses in x an y direction
    int xOffs, yOffs;             // offset of the coordinate system (indices)
    int xMin, xMax, yMin, yMax;   // ellipse indices in x and y direction
    double dist;                  // threshold to accept ellipse location (affine model)
    
    Parameter(int id1=71, int id2=72, double xd=30, double yd=30, int _xOffs=4, int _yOffs=0, 
       int _xMin=-2, int _xMax=3, int _yMin=-3, int _yMax=5, double _dist=10.) 
     : xDist(xd), yDist(yd), xOffs(_xOffs), yOffs(_yOffs), 
       xMin(_xMin), xMax(_xMax), yMin(_yMin), yMax(_yMax), dist(_dist) 
       { ids[0]=id1; ids[1]=id2; } 
  };

private:
  cv::Mat grayImage;
  vector<vector<EPNode> > graph;

  cv::Ptr<RTE::EDWrapper> elldetector;

  vector<RTE::CzEllipse> whiteElls;
  vector<RTE::CzEllipse> blackElls;
  vector<pair<unsigned, unsigned> > rings;

  double sqrDist;

  void DetectCoordSys(vector<RTE::CzEllipse> &whiteElls, vector<RTE::CzEllipse> &blackElls, 
         vector<pair<unsigned, unsigned> > &rings, vector<cv::Ptr<EllPattern> > &patterns);
  void Insert(EllPattern &pattern, int xIdx, int yIdx, RTE::CzEllipse &ell, Parameter &param);
  void AddSupportingEllipses(EllPattern &pat, vector<RTE::CzEllipse> &ells, Parameter &param);
  void InitGraph(EllPattern &pat, Parameter &param);
  void Mark(int x, int y, Parameter &param);
  void Look4EllsAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, Parameter &param);
  bool InsertEllipseAt(int x, int y, vector<RTE::CzEllipse> &ells, EllPattern &pat, float H[9], Parameter &param);
  bool IsLine2(float p1[2], float p2[2], float p3[2]);
  bool GetModel(int x, int y, float H[9], Parameter &param);

  inline EPNode& GetNode(int x, int y, Parameter &param);


public:

  Parameter param1, param2;
  cv::Mat dbg;

  GTPatternDetector(Parameter p1=Parameter(), Parameter p2=Parameter());
  ~GTPatternDetector();

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
  EllPattern(cv::Point2f &center, double quot) : quotientRing(quot) {
    ids[0]=-1, ids[1]=-1;
    imgPoints.push_back(center);
    objPoints.push_back(cv::Point3f(0.,0.,0.));
    location.push_back(make_pair(0,0));}
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
inline EPNode& GTPatternDetector::GetNode(int x, int y, Parameter &param)
{
  return graph[y+abs(param.yMin)][x+abs(param.xMin)];
}


} //--END--

#endif

