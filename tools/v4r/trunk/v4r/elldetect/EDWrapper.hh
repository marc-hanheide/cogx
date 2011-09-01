/**
 * $Id$
 */

#ifndef P_ED_WRAPPER_HH
#define P_ED_WRAPPER_HH

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SMath.h"
#include "syArray.hpp"
#include "syEllipse.hpp"
#include "syEdge.hpp"
#include "syEllipseDetection.hpp"
#include "syEllMethodSubpixel.hpp"
#include "syEllMethodDualEllipse.hpp"


namespace RTE 
{

class EDWrapper
{
public:
  class Parameter
  {
  public:
    bool whiteElls;          // find white ellipses
    bool blackElls;          // find black ellipses
    bool decodeWhite;        // decode the code of the marker around white ellipses
    bool decodeBlack;        // decode the code of the marker around white ellipses
    bool detRings;            // detect black ring in front of white background
    bool ellSubPx;           // subpixel refinement using dual method instead of least squares fit 

    double thrRing;          // threshold of inner/outer radius to accept ring [px]    (1)
    int iMinEllipseDiameter; // minimum/maximum ellipse diameter in pixel              (5)
    int iMaxEllipseDiameter;
    double dRadiusInOut;     // spacing from the ellipse border in pixels              (3)
                             //used to check min. contrast and max. contrast deviation
    int iMinContrast;        // minimum contrast to the background; good value is 20.0 (10)
    double dMaxContrastDev;  // maximum standard deviation of ellipse points           (35)
    double dMaxQuotient;     // maximum quotient long to short ellipse axis
    double dMinSupport;      // minimum pixel support                                  (0.6)
    double dMaxHypFiterror;  // maximum mean fiterror for hypotheses detection         (1.)
    double dMaxFiterror;     // maximum mean fiterror for exakt ellipse calculation    (1.)
    int iSobelApertureSize;  // aperture size of sobel operator
    int iCannyLow;           // canny edge detector, low/high border
    int iCannyHigh;
    bool bGaussSmooth;       // do a gaussian smoothing on the input image             (true)
    int iLinkingGrowCount;   // max. px for a segment to grow for edge-segment-linking (automatic <0)
    double dDualEllipseMinGradient;  // min. gradient for a pixel to be used for dual ellipse calculation   (30)
    int iDualEllipseBorder;  // num. of px used around the border of the ell. hyp. for dual refinement (3)

    int eMarkerCode;         // 0..12bit, 1..14bit, 2..20bit
    double dMarkerQuotientOuterInner; // quotient outermost to innermost radius of marker
    double dMarkerDestInnerRadius;    // inner circle radius in pixels after deskewing (good value is 12.0)
    int iMarkerRingWidth;         // outer ring width in pixels                  (10)
    double dMarkerMinContrast;    // minimal contrast needed (range: 0 ... 1.0)  (.5)

    Parameter(bool wh=true, bool bl=true, bool deWh=false, bool deBl=false, bool ring=true, bool _ellSubPx=true)
      : whiteElls(wh), blackElls(bl), decodeWhite(deWh), decodeBlack(deBl), detRings(ring), ellSubPx(_ellSubPx),
        thrRing(1.), iMinEllipseDiameter(2), iMaxEllipseDiameter(200), dRadiusInOut(3.), 
        iMinContrast(10), dMaxContrastDev(35.), dMaxQuotient(4.), dMinSupport(0.6), dMaxHypFiterror(1.),
        dMaxFiterror(1.), iSobelApertureSize(3), iCannyLow(70), iCannyHigh(140), bGaussSmooth(true), 
        iLinkingGrowCount(10), dDualEllipseMinGradient(30), iDualEllipseBorder(3), eMarkerCode(0), 
        dMarkerQuotientOuterInner(3.), dMarkerDestInnerRadius(12), iMarkerRingWidth(10), 
        dMarkerMinContrast(.5) {};
  };

private:

  Parameter param;

  cv::Mat grayImage;

  RTE::CzEllipseDetection elldetect;

  void Clear();
  void InitEllDetect();
  void DetectWhiteElls(IplImage *img, vector<RTE::CzEllipse> &ells);
  void DetectBlackElls(IplImage *img, vector<RTE::CzEllipse> &ells);
  void DetectRings(vector<RTE::CzEllipse> &white, vector<RTE::CzEllipse> &black, vector<pair<unsigned, unsigned> > &rings);

  template<typename T1,typename T2>
  inline T1 DistanceSqr2(const T1 d1[2], const T2 d2[2]);

  template <class Num>
  inline Num Sqr(Num x);



public:

  cv::Mat dbg;

  EDWrapper(Parameter p=Parameter());
  ~EDWrapper();

  void Detect(const cv::Mat &img, vector<RTE::CzEllipse> &whiteElls, vector<RTE::CzEllipse> &blackElls,
              vector<pair<unsigned, unsigned> > &rings);

  void SetParameter(Parameter &p);

  void DrawEllipses(cv::Mat &img, vector<RTE::CzEllipse> &ells, cv::Scalar color, bool printcode=false);
  void DrawEll(cv::Mat &img, RTE::CzEllipse &ell, cv::Scalar color);
  void DrawRing(cv::Mat &img, RTE::CzEllipse &iell, RTE::CzEllipse &oell, cv::Scalar color);
  void DrawRings(cv::Mat &img, vector<RTE::CzEllipse> &whiteElls, vector<RTE::CzEllipse> &blackElls, 
                 vector<pair<unsigned, unsigned> > &rings, cv::Scalar color);
};


template<typename T1,typename T2>
inline T1 EDWrapper::DistanceSqr2(const T1 d1[2], const T2 d2[2])
{
  return Sqr(d1[0]-d2[0]) + Sqr(d1[1]-d2[1]);
}

template <class Num>
inline Num EDWrapper::Sqr(Num x)
{
  return x*x;
}



} //--END--

#endif

