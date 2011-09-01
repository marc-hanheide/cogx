//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYELLIPSEDETECTION_HPP
#define  _SYELLIPSEDETECTION_HPP


// include RTE libraries
#include "multiplatform.hpp"
#include "syLoggerClass.hpp"
#include "TArray.hpp"
#include "syEllipse.hpp"
#include "syExcept.hpp"
#include "syEllipseHypothesis.hpp"
#include "syEllMethodDualEllipse.hpp"
#include "syMarkerDecoder.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"
#include "frIPTargetMeasurement.hpp"
#include "syString.hpp"

// include OpenCV libraries
#include OCV_CV_H
#include OCV_CXCORE_H



NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE Ellipse Detection
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class _API_IPTARGETMEASUREMENT CzEllipseDetection: public CzLoggerClass
{
public:

   enum eEllipseDetectionConfigMethod
   { 
      ELL_CONFIGMETHOD_HYPOTHESES   = 0, // no subpixel calculation (fast, most inexact)
      ELL_CONFIGMETHOD_DUAL_ELLIPSE = 1  // dual ellipse method (slow, most precise)
   };
   enum eEllipseDetectionMarkerCode
   {
      ELL_MARKERCODE_12BIT = 12,
      ELL_MARKERCODE_14BIT = 14,
      ELL_MARKERCODE_20BIT = 20
   };
   typedef struct sEllipseDetectionConfig
   {
      // detection method
      eEllipseDetectionConfigMethod eMethod;
      // minimum/maximum ellipse diameter in pixel
      int iMinEllipseDiameter;
      int iMaxEllipseDiameter;
      // spacing from the ellipse border in pixels used to check min. contrast and max. contrast deviation
      double dRadiusInOut;
      // minimum contrast to the background; good value is 20.0
      int iMinContrast;
      // maximum standard deviation of ellipse points
      double dMaxContrastDev;
      // maximum quotient long to short ellipse axis
      double dMaxQuotient;
      // minimum pixel support 
      double dMinSupport;
      // maximum mean fiterror for hypotheses detection
      double dMaxHypFiterror;
      // maximum mean fiterror for exakt ellipse calculation
      double dMaxFiterror;
      // aperture size of sobel operator
      int iSobelApertureSize;
      // canny edge detector, low/high border
      int iCannyLow;
      int iCannyHigh;
      // do a gaussian smoothing on the input image
      bool bGaussSmooth;
      // maximum amount of pixels for a segment to grow for edge-segment-linking
      // for automatic calculation (based on min./max. ellipse diameter set to < 0)
      int iLinkingGrowCount;
      // minimum gradient for a pixel to be used for dual ellipse calculation
      double dDualEllipseMinGradient;
      // number of pixels to use around the border of the ellipse hypothesis for dual ellipse calculation
      int iDualEllipseBorder;
      // find white or black ellipses
      bool bEllipseWhite;

      // options for decoding the markers
      // decode the code of the marker around the ellipse
      bool bDecodeMarker;
      // type of markers (12bit, 14bit, 20bit)
      eEllipseDetectionMarkerCode eMarkerCode;
      // quotient outermost to innermost radius of marker
      double dMarkerQuotientOuterInner;
      // inner circle radius in pixels after deskewing (good value is 12.0)
      double dMarkerDestInnerRadius;
      // outer ring width in pixels
      int iMarkerRingWidth;
      // minimal contrast needed (range: 0 ... 1.0) compared to the contrast of the inner circle to the surrounding area
      double dMarkerMinContrast;
   
      // debugging options
      // directory to store debugging images and log files
      CzN::CzString sDebugDirectory;
      // create debugging images
      bool bDebugImages;
      // create debugging images of marker detection
      bool bDebugImagesMarker;
      // create log files
      bool bLogFiles;

   } sEllipseDetectionConfig;

private:

   // unique logger instance   
   CzLogger *m_pLogger;
   // configuration for ellipse detection
   sEllipseDetectionConfig m_Config;
   // array of found ellipses
   CzTArray<CzEllipse*> m_pEllipses;
   // ellipse hypothesis detector
   CzEllipseHypothesis *m_pHypothesis;
   // exact ellipse detector - dual ellipse method
   CzEllMethodDualEllipse *m_pMethodDualEllipse;
   // class to decode marks
   CzMarkerDecoder *m_pMarkerDecoder;

   void SetLogging(bool bDoLogging);
   // writes logging and debugging files
   void WriteLogFiles();
   void WriteDebugImage(const IplImage *pImage);

   // finds exact ellipses on given OpenCV image and actual hypotheses
   void DoExactCalculation(const IplImage *pImage);

public:

   // constructor
   CzEllipseDetection();
   // destructor
   virtual ~CzEllipseDetection();

   // find ellipses on the given Steinbichler image
   int FindEllipses(const CzN::CzAOI *pImage);
   // find ellipses on the given OpenCV image
   int FindEllipses(const IplImage *pImage);
   // find exact ellipses on the image based on the given ellispse hypotheses
   int FindEllipses(const IplImage *pImage, CzArray<CzHypothesis*> &pHypothesis);

   // configuration for ellipse detection
   sEllipseDetectionConfig* GetConfig();
   
   // get detected ellipses count and ellipses
   CzTArray<CzEllipse*> GetEllipses() { return m_pEllipses; }
   const int GetEllipsesCount();
   CzEllipse* GetEllipse( int iIndex );
   // clear results array of ellipses
   void ClearEllipses();

   // clear all results
   void Clear();

   // interface test routine
   void __AddEllipse(double dX, double dY, double dA, double dB, double dPhi, double dFitError);

};
//end class/////////////////////////////////////////////////////////////////////


NAMESPACE_CLASS_END()

#endif
