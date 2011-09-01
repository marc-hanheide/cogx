//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYEllipseHypothesis_HPP
#define  _SYEllipseHypothesis_HPP

// include RTE libraries
#include "multiplatform.hpp"
#include "syLoggerClass.hpp"
#include "syArray.hpp"
#include "syHypothesis.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H
#include OCV_CXCORE_H
#include OCV_HIGHGUI_H


NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE Ellipse Hypothesis Detection
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEllipseHypothesis: public CzLoggerClass
{
private:
   
   // array of found ellipse hypothesis
   CzArray<CzHypothesis*> m_Hypotheses;
   
   IplImage *m_SobelDx;
   IplImage *m_SobelDy;
   IplImage *m_Canny;

   inline uchar GetPixel(const IplImage *img, int x, int y);
   inline short GetPixelI16(const IplImage *img, int x, int y);
   inline bool InRange(const IplImage *img, int x, int y, float ddx, float ddy);

   // Release sobel and canny result images
   void ClearSobelCanny();

   // Check contrast of given ellipse
   bool CheckContrast(const IplImage *pImage,
                      IplImage *pSobelDx,
                      IplImage *pSobelDy,
                      CzHypothesis *pEllipse,
                      CzArray<CzEdgel> &pEllPoints,
                      double dRadiusInOut,
                      int iMinContrast,
                      double dMaxContrastDev,
                      bool bEllipseWhite);

   // Link edges
   void LinkEdges(const IplImage *pImage,
                  IplImage *pSobelDx,
                  IplImage *pSobelDy,
                  IplImage *pCanny,
                  int iMinEllipseDiameter,
                  int iMaxEllipseDiameter,
                  double dMaxQuotient,
                  double dRadiusInOut,
                  int iMinContrast,
                  double dMaxContrastDev,
                  double dMinSupport,
                  double dMaxHypFiterror,
                  bool bEllipseWhite,
                  int iLinkingGrowCount);

public:

   // constructor
   CzEllipseHypothesis();
   // destructor
   virtual ~CzEllipseHypothesis();

   // Find ellipse hypotheses on given pImage
   int FindHypotheses(const IplImage *pImage,
                      int iMinEllipseDiameter,
                      int iMaxEllipseDiameter,
                      double dMaxQuotient,
                      double dRadiusInOut,
                      int iMinContrast,
                      double dMaxContrastDev,
                      double dMinSupport,
                      double dMaxHypFiterror,
                      int iSobelApertureSize,
                      int iCannyLow,
                      int iCannyHigh,
                      int iLinkingGrowCount,
                      bool bSmooth,
                      bool bEllipseWhite);

   // Creates sobel and canny images, if not yet created
   void GetSobelCanny(const IplImage *pImage, 
                      bool bSmooth,
                      int iSobelApertureSize,
                      int iCannyLow,
                      int iCannyHigh,
                      IplImage *&pOutSobelDx, 
                      IplImage *&pOutSobelDy, 
                      IplImage *&pOutCanny); 

   // Set ellipse hypotheses for further processing
   void SetHypotheses(CzArray<CzHypothesis*> &pHypotheses);

   // Get found ellipse hypotheses count and ellipse hypotheses
   const int GetHypothesesCount();
   CzHypothesis* GetHypothesis( int iIndex );
   // Clear results array of ellipse hypotheses
   void Clear();

};
//end class/////////////////////////////////////////////////////////////////////


inline uchar CzEllipseHypothesis::GetPixel(const IplImage *img, int x, int y)
{
  return ((uchar*)(img->imageData + img->widthStep*y))[x];
}

inline short CzEllipseHypothesis::GetPixelI16(const IplImage *img, int x, int y)
{
  return ((short*)(img->imageData + img->widthStep*y))[x];
}

inline bool CzEllipseHypothesis::InRange(const IplImage *img, int x, int y, float ddx, float ddy) {
		return ( ((x + ddx) < img->width && (x + ddx) > 0) &&
						 ((x - ddx) < img->width && (x - ddx) > 0) &&
						 ((y - ddy) < img->height && (y - ddy) > 0) &&
						 ((y + ddy) < img->height && (y + ddy) > 0));
}

NAMESPACE_CLASS_END()

#endif
