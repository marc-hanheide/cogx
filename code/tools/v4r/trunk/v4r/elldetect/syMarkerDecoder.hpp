//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYMARKERDECODER_HPP
#define  _SYMARKERDECODER_HPP

// include RTE libraries
#include "multiplatform.hpp"
#include "syLoggerClass.hpp"
#include "syEllipse.hpp"
#include "syMark.hpp"
#include "syString.hpp"

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
//       RTE Marker decoding
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzMarkerDecoder: public CzLoggerClass
{
private:

   // Class to decode marker images
   CzMark *m_Mark;
   // Destination ellipse radius in pixels to decode
   double m_dDestinationRadius;
   // Quotient outermost to innermost radius of mark
   double m_dQuotientOuterInner;
   // Outer ring width in pixels
   int m_iRingWidth;
   // minimal contrast needed (range: 0 ... 1.0) compared to the contrast of the inner circle to the surrounding area
   double m_dMinContrast;
   
   // debugging options
   // directory to store debugging images
   CzN::CzString m_sDebugDirectory;
   // create debugging images
   bool m_bDebugImages;
   
   // Transforms the given ellipse area of the originial image 
   // to a square (ellipse -> circle)
   // pImage: original image
   // pElllipse: ellipse on the image
   // returns: pointer to the created image
   IplImage *TransformEllipse(const IplImage *pImage, 
                              CzEllipse *pEllipse);
public:

   // constructor
   CzMarkerDecoder();
   // destructor
   virtual ~CzMarkerDecoder();

   // Initialize with decoding options
   // bEllipseWhite: TRUE: white ellipse, FALSE: black ellipse
   // iBitCode: bitcode width, has to be 12, 14 or 20
   // dQuotientOuterInner: quotient outermost to innermost radius of marker
   // iRingWidth: outer ring width in pixels
   void Init(bool bEllipseWhite,
             int iBitCode,
             double dQuotientOuterInner,
             double dDestInnerRadius,
             int iRingWidth,
             double dMinContrast,
             bool bWriteDebugImages,
             CzN::CzString sDebugDirectory);
   
   // Decode the mark around the given ellipse from the original image
   // pImage: original image
   // pEllipse: ellipse on the image to decode
   // iIdx: index is used for writing the debug image
   int DecodeEllipse(const IplImage *pImage, 
                     CzEllipse *pEllipse,
                     int iIdx);

   ////////////////////////////////////////////////////////////////////////////////
   // write debug image with deskewed ellipse
   void WriteDebugImage(const IplImage *pImage, 
                        int iIdx,
                        int iMarkID);
};
//end class/////////////////////////////////////////////////////////////////////


NAMESPACE_CLASS_END()

#endif
