//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYELLMETHODSUBPIXEL_HPP
#define  _SYELLMETHODSUBPIXEL_HPP

// include RTE libraries
#include "multiplatform.hpp"
#include "syArray.hpp"
#include "syEllipse.hpp"

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
//       RTE Ellipse Subpixel Detection
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEllMethodSubpixel
{
private:
   
   inline short GetPixelI16(IplImage *img, int x, int y);
   
public:

   // constructor
   CzEllMethodSubpixel();
   // destructor
   virtual ~CzEllMethodSubpixel();

   bool Subpixel(IplImage *dx, 
                 IplImage *dy, 
                 CzArray<CzEdgel> &points, 
                 CzEllipse *ell);

};
//end class/////////////////////////////////////////////////////////////////////


inline short CzEllMethodSubpixel::GetPixelI16(IplImage *img, int x, int y)
{
  return ((short*)(img->imageData + img->widthStep*y))[x];
}

NAMESPACE_CLASS_END()

#endif
