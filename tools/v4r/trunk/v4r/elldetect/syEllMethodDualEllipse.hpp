//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYELLMETHODDUALELLIPSE_HPP
#define  _SYELLMETHODDUALELLIPSE_HPP

// include RTE libraries
#include "multiplatform.hpp"
#include "syLoggerClass.hpp"
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
//       RTE DualEllipse Detection
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEllMethodDualEllipse: public CzLoggerClass
{
private:
   
   
public:

   // constructor
   CzEllMethodDualEllipse();
   // destructor
   virtual ~CzEllMethodDualEllipse();

   // Calculate exact ellipse parameters of the given ellipse using gradient images
   bool DualEllipse(double dDualEllipseMinGradient,
                    int iDualEllipseBorder,
                    IplImage *pSobelDx, 
                    IplImage *pSobelDy, 
                    CzArray<CzEdgel> &pEllPoints, 
                    CzEllipse *pEllipse);

//	double varianceE, varianceD, covarianceCenter;
};
//end class/////////////////////////////////////////////////////////////////////


NAMESPACE_CLASS_END()

#endif
