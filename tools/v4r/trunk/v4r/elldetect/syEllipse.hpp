//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYELLIPSE_HPP
#define  _SYELLIPSE_HPP

#include "multiplatform.hpp"

#include "syArray.hpp"
#include "syVector2.hpp"
#include "syEdgel.hpp"

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
//       RTE ellipse object
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEllipse
{
private:

   double a2, b2, a4, b4;  // a^2 etc., stored for Distance()
   double x2,y2;

   inline CzVector2 TransformToEllipse(const CzVector2 &p);
   inline CzVector2 TransformFromEllipse(const CzVector2 &p);

public:
   // ellipse position
   double dX, dY;    
   // ellipse long/short axis length
   double dA, dB;    
   // ellipse rotation in radiants
   double dPhi;      
   // mean fit error
   double dFitError; 
   // ellipse support
   double dSupport;  
   // ellipse marker id
   int iMarkerID;
   // boolean member for parallel calculation of ellipses
   bool bEllipseOK;

   CzEllipse();
   CzEllipse(double x_in, double y_in, double a_in, double b_in, double phi_in);
   void Set(double x_in, double y_in, double a_in, double b_in, double phi_in);


   inline double EllipseCircumference(double a, double b);
   inline bool InsideEllipse(double a, double b, double x0, double y0, double phi, double x, double y);	

   CzVector2 Tangent(const CzVector2 &p);
   CzVector2 TangentCentAxPar(const CzVector2 &p);
   unsigned EllipseSupport(CzArray<CzEdgel> &points, double inlDist, CzArray<bool> &inlIdx);    
   static bool FitEllipse(CzArray<CzEdgel> &ps, double &x, double &y, double &a, double &b, double &phi);
   bool computeAndSetGeomFromConic(double A, double B, double C, double D, double E, double F);
};
//end class/////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////
// Transform a point from image to ellipse co-ordinates.
inline CzVector2 CzEllipse::TransformToEllipse(const CzVector2 &p)
{
   return Rotate(p - CzVector2(dX, dY), -dPhi);
}

////////////////////////////////////////////////////////////////////////////////
// Transform a point from ellipse to image co-ordinates.
inline CzVector2 CzEllipse::TransformFromEllipse(const CzVector2 &p)
{
   return Rotate(p, dPhi) + CzVector2(dX, dY);
}

////////////////////////////////////////////////////////////////////////////////
// Approximation to ellipse circumference.
// (from Bartsch: Mathematische Formeln, Buch- und Zeit-Verlagsgesellschaft Koeln, 1988, p. 221)
inline double CzEllipse::EllipseCircumference(double a, double b)
{
   return M_PI*(1.5*(a + b) - sqrt(a*b));
}


////////////////////////////////////////////////////////////////////////////////
// Returns true if the position x/y is inside the given ellipse using the ellipse equation
bool inline CzEllipse::InsideEllipse(double a, double b, double x0, double y0, double phi, double x, double y)
{
   double dx = ((x - x0)*cos(phi) + (y-y0)*sin(phi)) / a;
   double dy = (-(x - x0)*sin(phi) + (y-y0)*cos(phi)) / b;
   double distance = dx * dx + dy * dy;
   return (distance < 1.0) ? 1 : 0;

}


NAMESPACE_CLASS_END()

#endif

