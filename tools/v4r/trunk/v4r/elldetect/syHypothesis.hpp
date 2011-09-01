//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYHYPOTHESIS_HPP
#define  _SYHYPOTHESIS_HPP

#include "multiplatform.hpp"

#include "syEllipse.hpp"
#include "syArray.hpp"
#include "syVector2.hpp"
#include "syEdgel.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H
#include OCV_CXCORE_H


NAMESPACE_CLASS_BEGIN( RTE )

//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE ellipse hypothesis object
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzHypothesis : public CzEllipse
{
private:


public:

   CzHypothesis(){};
   CzHypothesis(double x_in, double y_in, double a_in, double b_in, double phi_in);

   CzArray<CzEdgel> m_pEdgels;

   // Get edgels of ellipse hypothesis
   const int GetEdgelsCount();
   const CzEdgel* GetEdgel( int iIndex );

};
//end class/////////////////////////////////////////////////////////////////////


NAMESPACE_CLASS_END()

#endif

