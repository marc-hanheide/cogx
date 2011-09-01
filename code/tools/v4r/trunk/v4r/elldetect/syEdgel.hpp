//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef  _SYEDGEL_HPP
#define  _SYEDGEL_HPP

#include "syVector2.hpp"

NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE simple edgel object
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzEdgel
{
public:
   // edgel point position
   CzVector2 p;
   // edgel directory
   double dir;

   CzEdgel() {}
   CzEdgel(const CzVector2 &p_in, double d_in) {p = p_in; dir = d_in;}
   CzEdgel(const CzVector2 &p_in) : dir(0.) {p = p_in;}
};
//end class/////////////////////////////////////////////////////////////////////

NAMESPACE_CLASS_END()

#endif

