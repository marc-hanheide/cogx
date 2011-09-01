//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Michael Zillich,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef SYSEGMENT_HPP
#define SYSEGMENT_HPP

#include "multiplatform.hpp"

#include "syArray.hpp"
#include "syEdgel.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H

NAMESPACE_CLASS_BEGIN( RTE )


//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE canny edge segment
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzSegment
{
private:

public:
   // edgels of segment
   CzArray<CzEdgel> edgels;
   // start/end segment
	CzSegment *start;
	CzSegment *end;
	bool s, e;
	unsigned id;
	bool closable, closed;

   CzSegment();
   CzSegment(const CzArray<CzEdgel> &arr);

   // Returns true if any part of the Segment is at pixel position (x,y).
   bool IsAtPosition(int x, int y);
   // Returns the normalised tangent direction at the specified pixel.
   CzVector2 Tangent(int i, int l = 0, int u = UINT_MAX);
};
//end class/////////////////////////////////////////////////////////////////////

NAMESPACE_CLASS_END()

#endif

