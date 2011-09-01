//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef SYFORMSEGMENTGROUPS_HPP
#define SYFORMSEGMENTGROUPS_HPP

#include "syArray.hpp"
#include "syEdgel.hpp"
#include "sySegment.hpp"
#include "syVoteImage.hpp"

NAMESPACE_CLASS_BEGIN( RTE )

//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE form closed segment groups
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzFormClosedSegmentGroups
{
private:

  CzVoteImage *voteImg;

public:
   CzFormClosedSegmentGroups();
   ~CzFormClosedSegmentGroups(); 
   
   // Release vote image
   void Reset(IplImage *img);
   
   // Find lines
   void Operate(CzArray<CzSegment*> &segments, int iLinkingGrowCount);
   // Draw the vote image onto the image img (for debugging purposes)
	void DrawVoteImg(IplImage *img, bool use_colour);
	// Close a contour
	void CloseContour(CzSegment * seg, CzArray<int> &idsUsed, CzSegment * joined);
};
//end class/////////////////////////////////////////////////////////////////////

 
NAMESPACE_CLASS_END()

#endif

