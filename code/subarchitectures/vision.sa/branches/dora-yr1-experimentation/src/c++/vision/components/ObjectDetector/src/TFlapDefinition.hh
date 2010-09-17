/**
 * @file TFlapDefinition.hh
 * @author Andreas Richtsfeld
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of Flap Definition
 **/

#ifndef Z_TFlapDefinition_HH
#define Z_TFlapDefinition_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the definition of a flap for tracking.
 */
class TFlapDef
{
public:
	bool hypothesised;				///< Hypothesised Flap?
//	unsigned id;							///< id of the flap

  unsigned rects[2];				///< Rectangles of flap
//   double meanGap;						// mean value of smallest two gaps between 
														// two rectangle-corners
//   Array<unsigned> sharedLines;	// sharedLines from the two rectangles
  unsigned innerJcts[4];		///< the inner 4 L-Junctions of the flap
	Vector2 innerJctsIsct[4]; ///< the intersection points of the 4 inner L-Junctions
  unsigned outerJcts[4];		///< the outer 4 L-Junctions of the flap
	Vector2 outerJctsIsct[4]; ///< the intersection points of the 4 outer L-Junctions
														// [0,1] from rect[0] clockwise
														// [2,3] from rect[1] counter clockwise

	Vector2 center;						// center point of the flap (mean value of innerJcts->iscts)
// 	double radius;						// maximum radius from center to outerJcts->iscts

	Vector2 rectCenter[2];						///< Center points of the rectangles
	double rectRadius[2];								///< Maximum radius from rectCenter to junction intersections

	/** 6 possibilities for oCase (Anordnung der beiden Rechtecke lt. Flap-Def):
				Right/Left	...	oCase = 1
				Left/Right	...	oCase = 2
				Front/Top		...	oCase = 3
				Top/Front		... oCase = 4
				Left/Top		...	oCase = 5
				Top/Right		...	oCase = 6
	**/
// 	unssigned oCase;

};
}
#endif
