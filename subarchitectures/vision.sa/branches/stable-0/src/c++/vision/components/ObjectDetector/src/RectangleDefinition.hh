/**
 * @file RectangleDefinition.hh
 * @author Andreas Richtsfeld
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of Rectangle Definition
 **/

#ifndef Z_RectangleDefinition_HH
#define Z_RectangleDefinition_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the definition of a rectangle for tracking.
 */
class RectDef
{
public:
	
//	unsigned id; 							///< ID of the rectangle

//  unsigned clos;    				///< the underlying closure
  unsigned jcts[4]; 				///< L-junctions, in counter-clockwise order
	Vector2 jctsIsct[4];			///< Intersection points of the junctions
//  double parallelity;		 	///< parallelity of the two opposed edges
	Vector2 direction[2];			///< TODO mean direction of the two line-pairs (1,3 and 2,4)
//	double phi[2];						///< mean angle of the two line-pairs
//  unsigned nrOfLJcts;				///< number of L-Jcts from the underlying closure
	Vector2 centerPoint;			///< center point of the rectangle (2D mean value of corners)
//	double radius; 						///< maximum distance from center-point to a corner-point

//	unsigned width;						///< image width
//	unsigned height;					///< image height
//	unsigned pixelmass;				///<	number of pixels from lines of closure
//	unsigned *data;						///< data from CalculateSupport() (Bresenham alg.)
//	double pixelsupport; 			///< pixelsupport of the rectangle (in %) (>100 possible?)

};
}
#endif
