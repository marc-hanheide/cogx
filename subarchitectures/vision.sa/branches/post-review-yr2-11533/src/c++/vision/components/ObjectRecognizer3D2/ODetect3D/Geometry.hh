/**
 * $Id: Geometry.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 *
 * Johann Prankl, 30.11.2006 
 */


#ifndef P_GEOMETRY_HH
#define P_GEOMETRY_HH

#include "PNamespace.hh"
#include "Except.hh"
#include "Array.hh"
#include "Vector2.hh"
#include "matrix.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


namespace P 
{

double IsLeft( Vector2 p0, Vector2 p1, Vector2 p2 );
void ChainHull2D( Array<Vector2> &p, Array<Vector2> &h);
void ConvexHull( Array<Vector2> &p, Array<Vector2> &h);
int IWrap ( int ival, int ilo, int ihi );
int IMax ( int i1, int i2 );
int IMin ( int i1, int i2 );
int IModp ( int i, int j );
void AngleHalf(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3, Vector2 &p4);
double AngleRAD(const Vector2 &p1, const Vector2 &p2, const Vector2 &p3);
}

#include "Geometry.ic"

#endif

