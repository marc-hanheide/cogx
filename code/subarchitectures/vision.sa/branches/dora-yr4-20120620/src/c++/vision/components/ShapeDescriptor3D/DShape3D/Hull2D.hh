/**
 * $Id$
 *
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 * 
 */

#ifndef P_CONVEX_HULL_HH
#define P_CONVEX_HULL_HH

#include "Array.hh"
#include "Except.hh"
#include "opencv/cv.h"
#include "opencv/cxcore.h"
#include "Vector2I.hh"

namespace P
{

class Hull2D
{
private:
  Array<Vector2I> points;

public:
  Hull2D();
  ~Hull2D();

  void ConvexHull(int *hull, int &hnum);

  static void ConvexHull( int *points, int pnum, int *hull, int &hnum);

  inline void Insert(double p[2]);
  inline void Insert(int p[2]);
  inline void Clear();
  inline unsigned Size();
};


/************************** INLINE METHODES ******************************/
/**
 * Insert a 3d point to the data structure
 */
inline void Hull2D::Insert(double p[2])
{
  points.PushBack(Vector2I((int)(p[0]+.5),(int)(p[1]+.5)));
}

/**
 * Insert a 3d point to the data structure
 */
inline void Hull2D::Insert(int p[2])
{
  points.PushBack(Vector2I(p[0],p[1]));
}

/**
 * Clear the data structure
 */
inline void Hull2D::Clear()
{
  points.Clear();
}

/**
 * Get data size
 */
inline unsigned Hull2D::Size()
{
  return points.Size();
}



}

#endif

