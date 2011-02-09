/**
 * $Id$
 * Michael Zillich
 */

#ifndef P_ARC_HH
#define P_ARC_HH

#include "Vector2.hh"
#include "PNamespace.hh"
#include "Segment.hh"

namespace P
{

/**
 * A circular arc.
 */
class Arc
{
private:
  bool OrderAnglesCounterClockwise(double &a, double &b, double &c);
  void CalculateParams();
  void CalculateAngles();
  void CalculateTangents();
  void CalculateSignificance();
  bool HasInsideStrong(Arc *b);
  bool HasInsideWeak(Arc *b);

public:
  static int STRICT_CONVEXITY;

  double sig;
  double support;

  Segment *seg;      ///< the originating segment
  Arc *prev_on_seg;  ///< previous arc on the same segment
  Arc *next_on_seg;  ///< next arc on the same segment
  unsigned idx[5];   ///< index of start, end and mid point, according to pixel
                     ///< order on the segment. Note that this does not not have
                     ///< to be counter clockwise. In that case idx[START]
                     ///< corresponds to point[END] and vice versa!
  Vector2 point[5];  ///< first, last and mid point (in counter clockwise order)
  Vector2 norm[3];      ///< normals pointing inwards
  Vector2 center;       ///< circle center
  double radius;        ///< circle radius
  double start_angle;   ///< starting angle (in counter clockwise order)
  double end_angle;     ///< end angle (in counter clockwise order)
  double angular_span;  ///< difference between start and end angle
  Vector2 tang_pt[3];   ///< points of tangents
  Vector2 tang[3];      ///< tangents START, END, MID

  Arc(Segment *seg, unsigned i, unsigned j, unsigned k, const Vector2 &cent,
      double rad);
  void Draw(IplImage *img, int detail = 0);
  bool IsAtPosition(int x, int y);
  unsigned NumEdgels() {return idx[END] - idx[START] + 1;}
  double Radius() {return radius;}
  double ArcLength() {return angular_span*radius;}
  bool HasInside(Arc *b);
  bool HasInside(Vector2 &p);
  bool ConvexWith(Arc *b);
  double Insideness(Arc *b);
  double Convexity(Arc *b);
  bool HasInsideWedge(Arc *b);
  bool HasInsideBeam(Arc *b);
  bool CoaxialOverlap(Arc *b);
};



void DeleteArcs(Array<Arc*> &arcs);

inline void DeleteArc(Array<Arc*> &arcs, unsigned idx);


/***************************** INLINE METHODES ********************************/
inline void DeleteArc(Array<Arc*> &arcs, unsigned idx)
{
  delete arcs[idx];
  arcs.Erase(idx);
}



}

#endif

