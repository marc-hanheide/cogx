/**
 * $Id: Arc.hh,v 1.18 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_ARC_HH
#define Z_ARC_HH

#include "Vector2.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"

namespace Z
{

/**
 * A circular arc.
 */
class Arc : public Gestalt
{
private:
  bool OrderAnglesCounterClockwise(double &a, double &b, double &c);
  void CalculateParams();
  void CalculateAngles();
  void CalculateColors();
  void CalculateColorHistograms();
  void CalculateTangents();
  void CalculateSignificance();
  bool HasInsideStrong(Arc *b);
  bool HasInsideWeak(Arc *b);

public:
  static int STRICT_CONVEXITY;

  unsigned seg;      ///< the originating segment
  unsigned prev_on_seg;  ///< previous arc on the same segment
  unsigned next_on_seg;  ///< next arc on the same segment
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
  RGBColor mean_col_inside;
  RGBColor mean_col_outside;

  Arc(unsigned seg_id, unsigned i, unsigned j, unsigned k, const Vector2 &cent,
      double rad);
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  void DrawTangents();
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

inline Array<Gestalt*>& Arcs()
{
  return Gestalts(Gestalt::ARC);
}
inline Arc* Arcs(unsigned id)
{
  return (Arc*)Gestalts(Gestalt::ARC, id);
}
inline unsigned NumArcs()
{
  return NumGestalts(Gestalt::ARC);
}

}

#endif

