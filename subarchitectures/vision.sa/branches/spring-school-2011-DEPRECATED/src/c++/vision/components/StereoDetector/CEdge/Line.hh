/**
 * $Id$
 * Michael Zillich!
 */

#ifndef P_LINE_HH
#define P_LINE_HH

#include <list>
#include <set>
#include "Vector2.hh"
#include "PNamespace.hh"
#include "Segment.hh"
#include "SDraw.hh"
#include "ImgUtils.hh"

namespace P
{

/**
 * A directed straight line.
 */
class Line
{

public:
  unsigned id;
  int nb;

  Segment *seg;        ///< the originating segment
  unsigned idx[2];     ///< index of start and end point

  double sig;

  Vector2 point[2];    ///< end points, START/END
  Vector2 dir;         ///< direction of the line, normalised to 1
  Vector2 tang[2];     ///< tangents STAR/END
  double phi;          ///< angular direction
  double len;

                      ///                                      T
  Vector2 ln;         /// normalized line parameter (a/c b/c 1)
  Vector2 p;          /// midpoint
  float *vec;         /// line descriptor vector
  unsigned size;      /// size of the descriptor
  double err;

  Line();
  Line(Segment *seg, unsigned i, unsigned j);
  ~Line();

  void Recalc();
  void CalculateParameters();
  void CalculateParameters2();
  double MinEndpointDistance(const Line *l);
  double DistanceToPoint(const Vector2 &q);
  void CalculateSignificance(IplImage *prior, double norm);
  
  static void SampleLine(IplImage *prior, int x1,int y1,int x2,int y2, unsigned &sum, unsigned &num);
  static void SampleLine16SC1(IplImage *grad, int x1,int y1,int x2,int y2, int &sum, int &num);


  bool IsAtPosition(int x, int y);
  void DrawArrow(IplImage *img);
  void Draw(IplImage *img, int detail=0, CvScalar col=CV_RGB(255,0,0), int size=1);

  inline double Length() {return len;}
  inline unsigned NumEdgels() {return idx[END] - idx[START] + 1;}
  inline void AllocVec(unsigned s);
  inline void CopyVec(Line *l);
  inline void DeleteVec();

};

void DeleteLines(Array<Line*> &lines);

inline void DeleteLine(Array<Line*> &lines, unsigned idx);
inline void LinePts2Para(double p1[2], double p2[2], double l[3]);


/***************************** INLINE METHODES ********************************/
inline void DeleteLine(Array<Line*> &lines, unsigned idx)
{
  delete lines[idx];
  lines.Erase(idx);
}

inline void Line::AllocVec(unsigned s)
{
  if (vec!=0) delete[] vec;
  size=s;
  vec = new float[size];
}

inline void Line::CopyVec(Line *l)
{
  if (l->vec!=0){
    AllocVec(l->size);
    for (unsigned i=0; i<size; i++){
      vec[i]=l->vec[i];
    }
  }
}

inline void Line::DeleteVec()
{
  if (vec!=0)
  {
    delete[] vec;
    vec=0;
    size=0;
  }
}

/**
 * Compute parameters of a line defined by two points
 */
inline void LinePts2Para(double p1[2], double p2[2], double l[3])
{
  l[0] = p1[1]-p2[1];
  l[1] = p2[0]-p1[0];
  l[2] = p1[0]*p2[1] - p1[1]*p2[0];
}


}

#endif



