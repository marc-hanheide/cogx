 /**
 * $Id$
 *
 * @author  Michael Zillich,
 *      <A HREF="http://www.cs.bham.ac.uk">The University Of Birmingham</A>
 * @date August 2006
 */

#ifndef ELL_MARKERS_HH
#define ELL_MARKERS_HH

#include <vector>
#include <algorithm>
#include <opencv/cv.h>

/**
 * The calibration model.
 * A rectangular grid of nx * ny points, where the "upper right" point at
 * (nx, ny) is missing. This allows a unique pose determination.
 */
class Model
{
public:
  int nx, ny;  // numbers of points in x- and y-direction
  double dx, dy; // spacing of points in x- and y-direction
  int NumPoints() {return nx*ny - 1;}  // one corner is "missing"
};

class Ellipse
{
public:
  double x, y, a, b, phi;  // a, b are half-axes
  Ellipse() {}
  Ellipse(double _x, double _y, double _a, double _b, double _phi)
  : x(_x), y(_y), a(_a), b(_b), phi(_phi) {}
};

class EllipsePair
{
public:
  Ellipse ell1, ell2;
  CvPoint2D64f center;
  double size;
  EllipsePair() {}
  EllipsePair(Ellipse e1, Ellipse e2) : ell1(e1), ell2(e2)
  {
    center.x = (ell1.x + ell2.x)/2.;
    center.y = (ell1.y + ell2.y)/2.;
    size = std::max(ell1.a, ell2.a);
  }
};

extern bool DetectEllipseMarkers(char *rgba24, int width, int height,
  Model &model, std::vector<CvPoint2D64f> &centers,
  std::vector<Ellipse> &out_ells, std::vector<EllipsePair> &out_pairs,
  std::vector<CvPoint2D64f> &all_edgels);

#endif
