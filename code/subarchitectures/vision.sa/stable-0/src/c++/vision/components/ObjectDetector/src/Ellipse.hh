/**
 * $Id: Ellipse.hh,v 1.14 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_ELLIPSE_HH
#define Z_ELLIPSE_HH

#include "Edgel.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class Ellipse : public Gestalt
{
private:
  class ContourPoint
  {
  public:
    int x, y;
    double err;
    unsigned id;
    double dir;
    bool supports;
    ContourPoint() {};
    ContourPoint(int xi, int yi, double erri)
      : x(xi), y(yi), err(erri), id(UNDEF_ID), dir(0.), supports(false) {}
  };
  Array<ContourPoint> contour;
  double a2, b2, a4, b4;  // a^2 etc., stored for Distance()

  void CalculateSupport();
  void CalculateSignificance();
  Vector2 TransformToEllipse(const Vector2 &p);
  Vector2 TransformFromEllipse(const Vector2 &p);
  double DistanceCentAxPar(const Vector2 &p);
  bool GradientMatches(const Edgel &e);
  bool GradientMatches(const Vector2 &p, double dir);
  bool GradientMatchesCentAxPar(const Vector2 &p, double dir);
  Vector2 Tangent(const Vector2 &p);
  Vector2 TangentCentAxPar(const Vector2 &p);
  Vector2 Normal(const Vector2 &p);
  Vector2 NormalCentAxPar(const Vector2 &p);
  void OptimalNeighbour(int x, int y, int a, int b, int &x_o, int &y_o,
      double &d_o);
  void BuildContourString();
  void FillContourString();

public:
  unsigned group;							///< group the ellipse was constucted from
  double x, y, a, b, phi;			///< ellipse parameters
  double support;							///< relative support
  double abs_support;					///< absulute support
  double fit_error;						///< mean fit error
  double area;								///< area of the ellipse
  Vector2 vertex[2];					///< vertex [LEFT/RIGHT]					ARI: sinnvoll? ist ja auch in x,y,a,b,phi
  Vector2 dir;								///< direction between vertices, normalised to 1
  Array<unsigned> e_jct[2];		///< Ellipse-Junction START/END
  unsigned extEllipse;				///< ExtEllipses
  Array<unsigned> cones;			///< cones
  Array<unsigned> cylinders;	///< cylinders

  Ellipse(const Ellipse &e);
  Ellipse(unsigned grp_in, double x_in, double y_in, double a_in, double b_in, double phi_in);
  void AddEJunction(unsigned eJct);
  void AddExtEllipse(unsigned extEll);
  void AddCone(unsigned c);
  void AddCylinder(unsigned cyl);
  void DrawVotes();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  double Circumference();
  double Area();
  double Distance(const Vector2 &p);
  double Distance(const Edgel &e);
};

inline Array<Gestalt*>& Ellipses()
{
  return Gestalts(Gestalt::ELLIPSE);
}
inline Ellipse* Ellipses(unsigned id)
{
  return (Ellipse*)Gestalts(Gestalt::ELLIPSE, id);
}
inline unsigned NumEllipses()
{
  return NumGestalts(Gestalt::ELLIPSE);
}

extern bool FitEllipse(Array<unsigned> &arcs, unsigned l, unsigned u,
    double &x, double &y, double &a, double &b, double &phi);
extern bool FitEllipse(Array<unsigned> &arcs,
    double &x, double &y, double &a, double &b, double &phi);
extern double EllipseCircumference(double a, double b);
extern double EllipseArea(double a, double b);
extern double EllipseSupport(Array<unsigned> &arcs, unsigned l, unsigned u,
    double x, double y, double a, double b, double phi, double &fit_error,
    double *abs_sup = 0);
extern double EllipseSupport(Array<unsigned> &arcs,
    double x, double y, double a, double b, double phi, double &fit_error,
    double *abs_Sup);
extern double EllipseCurvature(double a, double b, double x, double y);

}

#endif
