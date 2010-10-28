/**
 * $Id: Cone.cc,v 1.0 2007/12/13 13:47:03 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "LJunction.hh"
#include "Cone.hh"


namespace Z
{

Cone::Cone(unsigned ell, unsigned l0, unsigned l1, unsigned lj) : Gestalt(CONE)
{
  extEllipse = Ellipses(ell)->extEllipse;
  ellipse = ell;
  ljct = lj;
  line[LEFT] = l0;
  line[RIGHT] = l1;
	
  // Cone = Ellipse + connection to intersection point of L-Junction
  point[0] = Ellipses(ell)->vertex[0];
  point[1] = LJunctions(ljct)->isct;
  point[2] = Ellipses(ell)->vertex[1];

	ellipseCenter = (point[2] + point[0])/2.;
	ellRadius = (ellipseCenter - point[0]).Length();
	topRadius = (ellipseCenter - point[1]).Length();

  Ellipses(ellipse)->AddCone(id);

	geoSig = 1.0;
	bool geometry = CheckGeometry(geoSig);
	if(!geometry)
		Mask(1000);
	
  CalculateSignificance();
}

/**
**	IsInside()
**	true, if the ellipse-center of cone is inside the topRadius of this cone
*/
bool Cone::IsInside(unsigned cone)
{
  if((Cones(cone)->ellipseCenter - ellipseCenter).Length() < topRadius) return true;
  else return false; 
}

/**
**	CheckGeometry()
**	
*/
bool Cone::CheckGeometry(double &diff)
{
	// return false, if L-Junction is inside the ellRadius
	if((ellipseCenter - point[1]).Length() < ellRadius) return false;
	
	// cone is a right circular cone? => calculate the angles of left and right surface line to the ellipse normal
	Vector2 dirEll = Normalise(point[2] - point[0]);
		dirEll = Rotate(dirEll, M_PI/2);
	Vector2 dirLeft = Normalise(point[1] - point[0]);
	Vector2 dirRight = Normalise(point[1] - point[2]);
	
	double oAngle0 = acos(Dot(dirEll, dirLeft));	// if opening angle of both lines goes to M_PI/2, then the difference will be smaller and smaller
	double oAngle1 = acos(Dot(dirEll, dirRight));

	if (oAngle0 > M_PI/2.) oAngle0 = M_PI - oAngle0;
	if (oAngle1 > M_PI/2.) oAngle1 = M_PI - oAngle1;

//	printf("Cone %u: oAngles: %4.2f - %4.2f\n", id, oAngle0, oAngle1);

	diff = fabs(acos(Dot(dirEll, dirLeft)) - acos(Dot(dirEll, dirRight)));
//	printf("	phis: %4.2f - %4.2f - %4.2f\n", PolarAngle(dirEll), PolarAngle(dirLeft), PolarAngle(dirRight));
//	printf("	Diff: %4.2f\n", diff);

/*	printf("Cone.cc - cone: %u - left: %4.2f - right: %4.2f - diff: %4.3f\n", id, acos(Dot(dirEll, dirLeft)), acos(Dot(dirEll, dirRight)), diff);
	printf("oAngle0: %4.2f - oAngle1: %4.2f\n", oAngle0, oAngle1);*/
 
	
	if (diff > 0.3 || oAngle0 > 1.3 || oAngle1 > 1.3) return false;														/// TODO Threshold for right circular cone

 	return true; 
}


void Cone::Draw(int detail)
{
  Ellipses(ellipse)->Draw(detail);
	
  if (detail == 0)
  {
		DrawLine2D(point[0].x, point[0].y, point[1].x, point[1].y, RGBColor::red);
		DrawLine2D(point[1].x, point[1].y, point[2].x, point[2].y, RGBColor::red);
  }
  if (detail > 0)
  {
		Lines(line[LEFT])->Draw(detail-1);
		Lines(line[RIGHT])->Draw(detail-1);
  }
}

const char* Cone::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s ", Gestalt::GetInfo());

  n += snprintf(info_text + n, info_size - n,
    "\nextEllipse: %i\nellipse: %i\nljct: %i\nlines: %i - %i", 
	extEllipse, ellipse, ljct, line[0], line[1]);

	
	return info_text;
}

bool Cone::IsAtPosition(int x, int y)
{
  return Ellipses(ellipse)->IsAtPosition(x, y) ||
	Lines(line[0])->IsAtPosition(x, y) ||
	Lines(line[1])->IsAtPosition(x, y);
}

void Cone::CalculateSignificance()
{
	sig = Ellipses(ellipse)->sig * geoSig;
}

}
