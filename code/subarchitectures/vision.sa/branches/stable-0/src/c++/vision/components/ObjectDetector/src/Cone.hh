/**
 * $Id: Cone.hh,v 1.0 2007/12/13 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CONE_HH
#define Z_CONE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

class Cone : public Gestalt
{
public:
  unsigned extEllipse; 
  unsigned ellipse;
  unsigned line[2];					// the two lines with the L-Junction
  unsigned ljct;						// ljct between lines
  Vector2 point[3]; 				// Left vertex of ellipse
														// cone end (from L-Junction)
														// Right vertex of ellipse
	Vector2 ellipseCenter;		// center point of the ellipse
	double ellRadius;					// circle around the ellipse (center to vertex)
	double topRadius;					// length of center point to the cone top TODO eigentlich nicht ganz ideal (center könnte auch Durchschnitt aller 3 Punkte sein?)
	
	double geoSig;						// significance of geometry (right circular cone)

// Array<unsigned> otherLines;  												// TODO ARI: Welche Linien gehören noch zum cone?

  Cone(unsigned ell, unsigned l0, unsigned l1, unsigned lj);
  bool IsInside(unsigned cone);
	bool CheckGeometry(double &geoSig);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Cones()
{
  return Gestalts(Gestalt::CONE);
}
inline Cone* Cones(unsigned id)
{
  return (Cone*)Gestalts(Gestalt::CONE, id);
}
inline unsigned NumCones()
{
  return NumGestalts(Gestalt::CONE);
}

}

#endif
