/**
 * @file Cylinder.hh
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Header of Gestalt Cylinders
 **/

#ifndef Z_CYLINDER_HH
#define Z_CYLINDER_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

/**
 * @brief Class Gestalt Cylinder
 */
class Cylinder : public Gestalt
{
private:
	double geometry;								// value for good geometry of an cylinder (less is better)

  void CalculateSignificance();
  void CalculateEqualVertex();
	bool CheckGeometry();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);

public:
  bool equalVertex;												// connect equal vertices of both ellipses, if true
  unsigned extEllipses[2];								// extended ellipses
  Array<unsigned> sharedLines;						// sharedLines	
  Array<unsigned> sharedLinesVertex[2];		// sharedLinesVertex[??/??]	TODO
	Vector2 center;													// center of the cone, estimated as point between the two ellipse-center points
	double radius;													// maximum distance from center to ellipse-center + ellipse->b (end of cylinder)

  Cylinder(unsigned ee0, unsigned ee1, Array<unsigned> &sL, Array<unsigned> *sLVtx);
  bool IsInside(unsigned cone);
  void AddSharedLines(Array<unsigned> &sL, Array<unsigned> *sLVtx);
  virtual void Draw(int detail = 0);
};

inline Array<Gestalt*>& Cylinders()
{
  return Gestalts(Gestalt::CYLINDER);
}
inline Cylinder* Cylinders(unsigned id)
{
  return (Cylinder*)Gestalts(Gestalt::CYLINDER, id);
}
inline unsigned NumCylinders()
{
  return NumGestalts(Gestalt::CYLINDER);
}

}

#endif
