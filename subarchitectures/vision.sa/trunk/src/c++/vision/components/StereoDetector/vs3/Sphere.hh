/**
 * @file Sphere.hh
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt Sphere.
 **/

#ifndef Z_SPHERE_HH
#define Z_SPHERE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 *	@brief Class, describing the Gestalt Sphere.
 */
class Sphere : public Gestalt
{
public:
	Ellipse *ellipse;
	double radius;

  Sphere(VisionCore* vc, Ellipse *e, double rad, double rat);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
	virtual void CalculateSignificance(double r);
};

inline Array<Gestalt*>& Spheres(VisionCore *core)
{
  return core->Gestalts(Gestalt::SPHERE);
}
inline Sphere* Spheres(VisionCore *core, unsigned id)
{
  return (Sphere*)core->Gestalts(Gestalt::SPHERE, id);
}
inline unsigned NumSpheres(VisionCore *core)
{
  return core->NumGestalts(Gestalt::SPHERE);
}

}

#endif
