/**
 * @file Ball.cc
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Header file of Gestalt Ball
 **/

#ifndef Z_Object_HH
#define Z_Object_HH

#include "VisionCore.hh"
#include "Gestalt.hh"
#include "ObjectTracker.hh"
#include "Cone.hh"
#include "Cube.hh"
#include "Cylinder.hh"
//#include "TObject.hh"

namespace Z
{

class Object : public Gestalt
{
public:
  Gestalt::Type type;								// type of the object
  unsigned gestalt_id;							// gestalt-id of the object
  unsigned tracked_id;							// true, if Object is tracked

	CubeDef cuD;											// properties of a cube
	ConeDef coD;											// properties of a cone
	CylDef cyD;												// properties of a cylinder
	BalDef baD;												// properties of a ball
	WalDef waD;												// properties of a wall
	ExitDef exD;											// properties of an exit

  Object(Gestalt::Type t, unsigned i, CubeDef cd);
  Object(Gestalt::Type t, unsigned i, ConeDef cd);
  Object(Gestalt::Type t, unsigned i, CylDef cd);
  Object(Gestalt::Type t, unsigned i, BalDef bd);
  Object(Gestalt::Type t, unsigned i, WalDef wd);
  Object(Gestalt::Type t, unsigned i, ExitDef ed);

	bool IsInside(unsigned object);
  void Tracked(unsigned id, Vector2 trGrCe);	// mark object as tracked
  void CalculateSignificance();

  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Objects()
{
  return Gestalts(Gestalt::OBJECT);
}
inline Object* Objects(unsigned id)
{
  return (Object*)Gestalts(Gestalt::OBJECT, id);
}
inline unsigned NumObjects()
{
  return NumGestalts(Gestalt::OBJECT);
}

}

#endif
