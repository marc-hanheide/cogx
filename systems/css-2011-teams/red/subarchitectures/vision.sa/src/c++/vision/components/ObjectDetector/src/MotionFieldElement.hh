/**
 * @file MotionFieldElement.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class represents a element of the motion field
 **/

#ifndef Z_MOTION_FIELD_ELEMENT_HH
#define Z_MOTION_FIELD_ELEMENT_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

/**
 * @brief Class MotionField: The class represents one element of the motion field
 */
class MotionFieldElement : public Gestalt
{
public:
	unsigned newID; 				///< ID of new L-Junction
	unsigned oldID;					///< ID of old L-Junction
	Vector2 point[2];				///< Start and end point of vector (old to new L-Junction)
	Vector2 dir;						///< Direction of vector
	double length;					///< Length of vector

	int motionCase;					///< Motion case: 0/1/2/3 = forward/backward/right/left
	Vector2 FOE;						///< Focus of Expansion
	double disMul;					///< Distance Multiplicator for FOE point
	double FOEsig;					///< Significance for FOE

  MotionFieldElement(unsigned nId, unsigned oId, Vector2 newIsct, Vector2 oldIsct, double s);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
	void SetFOE(int mC, Vector2 foe, double dM, double fSig);
};

inline Array<Gestalt*>& MotionFieldElements()
{
  return Gestalts(Gestalt::MOTION_FIELD_ELEMENT);
}
inline MotionFieldElement* MotionFieldElements(unsigned id)
{
  return (MotionFieldElement*)Gestalts(Gestalt::MOTION_FIELD_ELEMENT, id);
}
inline unsigned NumMotionFieldElements()
{
  return NumGestalts(Gestalt::MOTION_FIELD_ELEMENT);
}

}

#endif
