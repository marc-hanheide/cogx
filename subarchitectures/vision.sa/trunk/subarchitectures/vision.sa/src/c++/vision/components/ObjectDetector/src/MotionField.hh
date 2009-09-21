/**
 * @file MotionField.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores motion fields.
 **/

#ifndef Z_MOTION_FIELD_HH
#define Z_MOTION_FIELD_HH

#include "Vector2.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

/**
 * @brief This class stores motion fields.
 */
class MotionField : public Gestalt
{
private:
	int motionCase;					///< Motion case: 0/1/2/3 = forward/backward/right/left
	Vector2 FOE;						///< Focus of Expansion
	double disMul;					///< Distance Multiplicator for FOE point

public:
	unsigned age;						///< age of field

	MotionField(int mC, Vector2 foe, double dM, double fSig, double a);
	void GetMotion(Vector2 &m, Vector2 p);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& MotionFields()
{
  return Gestalts(Gestalt::MOTION_FIELD);
}
inline MotionField* MotionFields(unsigned id)
{
  return (MotionField*)Gestalts(Gestalt::MOTION_FIELD, id);
}
inline unsigned NumMotionFields()
{
  return NumGestalts(Gestalt::MOTION_FIELD);
}

}

#endif
