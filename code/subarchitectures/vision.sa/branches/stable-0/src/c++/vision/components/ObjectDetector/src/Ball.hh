/**
 * @file Ball.hh
 * @author Andreas Richtsfeld
 * @date Mon December 29 2008
 * @version 0.1
 * @brief Header file of Gestalt Ball
 **/

#ifndef Z_BALL_HH
#define Z_BALL_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the Gestalt Ball.
 */
class Ball : public Gestalt
{
public:
	unsigned ellipse;
	Vector2 center;
	double radius;

  Ball(unsigned e, double rad, double rat);

  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
	virtual void CalculateSignificance(double r);
};

inline Array<Gestalt*>& Balls()
{
  return Gestalts(Gestalt::BALL);
}
inline Ball* Balls(unsigned id)
{
  return (Ball*)Gestalts(Gestalt::BALL, id);
}
inline unsigned NumBalls()
{
  return NumGestalts(Gestalt::BALL);
}

}

#endif
