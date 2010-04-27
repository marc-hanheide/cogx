/**
 * @file LJunction.hh
 * @author Zillich
 * @date 2007
 * @version 0.1
 * @brief Gestalt L-Junction
 **/

#ifndef Z_L_JUNCTION_HH
#define Z_L_JUNCTION_HH

#include "Vector2.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"

namespace Z
{

/**
 * @brief Class of Gestalt L-Junction
 */
class LJunction : public Gestalt
{
private:
  void CalculateSignificance();
  void PrintSig();

public:
  unsigned line[2]; 					///< left and right arm
  double sense[2];						///< whether line dirs point to or from junction
  unsigned near_point[2];			///< which point of each line is nearer to the jct
  Vector2 dir[2];							///< directions of arms, pointing outwards
  double dist[2];							///< distance to left and right endpoints
  Vector2 isct;								///< intersection point
  Vector2 dir_i;							///< direction points inwards, bisects angle
  double r;										///< distance between endpoints
  double col_dist;						///< color distance between inside mean colors

  LJunction(unsigned i, unsigned j, unsigned end_i, unsigned end_j, const Vector2 &inter);
  void Recalc();
  virtual void Draw(int detail = 0);
  virtual void DrawInfo();
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
  double Gap(unsigned side);
  double OpeningAngle();
};

inline Array<Gestalt*>& LJunctions()
{
  return Gestalts(Gestalt::L_JUNCTION);
}
inline LJunction* LJunctions(unsigned id)
{
  return (LJunction*)Gestalts(Gestalt::L_JUNCTION, id);
}
inline unsigned NumLJunctions()
{
  return NumGestalts(Gestalt::L_JUNCTION);
}

extern bool Inside(unsigned a, unsigned b);
extern bool Opposing(unsigned a, unsigned b);

}

#endif

