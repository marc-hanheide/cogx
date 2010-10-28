/**
 * @file TrktCube.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked cubes
 **/

#ifndef Z_TRKT_CUBE_HH
#define Z_TRKT_CUBE_HH

#include "Vector2.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"
#include "TCubeDefinition.hh"

namespace Z
{

/**
 * @brief This class stores tracked cubes.
 */
class TrktCube : public Gestalt
{
public:

	Array<unsigned> tCubeIDs;
	Array<TCubeDef> tCubes;

	TrktCube(Array<unsigned> ids, Array<TCubeDef> cd);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& TrktCubes()
{
  return Gestalts(Gestalt::TRKT_CUBE);
}
inline TrktCube* TrktCubes(unsigned id)
{
  return (TrktCube*)Gestalts(Gestalt::TRKT_CUBE, id);
}
inline unsigned NumTrktCubes()
{
  return NumGestalts(Gestalt::TRKT_CUBE);
}

}

#endif
