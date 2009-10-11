/**
 * @file Cube.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of Gestalt Cube.
 **/

/*	TODO ARI: There are different variables for finding cubes from flaps 
**	(CreateFromFlaps) or from one flap with a L-Junction and some lines
**	(TryFlapClosing). So there may be other variables which are undefined.
**
*/


#ifndef Z_CUBE_HH
#define Z_CUBE_HH

#include "Gestalt.hh"
#include "VisionCore.hh"
#include "Vector2.hh"

namespace Z
{

/**
 * @brief Class of Gestalt Cube.
 */
class Cube : public Gestalt
{
public:
  Vector2 corner_points[4][2];		///< [RIGHT/SHARED/LEFT/ISCT][TOP/BOTTOM]
 
  unsigned flap;									///< flap of cube
  unsigned oFlaps[2];							///< two other flaps for the cube	(maybe undefined == UNDEF_ID)
  unsigned closingLJct; 					///< the closing L-Junction (maybe undefined == UNDEF_ID)
  unsigned closingColl; 					///< the closing Collinearity (maybe undefined == UNDEF_ID)

  unsigned jctLines[2]; 					///< lines from the closing LJunction/Collinearity [LEFT/RIGHT]
  Array<unsigned> cLines[2]; 			///< lines, which close the cube from jctLines to flap->outerJunctions

	double parallelity;							///< value for the parallity of the 3x4 parallel cube-edges

	Vector2 intersection_points[3];	// The 3 points from hidden intersection
  Vector2 center;									///< center of the cube-area
  double radius;									///< circumscribed radius

  Vector2 groundCenter; 					///< center of the ground plane from the cube
  Vector2 groundWidth;						///< width of the ground plane
  Vector2 groundDepth; 						///< depth of the ground plane

	double maxRatio;								///< maximum ratio between height, depth and width of the cube

  Cube(unsigned f, unsigned f2, unsigned f3);	// for CreateFromFlaps
  Cube(unsigned f, unsigned lj, unsigned c, unsigned ll, unsigned lr, Array<unsigned> *cL);
  void CalculateSignificance();
  void CalculateSignificance2();
  void OrderAreas();
  void CalculateProperties();
	double CheckParallelity();
  bool CheckW2HRatio();
  bool IsInside(unsigned cube);
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& Cubes()
{
  return Gestalts(Gestalt::CUBE);
}
inline Cube* Cubes(unsigned id)
{
  return (Cube*)Gestalts(Gestalt::CUBE, id);
}
inline unsigned NumCubes()
{
  return NumGestalts(Gestalt::CUBE);
}

}

#endif
