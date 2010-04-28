/**
 * @file TrktRectangle.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked rectangles
 **/

#ifndef Z_TRKT_RECTANGLE_HH
#define Z_TRKT_RECTANGLE_HH

#include "Vector2.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"
#include "RectangleDefinition.hh"

namespace Z
{

/**
 * @brief This class stores tracked rectangles
 */
class TrktRectangle : public Gestalt
{
private:

public:
	Array<unsigned> tRectIDs;				///< IDs of the tracked rectangles.
	Array<RectDef> tRects;					///< Properties (definition) of the tracked rectangles.

	TrktRectangle(Array<unsigned> ids, Array<RectDef> rd);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& TrktRectangles()
{
  return Gestalts(Gestalt::TRKT_RECTANGLE);
}
inline TrktRectangle* TrktRectangles(unsigned id)
{
  return (TrktRectangle*)Gestalts(Gestalt::TRKT_RECTANGLE, id);
}
inline unsigned NumTrktRectangles()
{
  return NumGestalts(Gestalt::TRKT_RECTANGLE);
}

}

#endif
