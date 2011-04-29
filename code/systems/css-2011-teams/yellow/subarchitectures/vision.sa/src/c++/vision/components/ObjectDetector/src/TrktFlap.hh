/**
 * @file TrktFlap.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked rectangles
 **/

#ifndef Z_TRKT_FLAP_HH
#define Z_TRKT_FLAP_HH

#include "Vector2.hh"
#include "Gestalt.hh"
#include "VisionCore.hh"
#include "TFlapDefinition.hh"

namespace Z
{

/**
 * @brief This class stores tracked flaps
 */
class TrktFlap : public Gestalt
{
private:

public:
	Array<unsigned> tFlapIDs;
	Array<TFlapDef> tFlaps;

	TrktFlap(Array<unsigned> ids, Array<TFlapDef> fd);
  void CalculateSignificance();
  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& TrktFlaps()
{
  return Gestalts(Gestalt::TRKT_FLAP);
}
inline TrktFlap* TrktFlaps(unsigned id)
{
  return (TrktFlap*)Gestalts(Gestalt::TRKT_FLAP, id);
}
inline unsigned NumTrktFlaps()
{
  return NumGestalts(Gestalt::TRKT_FLAP);
}

}

#endif
