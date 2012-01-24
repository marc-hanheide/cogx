
#ifndef Z_BESTRESULT_HH
#define Z_BESTRESULT_HH

#include "Gestalt.hh"
#include "VisionCore.hh"

namespace Z
{

class BestResult : public Gestalt
{
public:
  unsigned identifier; 

  BestResult(unsigned identifier);

  virtual void Draw(int detail = 0);
  virtual const char* GetInfo();
  virtual bool IsAtPosition(int x, int y);
};

inline Array<Gestalt*>& BestResults()
{
  return Gestalts(Gestalt::BEST_RESULT);
}
inline BestResult* BestResults(unsigned id)
{
  return (BestResult*)Gestalts(Gestalt::BEST_RESULT, id);
}
inline unsigned NumBestResults()
{
  return NumGestalts(Gestalt::BEST_RESULT);
}

}

#endif
