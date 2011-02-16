/**
 * $Id: FormLines.hh,v 1.12 2007/02/18 18:02:48 mxz Exp mxz $
 */

#ifndef Z_FORM_LINES_HH
#define Z_FORM_LINES_HH

#include "Array.hh"
#include "Edgel.hh"
#include "GestaltPrinciple.hh"

namespace Z
{
  
class FormLines : public GestaltPrinciple
{
private:
  bool done;

  void Create();
  void Rank();

public:
  FormLines(VisionCore *vc);
  virtual void Reset();
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate() {return !done;}
};

}

#endif

