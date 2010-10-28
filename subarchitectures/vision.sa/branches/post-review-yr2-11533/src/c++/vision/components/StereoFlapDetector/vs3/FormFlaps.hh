/**
 * $Id: FormFlaps.hh,v 1.3 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"
#include "Closure.hh"

namespace Z
{

class FormFlaps : public GestaltPrinciple
{
private:
  void HaveNewClosure(unsigned i);
  bool StartOfCollinearRun(Closure *clos_i, unsigned pos_i,
      Closure *clos_j, unsigned pos_j);
  bool EndOfCollinearRun(Closure *clos_i, unsigned pos_i,
      Closure *clos_j, unsigned pos_j);
  void NewFlap(Closure *clos_i, Closure *clos_j);
  void Mask();

public:
  FormFlaps(VisionCore *core);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}

