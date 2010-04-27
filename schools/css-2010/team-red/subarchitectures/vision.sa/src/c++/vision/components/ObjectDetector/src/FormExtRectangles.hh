/**
 * $Id: FormExtRectangles.hh,v 1.0 2007/10/23 17:27:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"

namespace Z
{

class FormExtRectangles : public GestaltPrinciple
{
private:
  void Rank();
  void Create(Gestalt::Type type, unsigned idx);

public:
  FormExtRectangles(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
