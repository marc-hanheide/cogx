/**
 * $Id: FormExtEllipses.hh,v 1.0 2007/12/06 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"

namespace Z
{

class FormExtEllipses : public GestaltPrinciple
{
private:
  bool needsOperate;
	
  void Rank();
  void Create();

public:
  FormExtEllipses(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
  void InformNewEJunction(unsigned idx);
  void NewExtEllipse(unsigned eJct);
  void ExtendExtEllipse(unsigned eJct);

};

}
