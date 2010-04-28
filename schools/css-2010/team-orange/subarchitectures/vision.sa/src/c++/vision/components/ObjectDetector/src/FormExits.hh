/**
 * $Id: FormExits.hh,v 1.0 2008/09/02 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"

namespace Z
{

class FormExits : public GestaltPrinciple
{
private:
	bool needsOperate;
	bool findExit;
	bool firstCall;
	unsigned imgWidth;
	unsigned imgHeight;

  void Rank();
	void Mask();
	void FindExit();

public:
  FormExits(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
