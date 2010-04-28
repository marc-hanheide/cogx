#include "GestaltPrinciple.hh"

namespace Z
{

class FormBestResults : public GestaltPrinciple
{
private:
  bool needsOperate;

  void Rank();
	void Mask();

public:
  FormBestResults(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
	virtual void OperateNonIncremental();
};

}
