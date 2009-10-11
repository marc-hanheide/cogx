/**
 * $Id: FormExtClosures.hh,v 1.0 2007/10/31 17:27:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"

namespace Z
{

class FormExtClosures : public GestaltPrinciple
{
private:
  bool firstCall;		// firstCall of InformNewGestalt()

  void Rank();
  void Create(unsigned idx);
  void Cr(unsigned idx);
  Array<unsigned> GetSharedLines(unsigned c0, unsigned c1);
  Array<unsigned> GetLines(Array<unsigned> sharedLines, unsigned c0, 
		unsigned c1);
  Array<unsigned> GetOtherLines(Array<unsigned> sharedLines, unsigned c0);
  Array<unsigned> GetLJunctions(unsigned c0, unsigned c1);
  Array<unsigned> GetCollinearities(unsigned c0, unsigned c1);

public:
  FormExtClosures(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
