/**
 * $Id: FormCorners.hh,v 1.3 2007/11/26 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"

namespace Z
{

class FormCorners : public GestaltPrinciple
{
private:
  Array<unsigned> cand;	// candidates for new corners: line with >= 2 l-jcts
  Array<unsigned> candSide; // side of new candidate (START/END)

  void Rank();
  void Create();
  void NewCorner(Array<unsigned> &lines, Array<unsigned> &jcts, 
	Array<unsigned> &near_point);
  unsigned CornerExists(Array<unsigned> lines);
  unsigned CheckCollinearity(unsigned l0, unsigned l1);

public:
  FormCorners(Config *cfg);
  void InformNewCandidate(unsigned l, unsigned side);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
