/**
 * $Id: FormWalls.hh,v 1.0 2008/06/16 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"
#include "Vector2.hh"

namespace Z
{

class FormWalls : public GestaltPrinciple
{
private:
  bool needsOperate;
	bool firstCall;
	unsigned imgWidth;
	unsigned imgHeight;

  void Rank();
	void Mask();
	void FindContinousLines();
	void FindConnectedLines();
	void FindLConnectedLines();
	void FindLongWallLine();
	void FindWallWithGap();
	void NewWall(bool corner, bool longLine, unsigned *wallLines, Vector2 *borderPoints, Vector2 *point, unsigned cornerPos);

public:
  FormWalls(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
