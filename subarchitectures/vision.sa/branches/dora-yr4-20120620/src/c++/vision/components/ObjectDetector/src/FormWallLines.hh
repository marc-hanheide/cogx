/**
 * $Id: FormWallLines.hh,v 1.0 2008/07/02 13:47:03 mxz Exp mxz $
 */

#include "GestaltPrinciple.hh"
#include "Vector2.hh"

namespace Z
{

class FormWallLines : public GestaltPrinciple
{
private:
  bool needsOperate;
	bool firstCall;
	unsigned imgWidth;
	unsigned imgHeight;

	Array<unsigned> outLines;									// lines, out of the image
	Array<unsigned> outLinesNearPoints; 			// near points of the outLines
	Array<unsigned> outLinesBorders;					// 0=left / 1=top / 2=right / 3=bottom

  void Rank();
	void Mask();
	void SearchLinesForWalls();
	void GetNextLines(unsigned line, unsigned lineNearPoint, Array<unsigned> &nextLines, Array<unsigned> &nextLinesNearPoints, 
			Array<unsigned> &lastLines);//, bool firstLine);
	void CreateWallLine(unsigned outLine, unsigned outLineNearPoint, unsigned outLineBorder, Array<unsigned> &nextLines, 
			Array<unsigned> &nextLinesNearPoints, Array<unsigned> &lastLines);
	bool IsWallLine(Array<unsigned> nLines);
	void MakeWallLine(unsigned outLine, unsigned outLineBorder, Array<unsigned> &oneLine, Array<unsigned> &oneLineNearPoint);

public:
  FormWallLines(Config *cfg);
  virtual void Operate(bool incremental);
	virtual void OperateNonIncremental();
  virtual bool NeedsOperate();
  virtual void Reset(const Image *img);
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
