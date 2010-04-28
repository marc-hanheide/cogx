/**
 * @file FormCubes.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Header of class FormCubes.
 **/

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Class of Gestalt principle FormCubes.
 */
class FormCubes : public GestaltPrinciple
{
private:
  class CubeCand
  {
	public:
		unsigned flap;
		Array<unsigned> lines[2];								// connected lines LEFT/RIGHT of flap
		Array<unsigned> linesNearPoints[2]; 		// near points of the lines	[LEFT/RIGHT]		// TODO ARI: notwendig?
		Array<unsigned> lastLines[2];						// the foregoing lines of lines [LEFT/RIGHT]
	
		CubeCand(unsigned f, Array<unsigned> *l, Array<unsigned> *lNP);
  };

  Array<CubeCand*> cands;
  
  void Rank();
  void Mask();
  void Create(unsigned flap);
  bool CreateFromFlaps(unsigned flap);
  void CloseFlap(unsigned flap);
  void GetLines(unsigned flap, Array<unsigned> *lines, 
  	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines);
  void GetNextLines(unsigned line, unsigned lineEnd, Array<unsigned> *lines,
  	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines, 
  	unsigned fSide, bool firstLine);
  void NewCollinearity(unsigned col);
  void NewLJunction(unsigned ljct);
  void UpdateCubeCand(unsigned cand);
  bool TryFlapClosing(unsigned flap, Array<unsigned> *lines,
  	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines);
  bool TryFlapClosingWithCollinearity(unsigned flap, Array<unsigned> *lines,
  	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines);


//  void CloseFlap(unsigned f0, unsigned r0, unsigned r1, unsigned *outerJcts);

  
public:
  FormCubes(Config *cfg);
  virtual void Reset(const Image *img);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
