/**
 * @file FormCubes.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Header of class FormCubes.
 **/

#include "GestaltPrinciple.hh"
#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"
#include "Cube.hh"

namespace Z
{

/**
 * @brief Class of Gestalt principle FormCubes.
 */
class FormCubes : public GestaltPrinciple
{
private:
	/**
	* @brief Class for cube candidates.
	* TODO Ist CubeCand noch notwendig?
	*/
  class CubeCand
  {
	public:
		unsigned flap;													///< Flap
		Array<unsigned> lines[2];								///< Connected lines LEFT/RIGHT of flap
		Array<unsigned> linesNearPoints[2]; 		///< Near points of the lines	[LEFT/RIGHT]		// TODO ARI: notwendig?
		Array<unsigned> lastLines[2];						///< The foregoing lines of lines [LEFT/RIGHT]
	
		CubeCand(unsigned f, Array<unsigned> *l, Array<unsigned> *lNP);
  };


  Array<CubeCand*> cands;										///< Cube candidates
  
  void Rank();
  void Mask();
  void Create(unsigned flap);
  bool CreateFromFlaps(unsigned flap);
  void CloseFlap(unsigned flap);
  void GetLines(unsigned flap, Array<unsigned> *lines, Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines);
  void GetNextLines(unsigned line, unsigned lineEnd, Array<unsigned> *lines, 
		Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines, unsigned fSide, bool firstLine);
  void NewCollinearity(unsigned col);
  void NewLJunction(unsigned ljct);
  void UpdateCubeCand(unsigned cand);
  bool TryFlapClosing(unsigned flap, Array<unsigned> *lines, Array<unsigned> *linesNearPoints, 
		Array<unsigned> *lastLines);
  bool TryFlapClosingWithCollinearity(unsigned flap, Array<unsigned> *lines, 
		Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines);

public:
  FormCubes(VisionCore *core);
  virtual void Reset();
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
