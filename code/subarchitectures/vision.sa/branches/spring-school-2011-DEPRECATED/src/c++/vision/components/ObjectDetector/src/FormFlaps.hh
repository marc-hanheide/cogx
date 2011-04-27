/**
 * @file Flap.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Header file of Gestalt-principle FormFlaps.
 **/

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @brief Class FormFlaps
 */
class FormFlaps : public GestaltPrinciple
{
private:
	void Mask();
  void Rank();
  void CreateFlapFromRectangles(unsigned idx);
  void CreateFlapFromExtRectangles(unsigned idx);
  bool IsExistingFlap(unsigned r0, unsigned r1);
  bool RectanglesSuperposed(unsigned r0, unsigned r1);
  bool SharedLines(unsigned r0, unsigned r1);
  double MeanGap(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts);
  void OrderJunctions(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts);
  Array<unsigned> GetSharedLines(unsigned r0, unsigned r1);
//  Array<unsigned> ExtLines(unsigned r0);
  void CloseFlap(unsigned f0, unsigned r0, unsigned r1, unsigned *outerJcts);

public:
  FormFlaps(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
