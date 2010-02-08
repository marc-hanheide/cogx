/**
 * @file FormFlapsAri.hh
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Header file of Gestalt-principle FormFlapsAri: Form flaps from rectangles, instead of closures.
 **/

#ifndef Z_FORM_FLAP_ARI_HH
#define Z_FORM_FLAP_ARI_HH

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 * @class FormFlapsAri Estimates flaps from neighboring rectangles.
 */
class FormFlapsAri : public GestaltPrinciple
{
private:
	void Mask();
  void Rank();
  void CreateFlapFromRectangles(unsigned idx);
//   void CreateFlapFromExtRectangles(unsigned idx);
  bool IsExistingFlap(unsigned r0, unsigned r1);
  bool RectanglesSuperposed(unsigned r0, unsigned r1);
  bool SharedLines(unsigned r0, unsigned r1);
  double MeanGap(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts);
  void SortJunctions(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts);
  Array<unsigned> GetSharedLines(unsigned r0, unsigned r1);
//  Array<unsigned> ExtLines(unsigned r0);
  void CloseFlap(unsigned f0, unsigned r0, unsigned r1, unsigned *outerJcts);

public:
  FormFlapsAri(VisionCore *core);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}


#endif