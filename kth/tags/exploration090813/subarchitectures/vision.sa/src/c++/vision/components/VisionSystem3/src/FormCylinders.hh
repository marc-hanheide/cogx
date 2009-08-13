/**
 * @file FormCylinders.hh
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Header of Gestalt-principle FormCylinders
 **/

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt-Principle FormCones
 */
class FormCylinders : public GestaltPrinciple
{
private:
  bool needsOperate;

  void Rank();
	void Mask();
  void Create();
  void NewEJunction(unsigned ejct);
  void NewCylinder(unsigned eE0, unsigned eE1, Array<unsigned> &sL, 
		Array<unsigned> *sLVtx);

public:
  FormCylinders(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
