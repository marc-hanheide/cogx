/**
 * @file FormCones.hh
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Header of Gestalt-principle FormCones
 **/

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt-Principle FormCones
 */
class FormCones : public GestaltPrinciple
{
private:
  bool needsOperate;

  void Rank();
	void Mask();
  void NewLJunction(unsigned ljct);
  void NewEJunction(unsigned ejct);
  void NewCone(unsigned ell, unsigned eL0, unsigned eL1, unsigned ljct);

public:
  FormCones(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
};

}
