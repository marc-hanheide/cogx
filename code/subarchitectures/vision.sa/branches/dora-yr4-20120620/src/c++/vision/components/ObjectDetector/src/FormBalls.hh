/**
 * @file FormBalls.hh
 * @author Andreas Richtsfeld
 * @date Mon December 29 2008
 * @version 0.1
 * @brief Header file of Gestalt-principle Form_Balls
 **/

#include "GestaltPrinciple.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt principle FormBalls
 */
class FormBalls : public GestaltPrinciple
{
private:
  bool needsOperate;
	double minRadius;

  void Rank();
	void Mask();

public:
	/**
	 *	@brief Constructor of Gestalt-principle FormBalls: Creates the Gestalt Ball from 
	 * 	the underlying Gestalts ellipses.
	 */
  FormBalls(Config *cfg);
  virtual void Operate(bool incremental);
  virtual bool NeedsOperate();
	virtual void OperateNonIncremental();
};

}
