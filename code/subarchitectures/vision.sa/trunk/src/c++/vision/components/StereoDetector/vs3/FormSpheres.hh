/**
 * @file FormSpheres.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt-principle FormSpheres
**/

#ifndef Z_FORM_SPHERE_HH
#define Z_FORM_SPHERE_HH

#include "GestaltPrinciple.hh"
#include "VisionCore.hh"

namespace Z
{

/**
 *	@brief Class of Gestalt principle FormSpheres
 */
class FormSpheres : public GestaltPrinciple
{
private:
	double minRadius;
	double roundness;

  void Rank();
	void Mask();
	void Create(unsigned ellID);

public:
	FormSpheres(VisionCore *vc);
  virtual bool NeedsOperate();
	virtual void PostOperate();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);

};

}

#endif