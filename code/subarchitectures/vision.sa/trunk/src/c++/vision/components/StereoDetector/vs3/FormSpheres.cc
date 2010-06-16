/**
 * @file FormSpheres.cc
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Implementation of Gestalt-principle FormSpheres
 **/

#include "FormSpheres.hh"
#include "Sphere.hh"
#include "Ellipse.hh"

namespace Z
{

static int CmpSpheres(const void *a, const void *b)
{
  if( (*(Sphere**)a)->sig > (*(Sphere**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormSpheres::Rank()
{
  RankGestalts(Gestalt::SPHERE, CmpSpheres);
}

/**
 *	@brief Masking bad results of spheres
 */
void FormSpheres::Mask()											// TODO masking!
{
//   for(unsigned i=0; i<NumCones(); i++)
//   {
// 		for(unsigned j=0; j<NumCones(); j++)
// 		{
// 			if(!Cones(i)->IsMasked() && !Cones(j)->IsMasked())
// 			if(Cones(i)->sig < Cones(j)->sig)
// 			{
// 				if(Cones(i)->IsInside(j))
// 				{	
// 					Cones(i)->Mask(j);		 
// 				}
// 			}		  
// 		}
//   }

	// Mask all spheres, which using a masked ellipse.
	for(unsigned i=0; i<NumSpheres(core); i++)
	{
		Sphere *sphere = (Sphere*)core->RankedGestalts(Gestalt::SPHERE, i);
		if(sphere->ellipse->IsMasked())
			sphere->Mask(10000);
	}

}

bool FormSpheres::NeedsOperate()
{ 
  return false;	
}


/**
 * @brief Constructor of Gestalt-principle FormSpheres: Creates the Gestalt Sphere from
 * the underlying Gestalts ellipses.
 */
FormSpheres::FormSpheres(VisionCore *vc) : GestaltPrinciple(vc)
{
	minRadius = 2.;				// minimum radius
	roundness = 0.15;			// required roundness of the sphere (arbitrary threshold)
}

/**
 * @brief Inform principle about new Gestalt.
 * @param type Type of Gestalt
 * @param idx Index of Gestalt
 */
void FormSpheres::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();

	if (type == Gestalt::ELLIPSE)
  	Create(idx);
	
  StopRunTime();
}

/**
 * @brief Try to create a new Gestalt sphere, when new ellipse appears.
 * Creation of shperes depends on "roundness" and minimum radius.
 * @param ellID Index of Gestalt ellipse.
 */
void FormSpheres::Create(unsigned ellID)
{
	Ellipse *ell = Ellipses(core, ellID);
	double ratio = (ell->a / ell->b);
	double radius = (ell->a + ell->b)/2.;
	if (ratio >= (1-roundness) && ratio <= (1+roundness) && radius > minRadius)	
		core->NewGestalt(GestaltPrinciple::FORM_SPHERES, new Sphere(core, ell, radius, ratio));
}

/**
 * @brief Inform principle about new Gestalt.
 */
void FormSpheres::PostOperate()
{
	Rank();
	Mask();
}


}
