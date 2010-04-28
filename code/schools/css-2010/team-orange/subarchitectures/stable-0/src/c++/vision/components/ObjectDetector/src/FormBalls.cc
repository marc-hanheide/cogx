/**
 * @file FormBalls.cc
 * @author Andreas Richtsfeld
 * @date Mon December 29 2008
 * @version 0.1
 * @brief Implementation of Gestalt-principle FormBalls
 **/

#include "FormBalls.hh"
#include "Ball.hh"
#include "Ellipse.hh"

namespace Z
{

static int CmpBalls(const void *a, const void *b)
{
  if( Balls(*(unsigned*)a)->sig > Balls(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormBalls::Rank()
{
  RankGestalts(Gestalt::BALL, CmpBalls);
}

/**
 *	@brief Masking bad results of balls
 */
void FormBalls::Mask()
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
}

bool FormBalls::NeedsOperate()
{ 
  return needsOperate;	
}

FormBalls::FormBalls(Config *cfg)
: GestaltPrinciple(cfg)
{
  needsOperate = false;

	// TODO define a minimum for the radius of the ball in pixel
	minRadius = 10.;
}

/**
 *	@brief	Calculate balls after creation of all ellipses.
 */
// TODO Könnte auch mit Operate gelöst werden, da Balls schon am Anfang (nach Ellipses) gesucht werden
// Dann müsste diese Funktion Create() heißen!
void FormBalls::OperateNonIncremental()
{
  StartRunTime();

	for(unsigned i=0; i<NumEllipses(); i++)
	{
		// take only unmasked ellipses
		if(Ellipses(i)->masked == UNDEF_ID)
		{
			// if length of a and b is nearly the same => new ball
			double ratio = (Ellipses(i)->a / Ellipses(i)->b);
			double radius = (Ellipses(i)->a + Ellipses(i)->b)/2.;
			if (ratio >= 0.8 && ratio <= 1.2 && radius > minRadius)	NewGestalt(new Ball(i, radius, ratio));
		}
	}

	Rank();
	Mask();

  StopRunTime();
}

/*
**	InformNewGestalt	
*/
// void FormBalls::InformNewGestalt(Gestalt::Type type, unsigned idx)
// {
// 	StartRunTime();
// 	if(VisionCore::config.GetValueInt("FORM_ELLIPSES") != 1) return;

/*  switch(type)
  {
    case Gestalt::E_JUNCTION:
	  NewEJunction(idx);
	  break;
    case Gestalt::L_JUNCTION:
	  NewLJunction(idx);
      break;
    default:
      break;
  }*/
// 	Mask();
// 	Rank();	
// 
//   StopRunTime();
// }

void FormBalls::Operate(bool incremental)
{
/*  StartRunTime();
  Create();
  StopRunTime();
*/
}

}
