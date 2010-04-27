
#include "Draw.hh"
#include "BestResult.hh"
#include "ExtEllipse.hh"
#include "Ellipse.hh"
#include "Flap.hh"
#include "Rectangle.hh"
#include "Cube.hh"

namespace Z
{

BestResult::BestResult(unsigned ide) : Gestalt(BEST_RESULT)
{
  identifier = ide;
//   CalculateSignificance();
}


void BestResult::Draw(int detail)
{
// 	printf("Draw best results\n");

	unsigned maxGestalts = 10;		// maximal number of gestalts, which should be shown

	if(NumCubes() > 0)
	{
		for(unsigned i=0; i<NumCubes(); i++)
		{
			if (Cubes(i)->masked == UNDEF_ID) Cubes(i)->Draw(detail);
		}
	}


// 	if(NumExtEllipses > 0)
// 	{
// 		for(unsigned i=0; i<NumExtEllipses(); i++)
// 		{
// 			if(i < maxGestalts) ExtEllipses(i)->Draw(detail);
// 		}
// 	}
// 	else
// 	{
// 		for(unsigned i=0; i<NumEllipses(); i++)
// 		{
// 			if(i < maxGestalts) Ellipses(i)->Draw(detail);
// 		}
// 	}

	if(NumFlaps() > 0)
	{
		for(unsigned i=0; i<NumFlaps(); i++)
		{
			if(i < maxGestalts) Flaps(i)->Draw(detail);
		}
	}
	else
	{
		for(unsigned i=0; i<NumRectangles(); i++)
		{
			if(i < maxGestalts) Rectangles(i)->Draw(detail);
		}
	}

}

const char* BestResult::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s ", Gestalt::GetInfo());

  n += snprintf(info_text + n, info_size - n, "\n");

	
	return info_text;
}

bool BestResult::IsAtPosition(int x, int y)
{
  return false;
}

}
