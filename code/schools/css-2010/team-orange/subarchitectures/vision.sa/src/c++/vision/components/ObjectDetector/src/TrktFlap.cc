/**
 * @file TrktFlap.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked flaps.
 **/

#include "TrktFlap.hh"
#include "Draw.hh"

namespace Z
{

/**
 * @brief Constructor of class TrktFlap
 * @param ids IDs of the tracked flap, ordered by age
 * @param fd Flap properties (TFlapDefinition), ordered by age
 */
TrktFlap::TrktFlap(Array<unsigned> ids, Array<TFlapDef> fd) : Gestalt(TRKT_FLAP)
{
	for(unsigned i=0; i< ids.Size(); i++)
	{
		tFlapIDs.PushBack(ids[i]);
		tFlaps.PushBack(fd[i]);
	}
	CalculateSignificance();
}

void TrktFlap::Draw(int detail)
{

	RGBColor col;

	if(detail == 0)
		for(unsigned i=0; i<tFlaps.Size(); i++)
		{
			if (tFlaps[i].hypothesised == true) col = RGBColor::red;
			else col = RGBColor::white;
			unsigned trans = 255-(i*192/tFlaps.Size());
	
			DrawLine2D(tFlaps[i].innerJctsIsct[0].x, tFlaps[i].innerJctsIsct[0].y, 
									tFlaps[i].innerJctsIsct[1].x, tFlaps[i].innerJctsIsct[1].y, col, trans);
			DrawLine2D(tFlaps[i].innerJctsIsct[1].x, tFlaps[i].innerJctsIsct[1].y, 
									tFlaps[i].outerJctsIsct[0].x, tFlaps[i].outerJctsIsct[0].y, col, trans);
			DrawLine2D(tFlaps[i].outerJctsIsct[0].x, tFlaps[i].outerJctsIsct[0].y, 
									tFlaps[i].outerJctsIsct[1].x, tFlaps[i].outerJctsIsct[1].y, col, trans);
			DrawLine2D(tFlaps[i].outerJctsIsct[1].x, tFlaps[i].outerJctsIsct[1].y, 
									tFlaps[i].innerJctsIsct[0].x, tFlaps[i].innerJctsIsct[0].y, col, trans);
	
			DrawLine2D(tFlaps[i].innerJctsIsct[2].x, tFlaps[i].innerJctsIsct[2].y, 
									tFlaps[i].innerJctsIsct[3].x, tFlaps[i].innerJctsIsct[3].y, col, trans);
			DrawLine2D(tFlaps[i].innerJctsIsct[3].x, tFlaps[i].innerJctsIsct[3].y, 
									tFlaps[i].outerJctsIsct[2].x, tFlaps[i].outerJctsIsct[2].y, col, trans);
			DrawLine2D(tFlaps[i].outerJctsIsct[2].x, tFlaps[i].outerJctsIsct[2].y, 
									tFlaps[i].outerJctsIsct[3].x, tFlaps[i].outerJctsIsct[3].y, col, trans);
			DrawLine2D(tFlaps[i].outerJctsIsct[3].x, tFlaps[i].outerJctsIsct[3].y, 
									tFlaps[i].innerJctsIsct[2].x, tFlaps[i].innerJctsIsct[2].y, col, trans);
		}

	if(detail > 0)
	{
		unsigned i = detail-1;
		if(detail > tFlaps.Size()) i = tFlaps.Size()-1;

		if (tFlaps[i].hypothesised == true) col = RGBColor::red;
		else col = RGBColor::white;
		unsigned trans = 255;

		DrawLine2D(tFlaps[i].innerJctsIsct[0].x, tFlaps[i].innerJctsIsct[0].y, 
								tFlaps[i].innerJctsIsct[1].x, tFlaps[i].innerJctsIsct[1].y, col, trans);
		DrawLine2D(tFlaps[i].innerJctsIsct[1].x, tFlaps[i].innerJctsIsct[1].y, 
								tFlaps[i].outerJctsIsct[0].x, tFlaps[i].outerJctsIsct[0].y, col, trans);
		DrawLine2D(tFlaps[i].outerJctsIsct[0].x, tFlaps[i].outerJctsIsct[0].y, 
								tFlaps[i].outerJctsIsct[1].x, tFlaps[i].outerJctsIsct[1].y, col, trans);
		DrawLine2D(tFlaps[i].outerJctsIsct[1].x, tFlaps[i].outerJctsIsct[1].y, 
								tFlaps[i].innerJctsIsct[0].x, tFlaps[i].innerJctsIsct[0].y, col, trans);

		DrawLine2D(tFlaps[i].innerJctsIsct[2].x, tFlaps[i].innerJctsIsct[2].y, 
								tFlaps[i].innerJctsIsct[3].x, tFlaps[i].innerJctsIsct[3].y, col, trans);
		DrawLine2D(tFlaps[i].innerJctsIsct[3].x, tFlaps[i].innerJctsIsct[3].y, 
								tFlaps[i].outerJctsIsct[2].x, tFlaps[i].outerJctsIsct[2].y, col, trans);
		DrawLine2D(tFlaps[i].outerJctsIsct[2].x, tFlaps[i].outerJctsIsct[2].y, 
								tFlaps[i].outerJctsIsct[3].x, tFlaps[i].outerJctsIsct[3].y, col, trans);
		DrawLine2D(tFlaps[i].outerJctsIsct[3].x, tFlaps[i].outerJctsIsct[3].y, 
								tFlaps[i].innerJctsIsct[2].x, tFlaps[i].innerJctsIsct[2].y, col, trans);
	}
}

const char* TrktFlap::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s\n\nFlap: %u\nRects: %u - %u\n\nFlaps: ", 
		Gestalt::GetInfo(), tFlapIDs[0], tFlaps[0].rects[0], tFlaps[0].rects[1]);
  	
	for(unsigned i=0; i<tFlapIDs.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "(%u) ", tFlapIDs[i]);

	n += snprintf(info_text + n, info_size - n, "\nRects: ");
	for(unsigned i=0; i<tFlaps.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "(%u %u) ", tFlaps[i].rects[0], tFlaps[i].rects[1]);

	return info_text;
}

bool TrktFlap::IsAtPosition(int x, int y)
{
// 	return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

void TrktFlap::CalculateSignificance()
{

}

}
