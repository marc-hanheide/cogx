/**
 * @file TrktRectangle.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked rectangles
 **/

#include "TrktRectangle.hh"
#include "Draw.hh"
#include "MotionField.hh"

namespace Z
{

/**
 * @brief Constructor of TrktRectangle
 * @param ids Array of tracked rectangle IDs
 * @param rd Array of rectangle definitions (properties)
 */
TrktRectangle::TrktRectangle(Array<unsigned> ids, Array<RectDef> rd) : Gestalt(TRKT_RECTANGLE)
{
	for(unsigned i=0; i< ids.Size(); i++)
	{
		tRectIDs.PushBack(ids[i]);
		tRects.PushBack(rd[i]);
	}
	CalculateSignificance();
}

/**
 * @brief Draw tracked rectangles
 * @param detail Degree of detail
 */
void TrktRectangle::Draw(int detail)
{
// 	RGBColor color;
// 	if(tRects[i].hypothesised) color = RGBColor::red;
// 	else color = RGBColor::white;

	if(detail < 2)
	{
		for(unsigned i=0; i<tRects.Size(); i++)
		{
			for(unsigned j=0; j<4; j++)
			{
				unsigned k=j+1; if(k>3) k=0;
				DrawLine2D(tRects[i].jctsIsct[j].x, tRects[i].jctsIsct[j].y, tRects[i].jctsIsct[k].x, tRects[i].jctsIsct[k].y, RGBColor::white
								, 255-(i*192/tRects.Size()));
			}
	
			if (i < (tRects.Size()-1) && detail >= 1)					// show center point lines
				DrawLine2D(tRects[i].centerPoint.x, tRects[i].centerPoint.y, tRects[i+1].centerPoint.x, tRects[i+1].centerPoint.y, RGBColor::red);
		}
	}
	
	if(detail >= 2) 		// show motion field lines
	{
		Vector2 helper; 
		helper = tRects[0].centerPoint;

		int numberOfMotionFields = NumMotionFields()-1;
		if(tRects.Size() < numberOfMotionFields) numberOfMotionFields = tRects.Size()-1;
		for(int i=numberOfMotionFields; i>=0; i--)
		{
			Vector2 motion; 
			MotionFields(i)->GetMotion(motion, helper);

			DrawLine2D(helper.x, helper.y, helper.x - motion.x, helper.y - motion.y, RGBColor::white);
			helper = helper - motion;
		}
		unsigned i = detail-2; 
		if (i >= tRects.Size()) i = tRects.Size()-1;
			for(unsigned j=0; j<4; j++)
			{
				unsigned k=j+1; if(k>3) k=0;
				DrawLine2D(tRects[i].jctsIsct[j].x, tRects[i].jctsIsct[j].y, tRects[i].jctsIsct[k].x, tRects[i].jctsIsct[k].y, RGBColor::white
								, 255-(i*192/tRects.Size()));
			}

	}
}

const char* TrktRectangle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s\n\nRectangle: %u\ntracking age: %u\n\nRects: ", 
		Gestalt::GetInfo(), tRectIDs[0], tRects.Size());

	for(unsigned i=0; i<tRectIDs.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "(%u) ", tRectIDs[i]);


	return info_text;
}

bool TrktRectangle::IsAtPosition(int x, int y)
{
// 	return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

void TrktRectangle::CalculateSignificance()
{

}

}
