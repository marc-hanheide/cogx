/**
 * @file TrktCube.hh
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief This class stores tracked cubes
 **/

#include "TrktCube.hh"
#include "Draw.hh"
#include "MotionField.hh"

namespace Z
{

TrktCube::TrktCube(Array<unsigned> ids, Array<TCubeDef> cd) : Gestalt(TRKT_CUBE)
{
	for(unsigned i=0; i< ids.Size(); i++)
	{
		tCubeIDs.PushBack(ids[i]);
		tCubes.PushBack(cd[i]);
	}
// 	CalculateSignificance();
	sig = cd[0].sig;
	if(tCubes[0].flap < 10000) sig = sig * 1.2;				/// HACK If flap is not hypothesized => enlarge significance!
}

/**
 * @brief Draw Gestalt TrktCube
 * @param detail Degree of detail.
 */
void TrktCube::Draw(int detail)
{
	for(unsigned i=0; i<tCubes.Size(); i++)
	{
		double transparency = 255.-(i*230./tCubes.Size());
		RGBColor color;
		if(tCubes[i].hypothesised) color = RGBColor::red;
		else color = RGBColor::white;

		if (detail == 0)
		{
		DrawLine2D(tCubes[i].corner_points[3][1].x,
					tCubes[i].corner_points[3][1].y,
					tCubes[i].corner_points[3][0].x,
					tCubes[i].corner_points[3][0].y, color, transparency);
		DrawLine2D(tCubes[i].corner_points[3][1].x,
					tCubes[i].corner_points[3][1].y,
					tCubes[i].corner_points[0][1].x,
					tCubes[i].corner_points[0][1].y, color, transparency);
		DrawLine2D(tCubes[i].corner_points[3][1].x,
					tCubes[i].corner_points[3][1].y,
					tCubes[i].corner_points[2][1].x,
					tCubes[i].corner_points[2][1].y, color, transparency);
		for (unsigned j=0; j<4; j++)
		{
			DrawLine2D(tCubes[i].corner_points[j][0].x,
						tCubes[i].corner_points[j][0].y,
						tCubes[i].corner_points[(j<3?j+1:0)][0].x,
						tCubes[i].corner_points[(j<3?j+1:0)][0].y, color, transparency);		// red
			
		}
		for (unsigned j=0; j<2; j++)
		{
			DrawLine2D(tCubes[i].corner_points[j][1].x,
						tCubes[i].corner_points[j][1].y,
						tCubes[i].corner_points[(j<2?j+1:0)][1].x,
						tCubes[i].corner_points[(j<2?j+1:0)][1].y, color, transparency);	
		}
		DrawLine2D(tCubes[i].corner_points[0][0].x,
					tCubes[i].corner_points[0][0].y,
					tCubes[i].corner_points[0][1].x,
					tCubes[i].corner_points[0][1].y, color, transparency);	//red
		DrawLine2D(tCubes[i].corner_points[1][0].x,
					tCubes[i].corner_points[1][0].y,
					tCubes[i].corner_points[1][1].x,
					tCubes[i].corner_points[1][1].y, color, transparency);
		DrawLine2D(tCubes[i].corner_points[2][0].x,
					tCubes[i].corner_points[2][0].y,
					tCubes[i].corner_points[2][1].x,
					tCubes[i].corner_points[2][1].y, color, transparency);
		}
	
		if (i < (tCubes.Size()-1) && ( detail == 1 || detail ==2))					// show center point lines
			DrawLine2D(tCubes[i].center.x, tCubes[i].center.y, tCubes[i+1].center.x, tCubes[i+1].center.y, RGBColor::red);
	}

	if(detail == 1 || detail == 2) 		// show motion field lines
	{
		Vector2 helper; 
		helper = tCubes[0].center;

		int numberOfMotionFields = NumMotionFields()-1;
		if(tCubes.Size() < numberOfMotionFields) numberOfMotionFields = tCubes.Size()-1;
		for(int i=numberOfMotionFields; i>=0; i--)
		{
			Vector2 motion; 
			MotionFields(i)->GetMotion(motion, helper);
			DrawLine2D(helper.x, helper.y, helper.x - motion.x, helper.y - motion.y, RGBColor::cyan);
			helper = helper - motion;
		}

		if (detail == 2)
		{
			unsigned i=0;
			RGBColor color;
			if(tCubes[i].hypothesised) color = RGBColor::red;
			else color = RGBColor::white;
	
			double transparency = 255.; //-(i*230./tCubes.Size());
		
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[3][0].x,
						tCubes[i].corner_points[3][0].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[0][1].x,
						tCubes[i].corner_points[0][1].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[2][1].x,
						tCubes[i].corner_points[2][1].y, color, transparency);
			for (unsigned j=0; j<4; j++)
			{
				DrawLine2D(tCubes[i].corner_points[j][0].x,
							tCubes[i].corner_points[j][0].y,
							tCubes[i].corner_points[(j<3?j+1:0)][0].x,
							tCubes[i].corner_points[(j<3?j+1:0)][0].y, color, transparency);		// red
				
			}
			for (unsigned j=0; j<2; j++)
			{
				DrawLine2D(tCubes[i].corner_points[j][1].x,
							tCubes[i].corner_points[j][1].y,
							tCubes[i].corner_points[(j<2?j+1:0)][1].x,
							tCubes[i].corner_points[(j<2?j+1:0)][1].y, color, transparency);	
			}
			DrawLine2D(tCubes[i].corner_points[0][0].x,
						tCubes[i].corner_points[0][0].y,
						tCubes[i].corner_points[0][1].x,
						tCubes[i].corner_points[0][1].y, color, transparency);	//red
			DrawLine2D(tCubes[i].corner_points[1][0].x,
						tCubes[i].corner_points[1][0].y,
						tCubes[i].corner_points[1][1].x,
						tCubes[i].corner_points[1][1].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[2][0].x,
						tCubes[i].corner_points[2][0].y,
						tCubes[i].corner_points[2][1].x,
						tCubes[i].corner_points[2][1].y, color, transparency);
		}
	}


	if(detail == 3)			// show corner points
	{
		char text[20];
		for (unsigned k=0; k<4; k++)
			for (unsigned j=0; j<2; j++)
			{
				snprintf(text, 20, "%u/%u", k, j);
				DrawText2D(text, tCubes[0].corner_points[k][j].x, tCubes[0].corner_points[k][j].y, RGBColor::blue);
			}
	}

	if(detail >= 4)			// show the single cubes
	{
		int i = detail - 4;
		if (i >= 0 && i < tCubes.Size())
		{
			RGBColor color;
			if(tCubes[i].hypothesised) color = RGBColor::red;
			else color = RGBColor::white;

			double transparency = 255.; //-(i*230./tCubes.Size());
		
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[3][0].x,
						tCubes[i].corner_points[3][0].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[0][1].x,
						tCubes[i].corner_points[0][1].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[3][1].x,
						tCubes[i].corner_points[3][1].y,
						tCubes[i].corner_points[2][1].x,
						tCubes[i].corner_points[2][1].y, color, transparency);
			for (unsigned j=0; j<4; j++)
			{
				DrawLine2D(tCubes[i].corner_points[j][0].x,
							tCubes[i].corner_points[j][0].y,
							tCubes[i].corner_points[(j<3?j+1:0)][0].x,
							tCubes[i].corner_points[(j<3?j+1:0)][0].y, color, transparency);		// red
				
			}
			for (unsigned j=0; j<2; j++)
			{
				DrawLine2D(tCubes[i].corner_points[j][1].x,
							tCubes[i].corner_points[j][1].y,
							tCubes[i].corner_points[(j<2?j+1:0)][1].x,
							tCubes[i].corner_points[(j<2?j+1:0)][1].y, color, transparency);	
			}
			DrawLine2D(tCubes[i].corner_points[0][0].x,
						tCubes[i].corner_points[0][0].y,
						tCubes[i].corner_points[0][1].x,
						tCubes[i].corner_points[0][1].y, color, transparency);	//red
			DrawLine2D(tCubes[i].corner_points[1][0].x,
						tCubes[i].corner_points[1][0].y,
						tCubes[i].corner_points[1][1].x,
						tCubes[i].corner_points[1][1].y, color, transparency);
			DrawLine2D(tCubes[i].corner_points[2][0].x,
						tCubes[i].corner_points[2][0].y,
						tCubes[i].corner_points[2][1].x,
						tCubes[i].corner_points[2][1].y, color, transparency);
		}
	}
}


/**
 * @brief Get all the information about the Gestalt for the information window.
 */
const char* TrktCube::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s\n\nCube: %u\ntracking age: %u\n\nflaps: %u - %u - %u\nrects: %u - %u - %u\nCubes: ", 
		Gestalt::GetInfo(), tCubeIDs[0], tCubes.Size(), tCubes[0].flap, tCubes[0].oFlaps[0], tCubes[0].oFlaps[1], 
		tCubes[0].rect[0], tCubes[0].rect[1], tCubes[0].rect[2]);

	for(unsigned i=0; i<tCubeIDs.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "(%u) ", tCubeIDs[i]);


	return info_text;
}

bool TrktCube::IsAtPosition(int x, int y)
{
// 	return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

void TrktCube::CalculateSignificance()
{

}

}
