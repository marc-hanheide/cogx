/**
 * @file Cube.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Class file of Gestalt Cube.
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Rectangle.hh"
#include "Flap.hh"
#include "Cube.hh"

namespace Z
{

/**
 * @brief Constructor for Gestalt Cube for creation from flaps.
 * @param f Flap 1
 * @param f Flap 2
 * @param f Flap 3
 */
Cube::Cube(unsigned f, unsigned f2, unsigned f3) : Gestalt(CUBE)
{
	maxRatio = 5.;

  flap = f;
  oFlaps[0] = f2;
  oFlaps[1] = f3;
  closingLJct = UNDEF_ID;
  closingColl = UNDEF_ID;
	
  // get the corner_points
  corner_points[1][0] = (LJunctions(Flaps(f)->innerJcts[1])->isct +
						 LJunctions(Flaps(f)->innerJcts[3])->isct +	
						 LJunctions(Flaps(f2)->innerJcts[1])->isct +
						 LJunctions(Flaps(f2)->innerJcts[3])->isct +
						 LJunctions(Flaps(f3)->innerJcts[1])->isct +
						 LJunctions(Flaps(f3)->innerJcts[3])->isct)/6;

  corner_points[1][1] = (LJunctions(Flaps(f)->innerJcts[2])->isct +
					     LJunctions(Flaps(f)->innerJcts[0])->isct)/2;
  corner_points[0][0] = (LJunctions(Flaps(f2)->innerJcts[2])->isct +
					     LJunctions(Flaps(f2)->innerJcts[0])->isct)/2;
  corner_points[2][0] = (LJunctions(Flaps(f3)->innerJcts[2])->isct +
					     LJunctions(Flaps(f3)->innerJcts[0])->isct)/2;

  corner_points[0][1] = (LJunctions(Flaps(f)->outerJcts[1])->isct +
					     LJunctions(Flaps(f2)->outerJcts[3])->isct)/2;
  corner_points[3][0] = (LJunctions(Flaps(f2)->outerJcts[1])->isct +
					     LJunctions(Flaps(f3)->outerJcts[3])->isct)/2;
  corner_points[2][1] = (LJunctions(Flaps(f3)->outerJcts[1])->isct +
					     LJunctions(Flaps(f)->outerJcts[3])->isct)/2;

  // Order the corner points to left/right/top area
  OrderAreas();

  // hidden corner of the cube => 31 = 01-(10-20) = 21-(10-00) = 30-(10-11) 
  intersection_points[0] = corner_points[0][1] - (corner_points[1][0] - corner_points[2][0])*0.9;
  intersection_points[1] = corner_points[2][1] - (corner_points[1][0] - corner_points[0][0])*0.9;
  intersection_points[2] = corner_points[3][0] - (corner_points[1][0] - corner_points[1][1])*0.9;
    
  corner_points[3][1] = (intersection_points[0] + intersection_points[1] + intersection_points[2])/3;

  
  CalculateProperties();
	parallelity = CheckParallelity();
  if(parallelity > 3.0) Mask(10000);																	/// TODO Threshold for parallelity
	else if(!CheckW2HRatio()) Mask(10001);
  CalculateSignificance2();
}

/**
 * @brief Constructor for Gestalt Cube for creation from flaps.
 * @param f Flap
 * @param lj L-Junction
 * @param c Collinearity
 * @param ll Junction lines left
 * @param lr Junction lines right
 * @param cL Array with ids of closing lines
 */
Cube::Cube(unsigned f, unsigned lj, unsigned c, unsigned ll, unsigned lr, Array<unsigned> *cL) : Gestalt(CUBE)
{
	maxRatio = 5.;

  flap = f;
  oFlaps[0] = UNDEF_ID;
  oFlaps[1] = UNDEF_ID;
  closingLJct = lj;	
  closingColl = c;

  // get corner points => they are not sorted
  corner_points[0][0] = LJunctions(Flaps(flap)->outerJcts[0])->isct;
  corner_points[0][1] = LJunctions(Flaps(flap)->outerJcts[1])->isct;
  corner_points[1][0] = LJunctions(Flaps(flap)->innerJcts[1])->isct;
  corner_points[1][1] = LJunctions(Flaps(flap)->innerJcts[0])->isct;
  corner_points[2][0] = LJunctions(Flaps(flap)->outerJcts[2])->isct;
  corner_points[2][1] = LJunctions(Flaps(flap)->outerJcts[3])->isct;
  if (closingLJct != UNDEF_ID)
	corner_points[3][0] = LJunctions(lj)->isct;
  else
 	corner_points[3][0] = Collinearities(c)->vertex;
  
  // hidden corner of the cube => 31 = 01-(10-20) = 21-(10-00) = 30-(10-11) 
  // Order the corner points to left/right/top area
  OrderAreas();

//   Vector2 intersection_points[3];	// The 3 points from hidden intersection
  intersection_points[0] = corner_points[0][1] - (corner_points[1][0] - corner_points[2][0])*0.9;
  intersection_points[1] = corner_points[2][1] - (corner_points[1][0] - corner_points[0][0])*0.9;
  intersection_points[2] = corner_points[3][0] - (corner_points[1][0] - corner_points[1][1])*0.9;
    
  corner_points[3][1] = (intersection_points[0] + intersection_points[1] + 
  						 intersection_points[2])/3;


  // L-junction and closing lines
  jctLines[LEFT] = ll; 
  jctLines[RIGHT] = lr;
  cLines[0] = cL[0];
  cLines[1] = cL[1];
	
  // Add Cube to lines?					// TODO ARI: Ist das notwendig?
  Lines(jctLines[LEFT])->AddCube(id);
  Lines(jctLines[RIGHT])->AddCube(id);
  for(unsigned i=0; i<cLines[0].Size(); i++)	
	Lines(cLines[0][i])->AddCube(id);
  for(unsigned i=0; i<cLines[1].Size(); i++)	
	Lines(cLines[1][i])->AddCube(id);
  
  CalculateProperties();
	parallelity = CheckParallelity();
  if(parallelity > 3.0) Mask(10000);																	/// TODO Masking Threshold for parallelity
	else if(!CheckW2HRatio()) Mask(10001);															/// TODO Masking Threshold for H2W-Ratio
  CalculateSignificance();
}

/**
 *	@brief Order the 3 visible areas of the cube if the cube lies with one area
 *	on the ground plane => 1/0 and 3/1 doesn´t change
 *	
 */
void Cube::OrderAreas()
{
  Vector2 cp[4][2];						// copy corner points
  for(unsigned i=0; i<4; i++)
		for(unsigned j=0; j<2; j++)
			cp[i][j] = corner_points[i][j];
	
  // first area a0 => [LEFT/SHARED][TOP/BOTTOM]=[0/1][0/1]
//   double max_x_0 = Max(corner_points[0][0].x, corner_points[0][1].x);
//   double max_x_1 = Max(corner_points[1][0].x, corner_points[1][1].x);
  double max_y_0 = Max(corner_points[0][0].y, corner_points[0][1].y);
  double max_y_1 = Max(corner_points[1][0].y, corner_points[1][1].y);
//   double min_x_0 = Min(corner_points[0][0].x, corner_points[0][1].x);
//   double min_x_1 = Min(corner_points[1][0].x, corner_points[1][1].x);

//  double max_x_a0 = Max(max_x_0, max_x_1);
	double max_y_a0 = Max(max_y_0, max_y_1);
//  double min_x_a0 = Min(min_x_0, min_x_1);

  // second area => [SHARED/RIGHT][TOP/BOTTOM]=[1/2][0/1]
//   double max_x_2 = Max(corner_points[2][0].x, corner_points[2][1].x);
  double max_y_2 = Max(corner_points[2][0].y, corner_points[2][1].y);
//   double min_x_2 = Min(corner_points[2][0].x, corner_points[2][1].x);
	
//  double max_x_a1 = Max(max_x_1, max_x_2);
	double max_y_a1 = Max(max_y_1, max_y_2);
//  double min_x_a1 = Min(min_x_1, min_x_2);

  // third area => [LEFT/SHARED/RIGHT/ISCT][TOP]=[0/1/2/3][0]
//   double max_x_3 = Max(corner_points[0][0].x, corner_points[1][0].x);
//   double max_x_4 = Max(corner_points[2][0].x, corner_points[3][0].x);
  double max_y_3 = Max(corner_points[0][0].y, corner_points[1][0].y);
  double max_y_4 = Max(corner_points[2][0].y, corner_points[3][0].y);
//   double min_x_3 = Min(corner_points[0][0].x, corner_points[1][0].x);
//   double min_x_4 = Min(corner_points[2][0].x, corner_points[3][0].x);

//  double max_x_a2 = Max(max_x_3, max_x_4);
	double max_y_a2 = Max(max_y_3, max_y_4);
//  double min_x_a2 = Min(min_x_3, min_x_4);	
	

  // which area is the top area?
  if (max_y_a0 < max_y_a1 && max_y_a0 < max_y_a2)				// 0 < 1,2 => turn left
  {
		corner_points[0][0] = cp[1][1];
		corner_points[2][0] = cp[0][0];
		corner_points[3][0] = cp[0][1];

		corner_points[1][1] = cp[2][0];
		corner_points[2][1] = cp[3][0];

		corner_points[0][1] = cp[2][1];
		corner_points[1][1] = cp[2][0];
  }	
  else if (max_y_a1 < max_y_a0 && max_y_a1 < max_y_a2)	// 1 < 0,2 => turn right
  {
		corner_points[0][0] = cp[2][0];
		corner_points[2][0] = cp[1][1];
		corner_points[3][0] = cp[2][1];

		corner_points[1][1] = cp[0][0];
		corner_points[2][1] = cp[0][1];

		corner_points[0][1] = cp[3][0];
		corner_points[1][1] = cp[0][0];
  }
}

/**
 * @brief	Calculate properties of the cube
 */
void Cube::CalculateProperties()
{
  // Properties for masking
  center = (corner_points[0][0] + corner_points[0][1] +
			corner_points[3][0] + corner_points[1][1] +
			corner_points[2][0] + corner_points[2][1])/6.;

  double r0 = (corner_points[0][0]-center).Length();
  double r1 = (corner_points[0][1]-center).Length();
  double r2 = (corner_points[3][0]-center).Length();
  double r3 = (corner_points[1][1]-center).Length();
  double r4 = (corner_points[2][0]-center).Length();
  double r5 = (corner_points[2][1]-center).Length();
  
  radius = Max(r0, r1);
  radius = Max(radius, r2);
  radius = Max(radius, r3);
  radius = Max(radius, r4);
  radius = Max(radius, r5);
	
  // Properties of ground plane
  groundCenter = (corner_points[0][1] + corner_points[1][1] +
				  corner_points[2][1] + corner_points[3][1])/4;
}


/**
 *	@brief Check parallelity of the 4 cube edges in the 3 directions
 *	TODO This is only for masking => use it for significance!
 */
double Cube::CheckParallelity()	
{
	// value for parallelity
	double parallelity = 0.;

  // Threshold for parallelity
//  double par = M_PI/6;																		/// TODO Threshold for parallelity
	
  // check parallelity between x/0 and x/1 - lines
  Vector2 edge[4];
  edge[0] = (corner_points[0][0] - corner_points[0][1]);
  edge[1] = (corner_points[1][0] - corner_points[1][1]);
  edge[2] = (corner_points[2][0] - corner_points[2][1]);
  edge[3] = (corner_points[3][0] - corner_points[3][1]);

  double dir[4];
  dir[0] = PolarAngle(edge[0]);
  dir[1] = PolarAngle(edge[1]);
  dir[2] = PolarAngle(edge[2]);
  dir[3] = PolarAngle(edge[3]);

  for(unsigned i=0; i<4; i++)
  {
 		int j=i+1;
 		if (j==4) j=0;
		parallelity += fabs(dir[i]-dir[j]);
	}
//   for(unsigned i=0; i<4; i++)
//   {
// 		int j=i+1;
// 		if (j==4) j=0;
// 		if ((dir[i]-dir[j]) > par)
// 			return false;
//   }	

  edge[0] = (corner_points[0][0] - corner_points[1][0]);
  edge[1] = (corner_points[0][1] - corner_points[1][1]);
  edge[2] = (corner_points[3][1] - corner_points[2][1]);
  edge[3] = (corner_points[3][0] - corner_points[2][0]);

  dir[0] = PolarAngle(edge[0]);
  dir[1] = PolarAngle(edge[1]);
  dir[2] = PolarAngle(edge[2]);
  dir[3] = PolarAngle(edge[3]);

  for(unsigned i=0; i<4; i++)
  {
 		int j=i+1;
 		if (j==4) j=0;
		parallelity += fabs(dir[i]-dir[j]);
	}  
// 	for(unsigned i=0; i<4; i++)
//   {
// 		int j=i+1;
// 		if (j==4) j=0;
// 		if ((dir[i]-dir[j]) > par)
// 			return false;
//   }	
  
  // check parallelity 
  edge[0] = (corner_points[0][0] - corner_points[3][0]);
  edge[1] = (corner_points[1][0] - corner_points[2][0]);
  edge[2] = (corner_points[1][1] - corner_points[2][1]);
  edge[3] = (corner_points[0][1] - corner_points[3][1]);

  dir[0] = PolarAngle(edge[0]);
  dir[1] = PolarAngle(edge[1]);
  dir[2] = PolarAngle(edge[2]);
  dir[3] = PolarAngle(edge[3]);

  for(unsigned i=0; i<4; i++)
  {
 		int j=i+1;
 		if (j==4) j=0;
		parallelity += fabs(dir[i]-dir[j]);
	}
//   for(unsigned i=0; i<4; i++)
//   {
// 		int j=i+1;
// 		if (j==4) j=0;
// 		if ((dir[i]-dir[j]) > par)
// 			return false;
//   }	

  return parallelity;
}

/**
**	CheckW2HRatio()
**	Check ratio between width, depth and heigt of an cube. Ratio must be smaller than 10:1
*/
bool Cube::CheckW2HRatio()
{
	bool ratio = true;

	Vector2 meanW = ((corner_points[1][0] - corner_points[2][0]) + (corner_points[1][1] - corner_points[2][1]) +
									(corner_points[0][1] - corner_points[3][1]) + (corner_points[0][0] - corner_points[3][0]))/4.;
	Vector2 meanD = ((corner_points[3][1] - corner_points[2][1]) + (corner_points[0][1] - corner_points[1][1]) +
									(corner_points[0][0] - corner_points[1][0]) + (corner_points[3][0] - corner_points[2][0]))/4.;
	Vector2 meanH = ((corner_points[2][0] - corner_points[2][1]) + (corner_points[1][0] - corner_points[1][1]) +
									(corner_points[0][0] - corner_points[0][1]) + (corner_points[3][0] - corner_points[3][1]))/4.;

	double absMeanW = meanW.Length();
	double absMeanD = meanD.Length();
	double absMeanH = meanH.Length();

	double wdRatio, dhRatio, hwRatio;
	wdRatio = absMeanW/absMeanD;
	dhRatio = absMeanD/absMeanH;
	hwRatio = absMeanH/absMeanW;

	if(wdRatio > maxRatio || wdRatio < 1./maxRatio) ratio = false;				/// TODO ratio for "true" or "false" cubes
	if(dhRatio > maxRatio || dhRatio < 1./maxRatio) ratio = false;
	if(hwRatio > maxRatio || hwRatio < 1./maxRatio) ratio = false;

/// TODO HACK ARI: Warnmeldung W-D-H falsch
// if(!ratio)
// {	printf("mean W-D-H of cube %u: %4.2f - %4.2f - %4.2f\n", id, absMeanW, absMeanD, absMeanH);
// 	printf("	ratio: %4.2f - %4.2f - %4.2f\n", wdRatio, dhRatio, hwRatio);
// }
	return ratio;
}

/**
 *	@brief Check if another cube is at the same position.
 *	@param cube Cube, which could be inside this cube.
 */
bool Cube::IsInside(unsigned cube)
{
  if((Cubes(cube)->center - center).Length() < radius) return true;
  else return false; 
}

/**
 * @brief Draw Gestalt Cube.
 * @param detail Degree of detail.
 */
void Cube::Draw(int detail)
{
  if (detail == 0 || detail == 1 || detail ==3)
  {
	DrawLine2D(corner_points[3][1].x,
			   corner_points[3][1].y,
			   corner_points[3][0].x,
			   corner_points[3][0].y, RGBColor::coral);
	DrawLine2D(corner_points[3][1].x,
			   corner_points[3][1].y,
			   corner_points[0][1].x,
			   corner_points[0][1].y, RGBColor::coral);
	DrawLine2D(corner_points[3][1].x,
			   corner_points[3][1].y,
			   corner_points[2][1].x,
			   corner_points[2][1].y, RGBColor::coral);
	for (unsigned i=0; i<4; i++)
	{
	  DrawLine2D(corner_points[i][0].x,
			     corner_points[i][0].y,
			     corner_points[(i<3?i+1:0)][0].x,
			     corner_points[(i<3?i+1:0)][0].y, RGBColor::red);		// red
	  
	}
	for (unsigned i=0; i<2; i++)
	{
	  DrawLine2D(corner_points[i][1].x,
			     corner_points[i][1].y,
			     corner_points[(i<2?i+1:0)][1].x,
			     corner_points[(i<2?i+1:0)][1].y, RGBColor::red);	
	}
	DrawLine2D(corner_points[0][0].x,
			   corner_points[0][0].y,
			   corner_points[0][1].x,
			   corner_points[0][1].y, RGBColor::red);	//red
	DrawLine2D(corner_points[1][0].x,
			   corner_points[1][0].y,
			   corner_points[1][1].x,
			   corner_points[1][1].y, RGBColor::red);
	DrawLine2D(corner_points[2][0].x,
			   corner_points[2][0].y,
			   corner_points[2][1].x,
			   corner_points[2][1].y, RGBColor::red);
  }

  if(detail == 1)
  {
	// Draw intersection points: 31 = 01-(10-20) = 21-(10-00) = 30-(10-11) 
	FillEllipse2D(intersection_points[0].x, intersection_points[0].y, 2, 2, 255, RGBColor::red);
	FillEllipse2D(intersection_points[1].x, intersection_points[1].y, 2, 2, 255, RGBColor::green);
	FillEllipse2D(intersection_points[2].x, intersection_points[2].y, 2, 2, 255, RGBColor::blue);

	char text[20];
	for (unsigned i=0; i<4; i++)
	  for (unsigned j=0; j<2; j++)
	  {
		snprintf(text, 20, "%u/%u", i, j);
		DrawText2D(text, corner_points[i][j].x, corner_points[i][j].y, RGBColor::blue);
	  }
  }
  
  if(detail == 2 && (closingLJct != UNDEF_ID || closingColl != UNDEF_ID))
  {
    unsigned r0 = Flaps(flap)->rects[0];
		unsigned r1 = Flaps(flap)->rects[1];

		char text[20];
		if (closingLJct != UNDEF_ID)
		{
			snprintf(text, 20, "%u", closingLJct);
			DrawText2D(text, LJunctions(closingLJct)->isct.x,
					LJunctions(closingLJct)->isct.y, RGBColor::blue);
		}
		else
		{	  
			snprintf(text, 20, "%u", closingColl);
			DrawText2D(text, Collinearities(closingColl)->vertex.x,
					Collinearities(closingColl)->vertex.y, RGBColor::blue);
		}
	
	
		// draw the 2 rects of the flap
		for(unsigned i = 0; i < 4; i++)
		{
				DrawLine2D(
			LJunctions(Rectangles(r0)->jcts[i])->isct.x,
			LJunctions(Rectangles(r0)->jcts[i])->isct.y,
			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.x,
			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.y,
						RGBColor::red);
				DrawLine2D(
			LJunctions(Rectangles(r1)->jcts[i])->isct.x,
			LJunctions(Rectangles(r1)->jcts[i])->isct.y,
			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.x,
			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.y,
						RGBColor::red);
		}
	
		// draw the L-Junction with closing lines
		Lines(jctLines[0])->Draw(1);
		Lines(jctLines[1])->Draw(1);
		
		for(unsigned i=0; i<cLines[0].Size(); i++)
			Lines(cLines[0][i])->Draw(1);
		for(unsigned i=0; i<cLines[1].Size(); i++)
			Lines(cLines[1][i])->Draw(1);
		
		// cLines
		for(unsigned i=0; i<cLines[LEFT].Size(); i++)	
		{
			DrawLine2D(Lines(cLines[LEFT][i])->point[0].x, Lines(cLines[LEFT][i])->point[0].y,
					Lines(cLines[LEFT][i])->point[1].x,	Lines(cLines[LEFT][i])->point[1].y, RGBColor::red);
		}
		for(unsigned i=0; i<cLines[RIGHT].Size(); i++)	
		{
			DrawLine2D( Lines(cLines[RIGHT][i])->point[0].x, Lines(cLines[RIGHT][i])->point[0].y,
					Lines(cLines[RIGHT][i])->point[1].x, Lines(cLines[RIGHT][i])->point[1].y,  RGBColor::red);
		}
  }	
  
  if(detail == 2 && oFlaps[0] != UNDEF_ID && oFlaps[1] != UNDEF_ID)
  {
		unsigned r0 = Flaps(flap)->rects[0];
		unsigned r1 = Flaps(oFlaps[0])->rects[0];
		unsigned r2 = Flaps(oFlaps[1])->rects[0];

		// draw the 3 rects of the flap
    for(unsigned i = 0; i < 4; i++)
		{
			DrawLine2D(
			LJunctions(Rectangles(r0)->jcts[i])->isct.x,
			LJunctions(Rectangles(r0)->jcts[i])->isct.y,
			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.x,
			LJunctions(Rectangles(r0)->jcts[(i<3?i+1:0)])->isct.y,
						RGBColor::red);
				DrawLine2D(
			LJunctions(Rectangles(r1)->jcts[i])->isct.x,
			LJunctions(Rectangles(r1)->jcts[i])->isct.y,
			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.x,
			LJunctions(Rectangles(r1)->jcts[(i<3?i+1:0)])->isct.y,
						RGBColor::red);
			DrawLine2D(
			LJunctions(Rectangles(r2)->jcts[i])->isct.x,
			LJunctions(Rectangles(r2)->jcts[i])->isct.y,
			LJunctions(Rectangles(r2)->jcts[(i<3?i+1:0)])->isct.x,
			LJunctions(Rectangles(r2)->jcts[(i<3?i+1:0)])->isct.y,
						RGBColor::red);
		}
  }	

  if (detail == 3)
  {
		DrawEllipse2D(center.x, center.y, radius, radius , 0., RGBColor::red);
  }
  
  if(detail > 3)
  {
		Flaps(flap)->Draw(detail-2);
	if (oFlaps[0] != UNDEF_ID && oFlaps[0] != UNDEF_ID)
	{
	  Flaps(oFlaps[0])->Draw(detail-2);
	  Flaps(oFlaps[1])->Draw(detail-2);
	}

/*	if (closingLines[0] != UNDEF_ID)
	{
	  Lines(closingLines[0])->Draw(detail-6);
	  Lines(closingLines[1])->Draw(detail-6);
	}
*/
  }
  
}

const char* Cube::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
	
  if (closingLJct != UNDEF_ID || closingColl != UNDEF_ID)
  {
    n += snprintf(info_text, info_size, 
	  "%s\nflap: %u\nL-Junction: %u\nCollinearity: %u\njctLines: %u - %u\ncLines: ",
      Gestalt::GetInfo(), flap, closingLJct, closingColl, jctLines[0], jctLines[1]);
	for(unsigned i=0; i<cLines[LEFT].Size(); i++)
  	  n += snprintf(info_text + n, info_size - n, "L(%u) ", cLines[LEFT][i]);
	for(unsigned i=0; i<cLines[RIGHT].Size(); i++)
	  n += snprintf(info_text + n, info_size - n, "R(%u) ", cLines[RIGHT][i]);
  }
  else	// CreateFromFlaps
  {
    n += snprintf(info_text, info_size, 
	  "%s\nflaps: %u - %u - %u\n",
      Gestalt::GetInfo(), flap, oFlaps[0], oFlaps[1]);
  }
  
	n += snprintf(info_text + n, info_size -n , 
	"\ni-p: %4.2f - %4.2f\ni-p1: %4.2f - %4.2f\ni-p2: %4.2f - %4.2f", intersection_points[0].x, intersection_points[0].y, intersection_points[1].x, intersection_points[1].y, intersection_points[2].x, intersection_points[2].y);

  
  return info_text;
}

bool Cube::IsAtPosition(int x, int y)
{
  return Flaps(flap)->IsAtPosition(x, y);
}

void Cube::CalculateSignificance()
{
	sig = Flaps(flap)->sig * 3.;
	sig = sig/(2.0*parallelity);
/*	if (oFlaps[0] != UNDEF_ID)
	{
	  sig += Flaps(oFlaps[0])->sig;
	  sig += Flaps(oFlaps[1])->sig;
	  sig = sig/3;
	}
*/
}

void Cube::CalculateSignificance2()
{
	sig = Flaps(flap)->sig + Flaps(oFlaps[0])->sig + Flaps(oFlaps[1])->sig;
	if(parallelity!=0) sig = sig/(2.0*parallelity);
/*	if (oFlaps[0] != UNDEF_ID)
	{
	  sig += Flaps(oFlaps[0])->sig;
	  sig += Flaps(oFlaps[1])->sig;
	  sig = sig/3;
	}
*/
}

}
