/**
 * @file Rectangle.cc
 * @author Richtsfeld Andreas
 * @date Februar 2008, 2010
 * @version 0.1
 * @brief Gestalt class of rectangles (quadrilaterals).
 **/

#include "Rectangle.hh"
#include "VisionCore.hh"
#include "Draw.hh"
#include "Edgel.hh"
#include "Segment.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Closure.hh"
#include "Vector2.hh"
#include <cstdio>

namespace Z
{

/**
 * Constructor of Gestalt class rectangle.
 * @param vc Vision core
 * @param c Closure
 * @param is[] The four intersection points in clockwise order.
 * @param par Parallelity for significance value
 */
Rectangle::Rectangle(VisionCore *vc, Closure *c, Vector2 is[4], double par) : Gestalt(vc, RECTANGLE)
{
  closure = c;
  parallelity = par;

  for(unsigned i = 0; i < 4; i++)
    isct[i] = is[i];

  Init();
}


/**
 * @brief Initialise the rectangle and calculate property values.
 */
void Rectangle::Init()
{
  // Calculate pixelsupport
  data = new unsigned[core->IW()*core->IH()];
  pixelmass = 0;
  for (unsigned i=0; i<(core->IW()*core->IH()); i++)
	  data[i] = UNDEF_ID;
  pixelsupport = CalculateSupport();

  // get direction (PI) and angle of the two line-pairs
  unsigned j;
  Vector2 isctLine[4];
  for(unsigned i=0; i<4; i++)
  {
	  if((i+1) > 3) j=0;
	  else j=i+1;

	  isctLine[i] = isct[i] - isct[j];
  }

  Vector2 v[2];
  v[0] = isctLine[0]-isctLine[2];
  v[1] = isctLine[1]-isctLine[3];

  direction[0] = Normalise(v[0]-v[2]);
  direction[1] = Normalise(v[1]-v[3]);

  phi[0] = ScaleAngle_0_pi(PolarAngle(direction[0]));
  phi[1] = ScaleAngle_0_pi(PolarAngle(direction[1]));

  // calculate center point of rectangle
  centerPoint.x = 0;
  centerPoint.y = 0;
  for(unsigned i=0; i<4; i++)
  {
	  centerPoint.x += isct[i].x;
	  centerPoint.y += isct[i].y;
  }
  centerPoint.x = centerPoint.x/4.;
  centerPoint.y = centerPoint.y/4.;

  // calculate maximum distance from centerPoint to one of the corner of the rectangle
  radius = (isct[0] - centerPoint).Norm();
  radius = max(radius, (isct[1] - centerPoint).Norm());
  radius = max(radius, (isct[2] - centerPoint).Norm());
  radius = max(radius, (isct[3] - centerPoint).Norm());

  // calculate inner radius from centerPoint to one of the mid points of the rectangle lines
  innerRadius = ((isct[0] + isct[1]/2.) - centerPoint).Norm();
  innerRadius = min(innerRadius,((((isct[1] + isct[2])/2.) - centerPoint).Norm()));
  innerRadius = min(innerRadius,((((isct[2] + isct[3])/2.) - centerPoint).Norm()));
  innerRadius = min(innerRadius,((((isct[3] + isct[0])/2.) - centerPoint).Norm()));

  CalculateSignificance();
}


/**
 *	@brief Draw rectangles.
 *	@param detail Degree of details.
 */
void Rectangle::Draw(int detail)
{
  if(detail == 0)
  {
    for(unsigned i = 0; i < 4; i++)
      DrawLine2D(isct[i].x, isct[i].y, isct[(i<3?i+1:0)].x, isct[(i<3?i+1:0)].y);
  }
	if(detail == 1)
	{
		FillEllipse2D(centerPoint.x, centerPoint.y, 2, 2, 0, RGBColor::white);
		DrawEllipse2D(centerPoint.x, centerPoint.y, radius, radius, 0, RGBColor::white);
		DrawEllipse2D(centerPoint.x, centerPoint.y, innerRadius, innerRadius, 0, RGBColor::white);
	}
  if(detail >= 2)
    closure->Draw(detail - 2);
}

/**
 * @brief Get info about the feature.
 * @return Returns string with all informations about this feature.
 */
const char* Rectangle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%s"
                                 "  closure: %u\n",
//                                  "  junctions: %u %u %u %u\n",
      Gestalt::GetInfo(), closure->ID()/*, ljcts[0]->ID(), ljcts[1]->ID(), ljcts[2]->ID(), ljcts[3]->ID()*/);
  return info_text;
}

/**
 *	@brief Returns true, if the rectangle center point is inside another rectangle radius.
 *	@param rectangle Index of rectangle to compare.
 */
bool Rectangle::IsInside(unsigned rectangle)
{
	if((Rectangles(core, rectangle)->centerPoint - centerPoint).Length() < innerRadius) return true;
	else return false;
}

/**
 * @brief Shows if feature is at this position in the image
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true, if feature is at this position.
 */
bool Rectangle::IsAtPosition(int x, int y)
{
  return closure->IsAtPosition(x, y);
}

/**
 * @brief Calculate significance value of the feature.
 */
void Rectangle::CalculateSignificance()
{
  //sig = -log(fmax(1., SumGaps())/fmax(1., Area()));
  //sig = SumGaps()/Circumference();

  // TODO ARI	
  //sig = parallelity * exp(4.-nrOfLJcts);

	sig = (pixelsupport * parallelity);
//printf("Rect: %u: pixelsupport: %6.3f - parallelity: %4.2f\n", id, pixelsupport, parallelity);
}


/**
 * @brief Calculate pixel-support: Get the line pixels from the constructed rectangle with bresenham \n
 * line drawing algorithm and compare them with the underlying line pixels.
 */
double Rectangle::CalculateSupport()
{
	double support = 0;

	GetRectPixels();				// write all pixels from new drawn rectangle into data[]

	// Catch Pixel from rect->clos->line=>idx[start/end] from
	// Segment->edgels(idx[Start/End]) for all lines from the closure
	Array<unsigned> lines;
	for(unsigned i=0; i<closure->lines.Size(); i++)
		lines.PushBack (closure->lines[i]->ID());

	for(unsigned i=0; i<lines.Size(); i++)
	{
		for(unsigned j=Lines(core, lines[i])->idx[0]; j<Lines(core, lines[i])->idx[1]; j++)
		{
			Vector2 p = Segments(core, Lines(core, lines[i])->seg->ID())->edgels[j].p;		// pixel from segment
			if (CountSupport((int)p.x, (int)p.y, 100)) support += 1;
		}
	}

//printf("support - pixelmass: %4.2f, %u\n", support, pixelmass);
	return support*10/((double)pixelmass);	
}

/**																																								/// TODO Sollte innerhalb der isct-points zeichnen!
 *  @brief GetRectPixels:
 *  Draws the rectangle between the estimated four L-Junctions.
 */
void Rectangle::GetRectPixels()
{
	// Get all corner points of rect																								/// TODO dieser Teil weg!!!
// 	Vector2 isct[4];
// 	for (unsigned i=0; i<4; i++)
// 		isct[i] = LJunctions(core, ljcts[i]->ID())->isct;

	// Calculate all line pixels
	int j = 0;
	for (unsigned i=0; i<4; i++)
	{
		if (i==3) j=0;
		else j=i+1;
		GetLinePixels(int(isct[i].x), int(isct[i].y), int(isct[j].x), int(isct[j].y), i);
	}
}

/**
 * @brief LineDrawing Algorithm from Bresnham. Draw a line from x1/y1 to x2/y2.
 * @param x1 x-Coordinate
 * @param y1 y-Coordinate
 * @param x2 x-Coordinate
 * @param y2 y-Coordinate
 * @param id ID of the line to draw.
 */
void Rectangle::GetLinePixels(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, err, incr, x, y;

  if(!ClipLine(core->IW()-1, core->IH()-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  // octants 1,4,5,8
  if(abs(dx) > abs(dy))
  {
    if(dx < 0)
    {
      swap(x2, x1);
      swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dy >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dy = -dy;
    }
    // first octant bresenham
    err = -dx/2;
    x = x1;
    y = y1;
    SetPixel(x, y, id);
    while(x < x2-1) // <= x2  version A
    {
      //SetPixel(x, y, id);  version A
      err += dy;
      if(err >= 0)
      {
        y += incr;
        err -= dx;
      }
      x++;
      SetPixel(x, y, id);
    }
  }
  // octants 2,3,6,7
  else // abs(dx) <= abs(dy)
  {
    if(dy < 0)
    {
      swap(x2, x1);
      swap(y2, y1);
      dx = -dx;
      dy = -dy;
    }
    if(dx >= 0)
      incr = 1;
    else
    {
      incr = -1;
      dx = -dx;
    }
    // second octant bresenham
    err = -dy/2;
    x = x1;
    y = y1;
    SetPixel(x, y, id);
    while(y < y2-1) // <= y2  version A
    {
      //SetPixel(x, y, id);  version A
      err += dx;
      if(err >= 0)
      {
        x += incr;
        err -= dy;
      }
      y++;
      SetPixel(x, y, id);
    }
  }
}

/**
 * @brief Fills the data array with line pixels.
 * @param x x-coordinate of the pixel
 * @param y y-coordinate of the pixel
 * @param id Line-ID ???
 */
void Rectangle::SetPixel(int x, int y, unsigned id)
{
  data[y*core->IW() + x] = id;
	pixelmass ++;
}

/**
 * @brief Returns true, if there is a line pixel at x,y in the image.
 * @param x x-coordinate of the pixel
 * @param y y-coordinate of the pixel
 * @param id ???
 * @return True, if pixel is line-pixel
 */
bool Rectangle::CountSupport(int x, int y, unsigned id)
{
	if (data[y*core->IW() + x] != UNDEF_ID) return true;
	return false;
}

}
