/**
 * @file Rectangle.cc
 * @author Richtsfeld Andreas
 * @date Februar 2008
 * @version 0.1
 * @brief This class stores rectangles
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

namespace Z
{

Rectangle::Rectangle(unsigned c, unsigned js[4], double par, unsigned ljct)
  : Gestalt(RECTANGLE)
{
	width = VisionCore::IW();
	height = VisionCore::IH();
  data = new unsigned[width*height];
	pixelmass = 0;

	/// init data
	for (unsigned i=0; i<(width*height); i++)
		data[i] = UNDEF_ID;

  clos = c;
  for(unsigned i = 0; i < 4; i++)
    jcts[i] = js[i];
  parallelity = par;
  nrOfLJcts = ljct;

	centerPoint.x = 0;
	centerPoint.y = 0;
	for(unsigned i=0; i<4; i++)
	{
		centerPoint.x += LJunctions(jcts[i])->isct.x;
		centerPoint.y += LJunctions(jcts[i])->isct.y;
	}
	centerPoint.x = centerPoint.x/4.;
	centerPoint.y = centerPoint.y/4.;

	// get direction (PI) of the two line-pairs
	unsigned j;
	Vector2 v[4];
	//double phi[4];
	//double norm[4];
	for(unsigned i=0; i<4; i++)
	{
		if((i+1) > 3) j=0;
		else j=i+1;

		v[i] = LJunctions(jcts[i])->isct - LJunctions(jcts[j])->isct;
//		phi[i] = PolarAngle(v[i]);

//		norm[i] = v[i].Norm();
	}

//   double diff[2];
//   diff[0] = fabs(fabs(phi[0]-phi[2])-M_PI);
//   diff[1] = fabs(fabs(phi[1]-phi[3])-M_PI);
/*

	double phi0 = (v[0].x*v[2].x + v[0].y*v[2].y)/(norm[0]*norm[2]);
	double phi1 = (v[1].x*v[3].x + v[1].y*v[3].y)/(norm[1]*norm[3]);

	phi0 = acos(phi0);
	phi1 = acos(phi1);

	printf("Rect: %u: dir[0-4]: %4.2f - %4.2f - %4.2f - %4.2f\n", id, phi[0], phi[1], phi[2], phi[3]);
	printf("phi0-1: %4.2f - %4.2f\n", phi0, phi1);

	if(phi0 > M_PI/2.) phi0 = M_PI - phi0;
	if(phi1 > M_PI/2.) phi1 = M_PI - phi1;

//	direction[0] = (dir[0] + dir[2])/2.;
//	direction[1] = (dir[1] + dir[3])/2.;

	printf("phi0-1: %4.2f - %4.2f\n\n", phi0, phi1);
  																				// TODO ARI: Eintragen der rects bei den Linien
  																				// schwierig, weil nur ljcts bekannt!!!
  																				// TODO Lösung => Unterlagertes Closure ist mit allen Linien bekannt =>
																					// Damit kann man aber über das Closere alle anderen Linien bestimmen =>
																					// eintragen ist daher nicht zwingend notwendig!
*/

	
	direction[0] = Normalise(v[0]-v[2]);
	direction[1] = Normalise(v[1]-v[3]);

	phi[0] = ScaleAngle_0_pi(PolarAngle(direction[0]));
	phi[1] = ScaleAngle_0_pi(PolarAngle(direction[1]));

	// calculate maximum distance from centerPoint to one of the corner of the rectangle
	radius = (LJunctions(jcts[0])->isct - centerPoint).Norm();
	radius = max(radius, (LJunctions(jcts[1])->isct - centerPoint).Norm());
	radius = max(radius, (LJunctions(jcts[2])->isct - centerPoint).Norm());
	radius = max(radius, (LJunctions(jcts[3])->isct - centerPoint).Norm());

	// calculate inner radius from centerPoint to one of the mid points of the rectangle lines
	innerRadius = (((LJunctions(jcts[0])->isct + (LJunctions(jcts[1]))->isct)/2.) - centerPoint).Norm();
	innerRadius = min(innerRadius,((((LJunctions(jcts[1])->isct + (LJunctions(jcts[2])->isct))/2.) - centerPoint).Norm()));
	innerRadius = min(innerRadius,((((LJunctions(jcts[2])->isct + (LJunctions(jcts[3])->isct))/2.) - centerPoint).Norm()));
	innerRadius = min(innerRadius,((((LJunctions(jcts[3])->isct + (LJunctions(jcts[0])->isct))/2.) - centerPoint).Norm()));

	pixelsupport = CalculateSupport();

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
      DrawLine2D(LJunctions(jcts[i])->isct.x, LJunctions(jcts[i])->isct.y,
          LJunctions(jcts[(i<3?i+1:0)])->isct.x,
          LJunctions(jcts[(i<3?i+1:0)])->isct.y,
          RGBColor::yellow);
  }
	if(detail == 1)
	{
		FillEllipse2D(centerPoint.x, centerPoint.y, 2, 2, 0, RGBColor::white);
		DrawEllipse2D(centerPoint.x, centerPoint.y, radius, radius, 0, RGBColor::white);
		DrawEllipse2D(centerPoint.x, centerPoint.y, innerRadius, innerRadius, 0, RGBColor::white);
	}
  if(detail >= 2)
    Closures(clos)->Draw(detail - 2);
}

const char* Rectangle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size, "%sclosure: %u\njunctions: %u %u %u %u\n",
      Gestalt::GetInfo(), clos, jcts[0], jcts[1], jcts[2], jcts[3]);
  return info_text;
}

/**
 *	@brief Returns true, if the rectangle center point is inside another rectangle radius.
 *	@param rectangle Index of rectangle to compare.
 */
bool Rectangle::IsInside(unsigned rectangle)
{
	if((Rectangles(rectangle)->centerPoint - centerPoint).Length() < innerRadius) return true;
	else return false;
}

bool Rectangle::IsAtPosition(int x, int y)
{
  return Closures(clos)->IsAtPosition(x, y);
}

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
***	Calculate pixel-support: Get the line pixels from the constructed rectangle with bresenham
***	line drawing algorithm and compare them with the underlying line pixels.
**/
double Rectangle::CalculateSupport()
{
	double support = 0;

	GetRectPixels();				// write all pixels from new drawn rectangle into data[]

	// Catch Pixel from rect->clos->line=>idx[start/end] from
	// Segment->edgels(idx[Start/End]) for all lines from the closure
	Array<unsigned> lines = Closures(clos)->lines;
	for(unsigned i=0; i<lines.Size(); i++)
	{
		for(unsigned j=Lines(lines[i])->idx[0]; j<Lines(lines[i])->idx[1]; j++)
		{
			Vector2 p = Segments(Lines(lines[i])->seg)->edgels[j].p;		// pixel from segment

			// count the supported pixels from the rectangle
			if (CountSupport((int)p.x, (int)p.y, 100)) support += 1;
		}
	}
//printf("support - pixelmass: %4.2f, %u\n", support, pixelmass);
	return support*10/((double)pixelmass);	
}

void Rectangle::GetRectPixels()
{
	// Get all corner points of rect
	Vector2 isct[4];
	for (unsigned i=0; i<4; i++)
		isct[i] = LJunctions(jcts[i])->isct;

	// Calculate all line pixels
	int j = 0;
	for (unsigned i=0; i<4; i++)
	{
		if (i==3) j=0;
		else j=i+1;
		GetLinePixels(int(isct[i].x), int(isct[i].y), int(isct[j].x), int(isct[j].y), i);
	}
}

void Rectangle::GetLinePixels(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, err, incr, x, y;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
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


void Rectangle::SetPixel(int x, int y, unsigned id)
{
  data[y*width + x] = id;
	pixelmass ++;
}

bool Rectangle::CountSupport(int x, int y, unsigned id)
{
	if (data[y*width + x] != UNDEF_ID) return true;
	return false;

  //return data[y*width + x] != UNDEF_ID && data[y*width + x] != id;
}

}
