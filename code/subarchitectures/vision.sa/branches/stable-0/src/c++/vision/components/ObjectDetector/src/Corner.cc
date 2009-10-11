/**
 * $Id: Corner.cc,v 1.2 2007/11/26 13:47:03 mxz Exp mxz $
 */

#include "Array.hh"
#include "Draw.hh"
#include "Corner.hh"
#include "Line.hh"
#include "LJunction.hh"

namespace Z
{

Corner::Corner(Array<unsigned> l, Array<unsigned> j, Array<unsigned> np)
	: Gestalt(CORNER)
{
  lines = l;
  ljcts = j;
  near_point = np;
}

/*
**	TODO ARI: Umbennen (ist eher ein Update oder so)
**	Add new L-Junction to existing corner
*/
void Corner::AddLJunctions(Array<unsigned> &lj, Array<unsigned> &li,
	Array<unsigned> &np)
{
  for (unsigned i=0; i<lj.Size(); i++)
  {
	if (!ljcts.Contains(lj[i]))
	  ljcts.PushBack(lj[i]);
  }
  
  for (unsigned i=0; i<li.Size(); i++)
  {
	if(!lines.Contains(li[i]))
	{
	  lines.PushBack(li[i]);
	  near_point.PushBack(np[i]);
	  Lines(li[i])->corners.PushBack(id);
	}
  }
}


/*
**	CalculateIntersection
*/
Vector2 Corner::CalculateIntersection(unsigned lj0, unsigned lj1, double &gap)
{
  // get the intersection points of both l-junctions
  Vector2 isct0 = LJunctions(lj0)->isct;
  Vector2 isct1 = LJunctions(lj1)->isct;
	
  // calculate the gap betweent the 2 points
  double sqrX = Sqr(isct0.x-isct1.x);
  double sqrY = Sqr(isct0.y-isct1.y);
  gap = sqrt(sqrX + sqrY);
	
  // calculate the mean corner intersection
  Vector2 isct;
  isct.x = (isct0.x + isct1.x)/2;
  isct.y = (isct0.y + isct1.y)/2;
	
  return isct;
}

/*
**	Add a third L-Jct to a existing Corner
*/
void Corner::Recalc(unsigned lj)
{
//  ljct[2] = lj;
//  sig += 100;
}

void Corner::Draw(int detail)
{
  for (unsigned i=0; i<ljcts.Size(); i++)
	LJunctions(ljcts[i])->Draw(detail+1);
}

const char* Corner::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
  n += snprintf(info_text, info_size, 
		"%s\nL-Jcts: ", Gestalt::GetInfo());
  for (unsigned i=0; i<ljcts.Size(); i++)
	n += snprintf(info_text + n, info_size - n,
  		"%i ", ljcts[i]);
	
  n += snprintf(info_text + n, info_size - n, "\nLines: ");
  
  for (unsigned i=0; i<lines.Size(); i++)
	n += snprintf(info_text + n, info_size - n,
  		"%i ", lines[i]);

  n += snprintf(info_text + n, info_size - n, "\nnear Point: ");
  
  for (unsigned i=0; i<near_point.Size(); i++)
	n += snprintf(info_text + n, info_size - n,
  		"%i ", near_point[i]);

   return info_text;
}

bool Corner::IsAtPosition(int x, int y)
{
  for (unsigned i=0; i<lines.Size(); i++)
  {
	if (Lines(lines[i])->IsAtPosition(x, y))
	  return true;
  }
  return false;
}

void Corner::CalculateSignificance(double gap)
{
//  double cGap = gap/50.;
//  sig = 1./(cGap + 0.005);
  sig += meanLength/(gap + 0.2);
}

}
