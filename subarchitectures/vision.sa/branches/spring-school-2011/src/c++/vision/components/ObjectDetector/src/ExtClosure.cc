/**
 * $Id: ExtClosure.cc,v 1.2 2007/10/31 13:47:03 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "Closure.hh"
#include "ExtClosure.hh"

namespace Z
{

ExtClosure::ExtClosure(unsigned c0, unsigned c1, Array<unsigned> l, 
	Array<unsigned> shL) : Gestalt(EXTCLOSURE)
{
  clos[0] = c0;
  clos[1] = c1;
  sharedLines = shL;
  lines = l;

  CalculateSignificance();
}
/*
Vector2 Closure::GetVertex(unsigned i)
{
  if(jcts[i] != UNDEF_ID && colls[i] != UNDEF_ID)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(jcts[i] != UNDEF_ID)
    return LJunctions(jcts[i])->isct;
  else if(colls[i] != UNDEF_ID)
    return Collinearities(colls[i])->vertex;
  else
    throw Except(__HERE__, "need one of L-jct or collinearity");
}
*/

void ExtClosure::Draw(int detail)
{
  if (detail == 0)
  {
//	Closures(clos[0])->Draw(detail);
//    Closures(clos[1])->Draw(detail);
	for(unsigned i=0; i<lines.Size(); i++)
    {
	  Lines(lines[i])->Draw(detail);
    }
	  
/*    for(unsigned i = 0; i < lines.Size(); i++)
    {
      unsigned j = (i < lines.Size()-1 ? i+1 : 0);
      Vector2 p, q, r, s;
      if(senses[i] == SAME)
      {
        p = Lines(lines[i])->point[START];
        q = Lines(lines[i])->point[END];
      }
      else
      {
        p = Lines(lines[i])->point[END];
        q = Lines(lines[i])->point[START];
      }
      DrawLine2D(p.x, p.y, q.x, q.y, RGBColor::blue);
      r = GetVertex(j);
      DrawLine2D(q.x, q.y, r.x, r.y, RGBColor::blue);
      if(senses[j] == SAME)
        s = Lines(lines[j])->point[START];
      else
        s = Lines(lines[j])->point[END];
      DrawLine2D(r.x, r.y, s.x, s.y, RGBColor::blue);
    }
*/   
	  
  }
  else if (detail == 1)
  {
	Closures(clos[0])->Draw(detail-1);
    Closures(clos[1])->Draw(detail-1);
    for(unsigned i=0; i<sharedLines.Size(); i++)
    {
	  Lines(sharedLines[i])->Draw(detail);
    }
  }
  else if (detail > 1)
  {
	Closures(clos[0])->Draw(detail-1);
    Closures(clos[1])->Draw(detail-1);
  }
}

const char* ExtClosure::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
	
  n += snprintf(info_text + n, info_size - n, "%sclos: %u - %u\n", 
	Gestalt::GetInfo(), clos[0], clos[1]);
	
  n += snprintf(info_text + n, info_size - n, "%u shared Lines:",
      sharedLines.Size());
  for(unsigned i = 0; i < sharedLines.Size(); i++)
    if(sharedLines[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " %d", sharedLines[i]);

  return info_text;
}

bool ExtClosure::IsAtPosition(int x, int y)
{
  return Closures(clos[0])->IsAtPosition(x, y);
}

void ExtClosure::CalculateSignificance()
{
	//sig =
}
}
