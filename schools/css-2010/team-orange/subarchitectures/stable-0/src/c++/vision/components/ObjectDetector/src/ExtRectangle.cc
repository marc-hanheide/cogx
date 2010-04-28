/**
 * $Id: Flap.cc,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
 */


#include "ExtRectangle.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Rectangle.hh"
#include "Line.hh"

namespace Z
{

ExtRectangle::ExtRectangle(unsigned rec, Array<unsigned> jcts,
		Array<unsigned> lines, Array<unsigned> colls)
  : Gestalt(EXTRECTANGLE)
{
  rect = rec;
  extLines = lines;
  extJcts = jcts;
  extColls = colls;

  CalculateSignificance();
}

void ExtRectangle::Draw(int detail)
{
  Rectangles(rect)->Draw(detail);

  for (unsigned i=0; i<extLines.Size(); i++)
		Lines(extLines[i])->Draw(detail-2);
}

const char* ExtRectangle::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
//  snprintf(info_text, info_size, "%srect: %u",
//      Gestalt::GetInfo(), rect);
  int n = 0;
	
  n += snprintf(info_text + n, info_size - n, "%srect: %u\n", 
	Gestalt::GetInfo(), rect);
	
  n += snprintf(info_text + n, info_size - n, "%u extLines:",
      extLines.Size());
  for(unsigned i = 0; i < extLines.Size(); i++)
    if(extLines[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " %d", extLines[i]);

  n += snprintf(info_text + n, info_size - n, "\n%u extJcts:",
      extJcts.Size());
  for(unsigned i = 0; i < extJcts.Size(); i++)
    if(extJcts[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " L(%d)", extJcts[i]);

  n += snprintf(info_text + n, info_size - n, "\n%u extColls:",
      extColls.Size());
  for(unsigned i = 0; i < extColls.Size(); i++)
    if(extColls[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " C(%d)", extColls[i]);

//    else if(colls[i] != UNDEF_ID)
//      n += snprintf(info_text + n, info_size - n, " C(%d)", ExtColls[i]);
//  n += snprintf(info_text + n, info_size - n, "\n");

	
  return info_text;
}

bool ExtRectangle::IsAtPosition(int x, int y)
{
  return Rectangles(rect)->IsAtPosition(x, y);
}

void ExtRectangle::CalculateSignificance()
{
	//sig =
}
}
