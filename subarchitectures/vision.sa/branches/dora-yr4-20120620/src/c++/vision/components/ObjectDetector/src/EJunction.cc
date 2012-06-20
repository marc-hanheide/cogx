/**
 * $Id: EJunction.cc,v 1.0 2007/12/19 13:47:03 mxz Exp mxz $
 */

#include "EJunction.hh"
#include "Draw.hh"
#include "Line.hh"
#include "Ellipse.hh"
// #include "FormEllipseJunctions.hh"
#include "VoteImage.hh"

namespace Z
{

EJunction::EJunction(unsigned l, unsigned e, unsigned end, unsigned ver,
	Vector2 inter, double *g) : Gestalt(E_JUNCTION)
{
  line = l;
  ellipse = e;
  lineEnd = end;
  vertex = ver;
  isct = inter;
	
  gap[0] = g[0];
  gap[1] = g[1];
	
  Lines(line)->AddEJunction(lineEnd, id);
  Ellipses(ellipse)->AddEJunction(id);
	
  CalculateSignificance();
}

void EJunction::Draw(int detail)
{
  if(detail == 0)
  {
	Vector2 vtx = Ellipses(ellipse)->vertex[vertex];
	Vector2 le = Lines(line)->point[lineEnd];

	DrawLine2D(isct.x, isct.y, vtx.x, vtx.y, RGBColor::blue);
	DrawLine2D(isct.x, isct.y, le.x, le.y, RGBColor::blue);
  }
  
  if(detail < 2)
  {
	Lines(line)->Draw(detail);
	Ellipses(ellipse)->Draw(detail);
  }
  else if(detail >= 2)
  {
	Lines(line)->Draw(detail+2);
	Ellipses(ellipse)->Draw(detail);
  }
}

/*
**																				TODO ARI: Notwendig?
*/
void EJunction::DrawVotes()
{
/*  VoteImageEllipse *vi = FormEllipseJunctions::vote;
  if(vi == 0)
    return;
  for(int x = 0; x < vi->width; x++)
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        if(el->id/8 == id)
        {
          unsigned vtype = el->id%8;
          switch(vtype)
          {
            case VOTE_TS:
            case VOTE_NLS:
            case VOTE_NRS:
              DrawPoint2D(x, y, RGBColor::magenta);
              break;
            case VOTE_TE:
            case VOTE_NLE:
            case VOTE_NRE:
              DrawPoint2D(x, y, RGBColor::cyan);
              break;
            default:
              DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
*/
}

const char* EJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
	"%s \n", Gestalt::GetInfo());

  n += snprintf(info_text + n, info_size - n, 
	"line: %i\nellipse: %i\n", line, ellipse);
	
	return info_text;
}

bool EJunction::IsAtPosition(int x, int y)
{
  if (Lines(line)->IsAtPosition(x, y) ||
	  Ellipses(ellipse)->IsAtPosition(x, y))
	return true;
  return false;
}

void EJunction::CalculateSignificance()
{
}

}
