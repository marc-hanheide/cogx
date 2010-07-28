/**
 * @file EJunction.cc
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Class file of Gestalt Ellipse-Junction (EJunction).
 **/

#include "EJunction.hh"
#include "Ellipse.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt E-Junction.
 * @param vc Vision core
 * @param l Line of E-Junction
 * @param e Ellipse of E-Junction
 * @param lE Line End, which is farer away from E-junction
 */
EJunction::EJunction(VisionCore *vc, Line *l, Ellipse *e, unsigned lE, unsigned vtx) : Gestalt(vc, E_JUNCTION)
{
	line = l;
	ellipse = e;
  lineEnd = lE;
	vertex = vtx;
	
	// calculate intersection point:
	try
	{
		isct = LineIntersection(line->point[lineEnd], line->tang[lineEnd], ellipse->vertex[vertex], ellipse->dir, &gap[0], &gap[1]);
	}
	catch (Except &e)
	{
		printf("EJunction::EJunction: Lines do not intersect exception.\n");
	}

	// get lines which are connected with the line via a collinearity
	UpdateColLines(line, Other(lineEnd));
	
	// Add e-junction to ellipsen
	ellipse->ejcts[vertex].PushBack(this);
	
//   CalculateSignificance();																																// TODO Calculate significance
}

/**
 * @brief Update the collinearity lines. Get all following lines, which are
 * connected to line via a collinearity.
 * @param l Line for which we are looking for new collinearities.
 * @param lE Line end, where we are searching for collinearities.
 */
void EJunction::UpdateColLines(Line *l, unsigned lE)
{
	// get all col-lines 
	for(unsigned i=0; i<l->coll[lE].Size(); i++)
	{
		Line *newColLine = l->coll[lE][i]->OtherLine(l);
		unsigned newColLineEnd = Other(l->coll[lE][i]->near_point[l->coll[lE][i]->WhichLineIs(newColLine)]);
		if(colLines.Find(newColLine) == UNDEF_ID)
		{
			colLines.PushBack(newColLine);
			colLinesEnd.PushBack(newColLineEnd);
			UpdateColLines(newColLine, newColLineEnd);
		}
	}
}

void EJunction::Draw(int detail)
{
	if(detail == 0)
	{
		ellipse->Draw(detail);
		line->Draw(detail);
		DrawPoint2D(isct.x, isct.y, RGBColor::blue);
	}
  else if(detail == 1)
  {
		ellipse->Draw(detail);
		line->Draw(detail);
	}
  else if(detail == 2)
  {
		ellipse->Draw(detail-1);
		line->Draw(detail-1);
		for(unsigned i=0; i<colLines.Size(); i++)
			colLines[i]->Draw(detail-1);
	}
	else if(detail == 3)
  {
		Vector2 vtx = ellipse->vertex[vertex];
		Vector2 le = line->point[lineEnd];

		DrawLine2D(isct.x, isct.y, vtx.x, vtx.y, RGBColor::blue);
		DrawLine2D(isct.x, isct.y, le.x, le.y, RGBColor::blue);
  }
  else if(detail == 4)
  {
		ellipse->Draw(0);
		line->Draw(0);
		DrawVotes();
  }
  else if(detail >= 5)
  {
		ellipse->Draw(0);
		line->Draw(0);
		
		DrawPoint2D(isct.x, isct.y, RGBColor::blue);
		char id_str[20];
    snprintf(id_str, 20, "isct");
    DrawText2D(id_str, isct.x+5, isct.y, RGBColor::blue);
	}
}

/*
**																				TODO ARI: obsolete?
**																				Wenn draw-votes für ellipsen und lines 
*/
void EJunction::DrawVotes()
{
  VoteImage *vi = ((FormEJunctions*)(core->Principles(GestaltPrinciple::FORM_E_JUNCTIONS)))->vote_img;
	unsigned baseIndex = ((FormEJunctions*)(core->Principles(GestaltPrinciple::FORM_E_JUNCTIONS)))->baseIndex;
	unsigned baseOffset = ((FormEJunctions*)(core->Principles(GestaltPrinciple::FORM_E_JUNCTIONS)))->baseOffset;
	
  if(vi == 0)
    return;
  for(int x = 0; x < vi->width; x++)
	{
    for(int y = 0; y < vi->height; y++)
    {
      VoteImage::Elem *el = vi->Pixel(x, y);
      while(el != 0)
      {
        if((el->id/baseIndex == ellipse->ID()+baseOffset) ||
					 (el->id/baseIndex == line->ID()))
        {
          unsigned vtype = el->id%8;
          switch(vtype)
          {
            case VOTE_TS:
              DrawPoint2D(x, y, RGBColor::magenta);
              break;
            case VOTE_TE:
              DrawPoint2D(x, y, RGBColor::cyan);
              break;
						case VOTE_EOTL:
							DrawPoint2D(x, y, RGBColor::magenta);
							break;
						case VOTE_EOTR:
							DrawPoint2D(x, y, RGBColor::magenta);
							break;
						case VOTE_EITL:
							DrawPoint2D(x, y, RGBColor::cyan);
							break;
						case VOTE_EITR:
							DrawPoint2D(x, y, RGBColor::cyan);
							break;
            default: // line itself
							DrawPoint2D(x, y, RGBColor::white);
              break;
          }
        }
        el = el->next;
      }
    }
	}
}

const char* EJunction::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, "%s", Gestalt::GetInfo());

  n += snprintf(info_text + n, info_size - n, 
	"  ellipse: %u @ vertex: %u\n  line: %u @ end: %u", ellipse->ID(), vertex, line->ID(), lineEnd);
	
	n += snprintf(info_text + n, info_size - n, "\n  colLines: ");
	for (unsigned i=0; i<colLines.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", colLines[i]->ID());

	n += snprintf(info_text + n, info_size - n, "\n  colLinesEnd: ");
	for (unsigned i=0; i<colLines.Size(); i++)
		n += snprintf(info_text + n, info_size - n, "%u ", colLinesEnd[i]);
  
	return info_text;
}

bool EJunction::IsAtPosition(int x, int y)
{
  if (line->IsAtPosition(x, y) || ellipse->IsAtPosition(x, y))
		return true;
  return false;
}

void EJunction::CalculateSignificance()
{
}

/**
 * @brief Returns true, if line is part of the e-junction.
 * @param l Line
 */
bool EJunction::IsLine(Line *l)
{
	if(line->ID() == l->ID()) return true;
	return false;
}

/**
 * @brief Returns true, if line is within the col-lines or is the e-junction line.
 * @param l Line to check
 * @return Returns true, if line is part of the ellipse junction.
 */
bool EJunction::IsColLine(Line *l)
{
	if(colLines.Contains(l) || l == line) return true;
	return false;
}

}





