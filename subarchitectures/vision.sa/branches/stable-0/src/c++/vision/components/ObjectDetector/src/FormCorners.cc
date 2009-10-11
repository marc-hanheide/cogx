/**
 * $Id: FormCorners.cc,v 1.6 2007/11/26 13:47:03 mxz Exp mxz $
 */

#include "Math.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Corner.hh"
#include "FormCorners.hh"

namespace Z
{

static int CmpCorners(const void *a, const void *b)
{
  if( Corners(*(unsigned*)a)->sig > Corners(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

bool FormCorners::NeedsOperate()
{
  if (cand.Size()==0)
	return false;
  return true;
}

/*
**	TODO ARI:
**	get new candidates for creating corners
*/
void FormCorners::InformNewCandidate(unsigned l, unsigned side)
{
  if (!cand.Contains(l))
  {
	cand.PushBack(l);
	candSide.PushBack(side);
  }  	
}

void FormCorners::Rank()
{
  // TODO ARI: 
  RankGestalts(Gestalt::CORNER, CmpCorners);
}

FormCorners::FormCorners(Config *cfg)
: GestaltPrinciple(cfg)
{
}

/*
*	receive new Form
*/
void FormCorners::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      //Create(idx-1);			// Create from the foregoing L-Junction	
      break;
    default:
      break;
  }
  Rank();
  StopRunTime();
}

void FormCorners::Operate(bool incremental)
{
  StartRunTime();
  Create();
  StopRunTime();
}


/*
**	TODO ARI: Fertigstellen und bereinigen
**
*/
void FormCorners::Create()
{
  for (unsigned i=0; i<cand.Size(); i++)
  {
	unsigned cSide = candSide[i];
	  
	// number of junctions at START and END of line-candidate
	unsigned sJcts = Lines(cand[i])->l_jct[0][0].Size() + 
	  Lines(cand[i])->l_jct[0][1].Size();
	unsigned eJcts = Lines(cand[i])->l_jct[1][0].Size() +
	  Lines(cand[i])->l_jct[1][1].Size();
	  
	if (cSide == START)
	{
	  // get all L-Junctions
	  Array<unsigned> jcts;
	  for (unsigned j=0; j<Lines(cand[i])->l_jct[0][0].Size(); j++)
		jcts.PushBack(Lines(cand[i])->l_jct[0][0][j]); 
	  for (unsigned j=0; j<Lines(cand[i])->l_jct[0][1].Size(); j++)
		jcts.PushBack(Lines(cand[i])->l_jct[0][1][j]); 
	
	  // get the lines and near_points of the junctions
	  Array<unsigned> lines;
	  Array<unsigned> near_point;
	  for (unsigned j=0; j<sJcts; j++)
	  {
		if (!lines.Contains(LJunctions(jcts[j])->line[0]))
		{
		  lines.PushBack(LJunctions(jcts[j])->line[0]);
		  near_point.PushBack(LJunctions(jcts[j])->near_point[0]);
		}		
		if (!lines.Contains(LJunctions(jcts[j])->line[1]))
		{
		  lines.PushBack(LJunctions(jcts[j])->line[1]);
		  near_point.PushBack(LJunctions(jcts[j])->near_point[1]);
		}		
	  }
	  
	  unsigned corner = CornerExists(lines);
	  if(corner == UNDEF_ID)
		NewCorner(lines, jcts, near_point);
	  else
		Corners(corner)->AddLJunctions(jcts, lines, near_point);
	}
	
	if (cSide == END)
	{
	  // get all L-Junctions
	  Array<unsigned> jcts;
	  for (unsigned j=0; j<Lines(cand[i])->l_jct[1][0].Size(); j++)
		jcts.PushBack(Lines(cand[i])->l_jct[1][0][j]); 
	  for (unsigned j=0; j<Lines(cand[i])->l_jct[1][1].Size(); j++)
		jcts.PushBack(Lines(cand[i])->l_jct[1][1][j]); 

	  // get the lines of the junctions
	  Array<unsigned> lines;
	  Array<unsigned> near_point;
	  for (unsigned j=0; j<eJcts; j++)
	  {
		if (!lines.Contains(LJunctions(jcts[j])->line[0]))
		{
		  lines.PushBack(LJunctions(jcts[j])->line[0]);
		  near_point.PushBack(LJunctions(jcts[j])->near_point[0]);
		}		
		if (!lines.Contains(LJunctions(jcts[j])->line[1]))
		{
		  lines.PushBack(LJunctions(jcts[j])->line[1]);
		  near_point.PushBack(LJunctions(jcts[j])->near_point[1]);
		}		
	  }
	  
	  unsigned corner = CornerExists(lines);
	  if(corner == UNDEF_ID)
		NewCorner(lines, jcts, near_point);
	  else
		Corners(corner)->AddLJunctions(jcts, lines, near_point);
	}
	
	// remove candidate
	cand.Erase(i);
	candSide.Erase(i);
  }
}


/* 
**  TODO ARI: Wie kann man sicherstellen, dass 3 Linien existieren und 
**	wie kann die T-Junctions perfekt aussortieren?
**	Make new corner if corner has no Collinearity and don´t already exists
*/
void FormCorners::NewCorner(Array<unsigned> &lines, Array<unsigned> &jcts, 
	Array<unsigned> &near_point)
{
  // HACK ARI: Dies ist keine sichere überprüfung, dass keine T-Junctions 
  // verwendet werden.
  if (lines.Size() > 2)
    if (CheckCollinearity(lines[1], lines[2]) == UNDEF_ID)
	{
  	  NewGestalt(new Corner(lines, jcts, near_point));	
	  
	  // Insert number of corner to lines->corners
	  for (unsigned i=0; i<lines.Size(); i++)
	  {
		Lines(lines[i])->corners.PushBack(NumCorners()-1);
	  }
  	}
}

/*
**  TODO ARI: Könnte zu Corners als static kopiert werden (IsCorner?)
**	=> jede Linie des neuen muss im gefundenen Corner sein!
**
**	returns the number of  the corner, 
**	if the corner with the same lines already exists
*/
unsigned FormCorners::CornerExists(Array<unsigned> lines)
{
  // get all corners
  for (unsigned i=0; i<NumCorners(); i++)
  {
	// can find all new lines in a corner?
 	bool foundCorner = true;	
	for(unsigned j=0; j<lines.Size(); j++)
	{
	  bool foundLine = false;
	  for (unsigned k=0; k<Corners(i)->lines.Size(); k++)
	  {
		if (Corners(i)->lines[k] == lines[j])
		  foundLine = true;
	  }
	  if (!foundLine) foundCorner = false;
	}
	if (foundCorner) return i;

	// can find a corner where all lines in new lines?
 	foundCorner = true;	
	for (unsigned k=0; k<Corners(i)->lines.Size(); k++)
	  {
	  bool foundLine = false;
	  for(unsigned j=0; j<lines.Size(); j++)
	  {
		if (Corners(i)->lines[k] == lines[j])
		  foundLine = true;
	  }
	  if (!foundLine) foundCorner = false;
	}
	if (foundCorner) return i;
  }

  return UNDEF_ID;
}

/*
**	TODO ARI: Diese Funktion entspricht ungefähr FormJunctions::LBetween
**	Returns the number of the collinearity, 
**	if the collinearity with the two lines exists
*/
unsigned FormCorners::CheckCollinearity(unsigned l0, unsigned l1)
{
  unsigned exists = UNDEF_ID;
  for(unsigned i=0; i<NumCollinearities(); i++)
  {
	if (Collinearities(i)->line[0] == l0 ||
		Collinearities(i)->line[1] == l0)
	  if (Collinearities(i)->line[0] == l1 ||
		  Collinearities(i)->line[1] == l1)
	  	  exists = i;
  }
  return exists;
}

}
