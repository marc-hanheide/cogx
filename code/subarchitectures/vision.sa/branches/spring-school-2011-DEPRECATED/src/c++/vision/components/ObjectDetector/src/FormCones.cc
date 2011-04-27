/**
 * @file FormCones.cc
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Gestalt Principle FormCones
 **/

#include "FormCones.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "ExtEllipse.hh"
#include "Cone.hh"
#include "LJunction.hh"
#include "EJunction.hh"

namespace Z
{

static int CmpCones(const void *a, const void *b)
{
  if( Cones(*(unsigned*)a)->sig > Cones(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormCones::Rank()
{
  RankGestalts(Gestalt::CONE, CmpCones);
}

/**
 *	@brief Mask cones which have the center point inside the cube radius of another
 *	cube with higher significance.
 */
void FormCones::Mask()
{
  for(unsigned i=0; i<NumCones(); i++)
  {
		for(unsigned j=0; j<NumCones(); j++)
		{
			if(!Cones(i)->IsMasked() && !Cones(j)->IsMasked())
			if(Cones(i)->sig < Cones(j)->sig)
			{
				if(Cones(i)->IsInside(j))
				{	
					Cones(i)->Mask(j);		 
				}
			}		  
		}
  }
}

bool FormCones::NeedsOperate()
{ 
  return needsOperate;	
}

/**
 * @brief Constructor of class FormCones.
 * @param cfg Config
 */
FormCones::FormCones(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = false;
}

/**
 * @brief InformNewGestalt
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormCones::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  if(VisionCore::config.GetValueInt("FORM_EXTELLIPSES") != 1) return;

  switch(type)
  {
    case Gestalt::E_JUNCTION:
	  NewEJunction(idx);
	  break;
    case Gestalt::L_JUNCTION:
	  NewLJunction(idx);
      break;
    default:
      break;
  }
	Mask();
	Rank();	

  StopRunTime();
}

/**
 * @brief Operate()
 * @param bool Incremental function
 */
void FormCones::Operate(bool incremental)
{
/*  StartRunTime();
  Create();
  StopRunTime();
*/
}

/**
 *	@brief NewEJunction()
 *	@param ejct Index of new E-Junction
 */
void FormCones::NewEJunction(unsigned ejct)
{
  unsigned ellipse = EJunctions(ejct)->ellipse;
  unsigned extEllipse = Ellipses(ellipse)->extEllipse;

  // extLines from the extEllipse
  Array<unsigned> extLines = ExtEllipses(extEllipse)->extLines;
  Array<unsigned> extLinesEnd = ExtEllipses(extEllipse)->extLinesEnd;
  Array<unsigned> extLinesVertex = ExtEllipses(extEllipse)->extLinesVertex;

  // collLines from the extEllipse
  Array<unsigned> colLines = ExtEllipses(extEllipse)->colLines;
  Array<unsigned> colLinesEnd = ExtEllipses(extEllipse)->colLinesEnd;
  Array<unsigned> colLinesVertex = ExtEllipses(extEllipse)->colLinesVertex;

  //<<<<<< search L-Junction between extLines >>>>>//
  for(unsigned i=0; i<extLines.Size(); i++)
  {
		// get L-Junctions of extLinesEnd
		Array<unsigned> ljcts;
		Lines(extLines[i])->GetAllLJunctions(&ljcts);
		
		// every ljct k
		for (unsigned k=0; k<ljcts.Size(); k++)
		{
			// get left and right arm (line) from the ljct
			unsigned line[2];
			line[LEFT] = LJunctions(ljcts[k])->line[LEFT];
			line[RIGHT] = LJunctions(ljcts[k])->line[RIGHT];
				
			// is left or right line equal extLine 
			unsigned side, other;	
			if (line[LEFT] == extLines[i])
			{
				side = LEFT;
				other = RIGHT;
			}
			else 
			{
				side = RIGHT;
				other = LEFT;
			}
				
			// extLines contains also other line of L-Junction?
			if (extLines.Contains(line[other]))
			{	
				// search position of other line in extLines
				unsigned oLpos;
				for(unsigned l=0; l<extLines.Size(); l++)
				{
					if (extLines[l] == line[other])
						oLpos = l;			  
				}
		
				// L-Jct and E-Jct are at the different end of both lines
				if (extLinesVertex[i] != extLinesVertex[oLpos] &&									// different ellipse vertex
					extLinesEnd[i] != LJunctions(ljcts[k])->near_point[side] &&			// different line ends (L-/E-Jct)
					extLinesEnd[oLpos] != LJunctions(ljcts[k])->near_point[other])	// different line ends (L-/E-Jct) 
				{
					NewCone(ExtEllipses(extEllipse)->ellipse, extLines[i],
							extLines[oLpos], ljcts[k]);
					return;			
				}
			}
    }
  }
  
  //<<<<<< search L-Junction between colLine and extLine >>>>>//
  for(unsigned i=0; i<colLines.Size(); i++)
  {
		// get L-Junctions of extLinesEnd
		Array<unsigned> ljcts;
		Lines(colLines[i])->GetAllLJunctions(&ljcts);
		
		// every ljct k
		for (unsigned k=0; k<ljcts.Size(); k++)
		{
			// get left and right arm (line) from the ljct
			unsigned line[2];
			line[LEFT] = LJunctions(ljcts[k])->line[LEFT];
			line[RIGHT] = LJunctions(ljcts[k])->line[RIGHT];
				
			// is left or right line equal extLine 
			unsigned side, other;	
			if (line[LEFT] == colLines[i])
			{
				side = LEFT;
				other = RIGHT;
			}
			else 
			{
				side = RIGHT;
				other = LEFT;
			}
				
			// extLines contains also other line of L-Junction?
			if (extLines.Contains(line[other]))
			{	
				// search position of other line in extLines
				unsigned oLpos;
				for(unsigned l=0; l<extLines.Size(); l++)
				{
					if (extLines[l] == line[other])
						oLpos = l;			  
				}
		
				// L-Jct and E-Jct are at the different end of both lines
				if (colLinesVertex[i] != extLinesVertex[oLpos] &&										// different ellipse vertex
					colLinesEnd[i] != LJunctions(ljcts[k])->near_point[side] &&				// different line ends (L-/E-Jct)
					extLinesEnd[oLpos] != LJunctions(ljcts[k])->near_point[other])		// different line ends (L-/E-Jct) 
				{
					NewCone(ExtEllipses(extEllipse)->ellipse, colLines[i],
					extLines[oLpos], ljcts[k]);
					return;			
				}
			}
			
			// colLines contains also other line of L-Junction?								// TODO ARI: Hat das hier Sinn? gute Ergebnisse?
			if (colLines.Contains(line[other]))
			{	
				// search position of other line in extLines
				unsigned oLpos;
				for(unsigned l=0; l<colLines.Size(); l++)
				{
					if (colLines[l] == line[other])
						oLpos = l;			  
				}
		
				// L-Jct and E-Jct are at the different end of both lines
				if (colLinesVertex[i] != colLinesVertex[oLpos] &&										// different ellipse vertex
					colLinesEnd[i] != LJunctions(ljcts[k])->near_point[side] &&				// different line ends (L-/E-Jct)
					colLinesEnd[oLpos] != LJunctions(ljcts[k])->near_point[other])		// different line ends (L-/E-Jct) 
				{
					NewCone(ExtEllipses(extEllipse)->ellipse, colLines[i], colLines[oLpos], ljcts[k]);
					return;			
				}
			}
    }
  }
}

/**
 *	@brief NewLJunction()
 *	@param ljct Index of new ljct
 */
void FormCones::NewLJunction(unsigned ljct)
{
  // get the extEllipses from both lines of the L-Junction
  Array<unsigned> eE0 = Lines(LJunctions(ljct)->line[0])->extEllipses;
  Array<unsigned> eE1 = Lines(LJunctions(ljct)->line[1])->extEllipses;
	
  // compare all extEllipses of both lines
  for(unsigned i=0; i<eE0.Size(); i++)
  {
		if(eE1.Contains(eE0[i]))
		{
			unsigned ellipse = ExtEllipses(eE0[i])->ellipse;
			unsigned eL0 = LJunctions(ljct)->line[0];
			unsigned eL1 = LJunctions(ljct)->line[1];
	
			// lines: l-junction and e-junctions end/vertex
			unsigned eL0LJctEnd = LJunctions(ljct)->near_point[0];	// LJct-End
			unsigned eL1LJctEnd = LJunctions(ljct)->near_point[1];  
			unsigned eL0EJctEnd;		// EJct-End?
			unsigned eL1EJctEnd;
			unsigned eL0EJctVertex;	// EJct-Vertex?
			unsigned eL1EJctVertex;				
			
			for(unsigned j=0; j<ExtEllipses(eE0[i])->extLines.Size(); j++)
			{
				if (eL0 == ExtEllipses(eE0[i])->extLines[j])
				{
					eL0EJctEnd = ExtEllipses(eE0[i])->extLinesEnd[j];
					eL0EJctVertex = ExtEllipses(eE0[i])->extLinesVertex[j];
				}
				if (eL1 == ExtEllipses(eE0[i])->extLines[j])
				{
					eL1EJctEnd = ExtEllipses(eE0[i])->extLinesEnd[j];
					eL1EJctVertex = ExtEllipses(eE0[i])->extLinesVertex[j];
				}
			}
			for(unsigned j=0; j<ExtEllipses(eE0[i])->colLines.Size(); j++)
			{
				if (eL0 == ExtEllipses(eE0[i])->colLines[j])
				{
					eL0EJctEnd = ExtEllipses(eE0[i])->colLinesEnd[j];
					eL0EJctVertex = ExtEllipses(eE0[i])->colLinesVertex[j];
				}
				if (eL1 == ExtEllipses(eE0[i])->colLines[j])
				{
					eL1EJctEnd = ExtEllipses(eE0[i])->colLinesEnd[j];
					eL1EJctVertex = ExtEllipses(eE0[i])->colLinesVertex[j];
				}
			}
		
			// e-jct and l-jct at different line-ends
			// lines on different vertex of ellipse
			if (eL0LJctEnd != eL0EJctEnd &&	eL1LJctEnd != eL1EJctEnd &&	eL0EJctVertex != eL1EJctVertex)
				NewCone(ellipse, eL0, eL1, ljct);
		}
  }
}

/**
 *	@brief Generate new Gestalt cone.
 *	@param ell Ellipse index
 *	@param eL0 First ellipse line
 *	@param eL1 Second ellipse line
 *	@param ljct Index of L-Junction
 *	@TODO Überprüfungen einbauen?
 */
void FormCones::NewCone(unsigned ell, unsigned eL0, unsigned eL1, unsigned ljct)
{
  // Check if cone already exists
  bool newCone = true;
  Array<unsigned> cones = Ellipses(ell)->cones;
	
  for(unsigned i=0; i<cones.Size(); i++)
		if((Cones(cones[i])->line[0] == eL0 && Cones(cones[i])->line[1] == eL1) ||
			 (Cones(cones[i])->line[0] == eL1 && Cones(cones[i])->line[1] == eL0))
				newCone = false;

	// do not create a cone, if ellipse radius a is smaller than 10.
	if(Ellipses(ell)->a < 10.)
		newCone = false;
	
  if (newCone)
		NewGestalt(new Cone(ell, eL0, eL1, ljct));
}


}
