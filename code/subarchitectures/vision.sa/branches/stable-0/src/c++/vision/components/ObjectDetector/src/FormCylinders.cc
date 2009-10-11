/**
 * @file FormCylinders.cc
 * @author Andreas Richtsfeld
 * @date December 2007
 * @version 0.1
 * @brief Gestalt Principle FormCylinders
 **/

#include "FormCylinders.hh"
#include "Line.hh"
#include "EJunction.hh"
#include "Ellipse.hh"
#include "ExtEllipse.hh"
#include "Cylinder.hh"

namespace Z
{

static int CmpCylinders(const void *a, const void *b)
{
  if( Cylinders(*(unsigned*)a)->sig > Cylinders(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormCylinders::Rank()
{
  RankGestalts(Gestalt::CYLINDER, CmpCylinders);
}

/**
 * @brief Constructor of class FormCylinders
 * @param cfg Config
 */
FormCylinders::FormCylinders(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = false;
}

/**
 *	@brief Mask cylinders which have the center point inside the cylinder radius of another
 *	cylinder with higher significance.
 */
void FormCylinders::Mask()
{
  for(unsigned i=0; i<NumCylinders(); i++)
  {
		for(unsigned j=0; j<NumCylinders(); j++)
		{
			if(!Cylinders(i)->IsMasked() && !Cylinders(j)->IsMasked() && i!=j)
			if(Cylinders(i)->sig <= Cylinders(j)->sig)
			{
				if(Cylinders(i)->IsInside(j))
				{	
					Cylinders(i)->Mask(j);
				}
			}
		}
  }
}



/**
 * @brief InformNewGestalt()	
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormCylinders::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  if(VisionCore::config.GetValueInt("FORM_EXTELLIPSES") != 1) return;
  StartRunTime();

  if (type == Gestalt::E_JUNCTION)
  	NewEJunction(idx);

	Rank();
	Mask();

  StopRunTime();
}


/**
 * @brief Operate()
 * @param bool Incremental function
 */
void FormCylinders::Operate(bool incremental)
{
}


/**
 *	@brief NewEJunction(): Try to create a new cylinder, if a new E-Junction is available
 *	@param ejct E-Junction index
 */
void FormCylinders::NewEJunction(unsigned ejct)
{
  unsigned extEllipse = Ellipses(EJunctions(ejct)->ellipse)->extEllipse;

  // extLines from the extEllipse
  Array<unsigned> extLines = ExtEllipses(extEllipse)->extLines;
  Array<unsigned> extLinesEnd = ExtEllipses(extEllipse)->extLinesEnd;
  Array<unsigned> extLinesVertex = ExtEllipses(extEllipse)->extLinesVertex;

  // collLines from the extEllipse
  Array<unsigned> colLines = ExtEllipses(extEllipse)->colLines;
  Array<unsigned> colLinesEnd = ExtEllipses(extEllipse)->colLinesEnd;
  Array<unsigned> colLinesVertex = ExtEllipses(extEllipse)->colLinesVertex;

  //<<<<< connection between ellipses with sharedLines >>>>>//
	
  // search all ExtEllipse (eE) for the extLines
  Array<unsigned> eE; 
  for(unsigned i=0; i<extLines.Size(); i++)
  {  
		Array<unsigned> extEllOfExtLines = Lines(ExtEllipses(extEllipse)->extLines[i])->extEllipses;			// TODO ARI: Mit lines->GetAllCollinearities()
		for(unsigned j=0; j<extEllOfExtLines.Size(); j++)
		{
			if (!eE.Contains(extEllOfExtLines[j]) && extEllipse != extEllOfExtLines[j])
				eE.PushBack(extEllOfExtLines[j]);
		}
  }

  // every extEllipse eE[i]
  for(unsigned i=0; i<eE.Size(); i++)
  {
		Array<unsigned> sharedLines;
		Array<unsigned> sharedLinesVertex[2];
			
		// get all extLines j of eE[i]
		for (unsigned j=0; j<ExtEllipses(eE[i])->extLines.Size(); j++)
		{
			// get all extLines k
			for (unsigned k=0; k<extLines.Size(); k++)
			{
				if (extLines[k] == ExtEllipses(eE[i])->extLines[j] &&					// same lines
					extLinesEnd[k] != ExtEllipses(eE[i])->extLinesEnd[j])				// different nearPoints of extended lines => TODO: Gibt es hier Fehler?
				if (!sharedLines.Contains(extLines[k]))
				{
					sharedLines.PushBack(extLines[k]);
					sharedLinesVertex[0].PushBack(extLinesVertex[k]);
					sharedLinesVertex[1].PushBack(ExtEllipses(eE[i])->extLinesVertex[j]);
				}
			}
		}

		if (sharedLines.Size() > 0)
			NewCylinder(extEllipse, eE[i], sharedLines, sharedLinesVertex);
	}
  
	//<<<<< connection between ellipses with sharedLine and colLine >>>>>//

	// search all ExtEllipse (eEcL) for the colLines
	Array<unsigned> eEcL; 
	for(unsigned i=0; i<colLines.Size(); i++)
	{  
		Array<unsigned> extEllOfColLines = Lines(ExtEllipses(extEllipse)->colLines[i])->extEllipses;
		for(unsigned j=0; j<extEllOfColLines.Size(); j++)
		{
			if (!eEcL.Contains(extEllOfColLines[j]) && extEllipse != extEllOfColLines[j])
				eEcL.PushBack(extEllOfColLines[j]);
		}
	}

	// every ExtEllipse eEcL[i]
	for(unsigned i=0; i<eEcL.Size(); i++)
	{
		Array<unsigned> sharedLines;	
		Array<unsigned> sharedLinesVertex[2];
			
		// get all extLines of eEcL
		for (unsigned j=0; j<ExtEllipses(eEcL[i])->extLines.Size(); j++)
		{
			// get all extLines of extEllipse
			for (unsigned k=0; k<colLines.Size(); k++)
			{
			if (colLines[k] == ExtEllipses(eEcL[i])->extLines[j] &&					// same lines
				colLinesEnd[k] != ExtEllipses(eEcL[i])->extLinesEnd[j])				// different nearPoints of extended lines   	// TODO: Hier bei den Ends können fehlerhafte Werte auftreten!
				if (!sharedLines.Contains(colLines[k]))
				{
					sharedLines.PushBack(colLines[k]);
					sharedLinesVertex[0].PushBack(colLinesVertex[k]);
					sharedLinesVertex[1].PushBack(ExtEllipses(eEcL[i])->extLinesVertex[j]);
				}
			}
		}

	if (sharedLines.Size() > 0)
	  NewCylinder(extEllipse, eEcL[i], sharedLines, sharedLinesVertex);
  }
}

/**
 *	@brief NewCylinder(): Make new cylinder, if cylinder don´t already exists
 *	Other conditions ???
 *	@param eE0 Extended ellipse 0
 *	@param eE1 Extended ellipse 1
 *	@param sL Shared lines
 *	@param sLVtx Shared lines vertices
 */
void FormCylinders::NewCylinder(unsigned eE0, unsigned eE1, Array<unsigned> &sL, Array<unsigned> *sLVtx)
{
  // Check if cylinder already exists
  bool newCylinder = true;
  Array<unsigned> cyl = Ellipses(ExtEllipses(eE0)->ellipse)->cylinders;
	
  for(unsigned i=0; i<cyl.Size(); i++)
  {
		if((Cylinders(cyl[i])->extEllipses[0] == eE0 &&	Cylinders(cyl[i])->extEllipses[1] == eE1) ||
			(Cylinders(cyl[i])->extEllipses[0] == eE1 && Cylinders(cyl[i])->extEllipses[1] == eE0))
		{
			Cylinders(cyl[i])->AddSharedLines(sL, sLVtx);
			newCylinder = false;
		}
  }

  if (newCylinder)
  {
/*		// Condition:
		// The line between ellipse1->vertex[LEFT/RIGHT] and 
		// ellipse2->vertex[LEFT/RIGHT] should have the same direction as the mean
		// of the normals of the both ellipses. => (meanDir ~ phi0, phi1 )
	
		Vector2 vertex[2][2]; 		///< vertex [ELL0/ELL1] [LEFT/RIGHT]
		vertex[0][0] = ExtEllipses(eE0)->vertex[0];
		vertex[0][1] = ExtEllipses(eE0)->vertex[1];
		vertex[1][0] = ExtEllipses(eE1)->vertex[0];
		vertex[1][1] = ExtEllipses(eE1)->vertex[1];
		
		// mean direction: Normals of the ellipses 
		double phiEll0 = Ellipses(ExtEllipses(eE0)->ellipse)->phi;
		double phiEll1 = Ellipses(ExtEllipses(eE1)->ellipse)->phi;
		double meanDir = ScaleAngle_0_pi(M_PI/2. + (phiEll0 + phiEll1)/2.);			// stimmt das so???
	
		// calculate direction of the 2 connecting "lines" 
		double diff0 = 0, diff1 = 0, phi0 = 0, phi1 = 0;
		
		double x0 = vertex[0][0].x - vertex[1][0].x;
		double y0 = vertex[0][0].y - vertex[1][0].y;
		double x1 = vertex[0][1].x - vertex[1][1].x;
		double y1 = vertex[0][1].y - vertex[1][1].y;
	
		if (x0 != 0 && x1 != 0)
		{
			phi0 = atan(y0/x0);	// angle of connecting "lines"
			phi1 = atan(y1/x1);
			if (phi0 < 0) phi0 += M_PI;
			if (phi1 < 0) phi1 += M_PI;
				
			// deviation between ellipse meanDir and angle of connecting "lines"
			diff0 = fabs(meanDir - ScaleAngle_0_pi(phi0)); 
			diff1 = fabs(meanDir - ScaleAngle_0_pi(phi1)); 
		}
		else printf("Error: FormCylinders: x0 or x1 is 0!\n");
				
		double threshold = M_PI/4.;		// Threshold for the "same" direction
		
		// Deviation between connecting lines and the mean ellipse normals
		if (diff0 < threshold && diff1 < threshold)
			newCylinder = true;
		else
			newCylinder = false;

		// The two connecting lines should be "parallel"
		// The ellipse normals should be the "same"
    if (fabs(phi0 - phi1) > threshold || fabs(phiEll0-phiEll1) > threshold) newCylinder = false;
*/
// TODO ARI: Aussortieren der Cylinder ansehen!
//printf("\nCylinder %i: ", NumCylinders());
//printf(" phiEll0, phiEll1, meanDir: %f, %f, %f\n", phiEll0, phiEll1, meanDir);
//printf("fabs(phiEll0-phiEll1): %f\n", fabs(phiEll0-phiEll1));

/*    if (fabs(phi0 - phi1) > threshold) printf("		FALSE: %f\n", fabs(phi0 - phi1));
      if(newCylinder) printf("true\n");
	else printf("false\n");
		
	printf(" diff0, diff1: %f - %f\n", diff0, diff1);  
	printf(" phiEll0, phiEll1, meanDir: %f, %f, %f\n", phiEll0, phiEll1, meanDir);
*/
		if (newCylinder) NewGestalt(new Cylinder(eE0, eE1, sL, sLVtx));
  }  
}

bool FormCylinders::NeedsOperate()
{ 
  return needsOperate;	
}

}
