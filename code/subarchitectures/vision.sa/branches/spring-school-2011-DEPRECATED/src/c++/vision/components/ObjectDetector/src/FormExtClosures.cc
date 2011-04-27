/**
 * $Id: FormExtClosures.cc,v 1.0 2007/10/31 17:27:03 mxz Exp mxz $
 */

#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "ExtClosure.hh"
#include "FormExtClosures.hh"

namespace Z
{
	
static int CmpExtClosures(const void *a, const void *b)
{
  if( ExtClosures(*(unsigned*)a)->sig > ExtClosures(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormExtClosures::Rank()
{
  RankGestalts(Gestalt::EXTCLOSURE, CmpExtClosures);
}

bool FormExtClosures::NeedsOperate()
{
  return false;
}

FormExtClosures::FormExtClosures(Config *cfg)
: GestaltPrinciple(cfg)
{
  firstCall=true;
}

/*
*	InformNewGestalt receives new rectangles
*/
void FormExtClosures::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  // ARI: try to built ExtClosure from the foregoing closure
  if (type == Gestalt::CLOSURE)
	Create(idx);
  StopRunTime();
}

void FormExtClosures::Operate(bool incremental)
{
//  Create();
}


/*
**	TODO ARI: 
*/
void FormExtClosures::Create(unsigned idx)
{
  unsigned c0 = idx;			// Closure to check

  // get shared lines from all closures c1
  for (unsigned c1=0; c1<c0; c1++)
  {
	Array<unsigned> sharedLines = GetSharedLines(c0, c1);
	if (sharedLines.Size() > 0)
	{
	  // get closure-strings without shared lines
	  Array<unsigned> linesC0 = GetOtherLines(sharedLines, c0);
	  Array<unsigned> linesC1 = GetOtherLines(sharedLines, c1);

	  if (linesC0.Size() > 0 && linesC1.Size() >0)
	  {
//printf("linesC0.Size() = %i\n", linesC0.Size());
//printf("linesC1.Size() = %i\n", linesC1.Size());		
		
		  // get end-points of closure strings (line-nr)
		  unsigned startC0 = linesC0[0];
		  unsigned endC0 = linesC0[linesC0.Size()-1];
		  unsigned startC1 = linesC1[0];
		  unsigned endC1 = linesC1[linesC1.Size()-1];
	
//printf("Closure: %i - %i\n",c0, c1);	
//for (unsigned u=0; u<linesC0.Size(); u++) printf("lines C0: %i\n", linesC0[u]);
//for (unsigned u=0; u<linesC1.Size(); u++) printf("lines C1: %i\n", linesC1[u]);
	
		  // try to connect the 4 line-endpoints
		  //startC0 - endC1 ==> con1
		  //startC1 - endC0 ==> con2
	
		  unsigned con1 = UNDEF_ID;
		  unsigned con2 = UNDEF_ID;
	
		  // get all L-Jcts and Colls
		  for(unsigned j=0; j<NumLJunctions(); j++)
		  {
			if (LJunctions(j)->line[LEFT] == startC0 &&
				LJunctions(j)->line[RIGHT] == endC1)
			{
//printf("\n    connection found 1\n");
			  con1 = j;
			}
			if (LJunctions(j)->line[LEFT] == startC1 &&
				LJunctions(j)->line[RIGHT] == endC0)
			{
//printf("\n    connection found 2\n");
			  con2 = j;
			}
		  }
		  
		  for(unsigned j=0; j<NumCollinearities(); j++)
		  {
			if (Collinearities(j)->line[LEFT] == startC0 &&
				Collinearities(j)->line[RIGHT] == endC1)
			{
//printf("\n    connection found 3\n");
			  con1 = j;
			}
			if (Collinearities(j)->line[LEFT] == startC1 &&
				Collinearities(j)->line[RIGHT] == endC0)
			{
//printf("\n    connection found 4\n");
			  con2 = j;
			}
		  }
		  
		  if (con1!=UNDEF_ID && con2!=UNDEF_ID)
		  {
			Array<unsigned> extLines = GetLines(sharedLines, c0, c1);
//printf("!!!!!!!!!!!! beide Connections gefunden !!!!!!!!\n");
//printf("con1= %i und con2= %i\n", con1, con2);
			NewGestalt(new ExtClosure(c0, c1, extLines, sharedLines));
		  }
	  }
	}
  }
}



/*
**	TODO ARI: funktioniert nicht (Create-Version 1)
**
*/
void FormExtClosures::Cr(unsigned idx)
{
/*  // find closures with sharedLine
  for (unsigned i=0; i<NumClosures()-2; i++)
  {
	Array<unsigned> sharedLines = GetSharedLines(idx, i);
	if (sharedLines.Size() > 0)
	{
	  // Closure->lines - sharedLines greater than 2?
	  if((Closures(idx)->lines.Size() - sharedLines.Size()) > 2)		// Schwachsinn!!!
	  {
		// get lines and junctions
		Array<unsigned> lines = GetLines(sharedLines, idx, i);	  
		Array<unsigned> jcts = GetLJunctions(idx, i);
		Array<unsigned> colls = GetCollinearities(idx, i);
		 
		bool closed = false;
		bool firstCall = true;

		Array<unsigned> newClosureLines;
		Array<unsigned> newClosureJcts;
		Array<unsigned> newClosureColls;		  
		  
		unsigned actLine = UNDEF_ID;
		unsigned firstLine = UNDEF_ID;
		 
		unsigned maxSteps = lines.Size()*2;			// UMÄNDERN (NOTFALLABBRUCH)
		  
		while (!closed)	
		{	
			maxSteps--;
printf("maxSteps: %u\n", maxSteps);
			if (maxSteps==0) break;
			
			// get first line
			if (firstCall)
			{
			  newClosureLines.PushBack(lines[0]);
			  actLine = lines[0];
			  firstLine = lines[0]; 
			  lines.Erase(0);		// delete from line
printf("new actLine: %u\n", actLine);
			  firstCall = false;
			}
			
			for (unsigned k=0; k<jcts.Size(); k++){
//printf("k<jcts.Size: %i<%i\n", k, jcts.Size());
			  // Erstes L-Junction-Ende probieren
			  if (LJunctions(jcts[k])->line[0] == actLine)				 
			  {
				  unsigned otherEnd = LJunctions(jcts[k])->line[1];
				  // wenn lines.Contains(actLine), dann
				  if (lines.Contains(otherEnd))
				  {
					actLine = otherEnd;
					newClosureJcts.PushBack(jcts[k]);
					newClosureLines.PushBack(actLine);
printf("Passende L-Junction gefunden: %u\n", jcts[k]);
printf("new actLine: %u\n", actLine);
					if (actLine == firstLine) closed = true;					// Abbruchbedingung 1
					jcts.Erase(k);
					break;	// break the for?
				  }
			  }
			  
			  // Zweites L-Junction-Ende probieren
			  if (LJunctions(jcts[k])->line[1] == actLine)
			  {
				  unsigned otherEnd = LJunctions(jcts[k])->line[0];
				  // wenn lines.Contains(actLine), dann
				  if (lines.Contains(otherEnd))
				  {
					actLine = otherEnd;
					newClosureJcts.PushBack(jcts[k]);			
					newClosureLines.PushBack(actLine);
printf("Passende L-Junction gefunden 2 %u\n", jcts[k]);
printf("new actLine: %u\n", actLine);
					if (actLine == firstLine) closed = true;					// Abbruchbedingung 2
					jcts.Erase(k);
					break;	// break the for?
				  }
			  }
		  	}  


// Doch Colls einzeln da sonst Array-Problem auftreten kann!
			for (unsigned k=0; k<colls.Size(); k++){

			  // Erstes Coll-Ende probieren
			  if (Collinearities(colls[k])->line[0] == actLine)				 
			  {
				  unsigned otherEnd = Collinearities(colls[k])->line[1];
				  // wenn lines.Contains(actLine), dann
				  if (lines.Contains(otherEnd))
				  {
					actLine = otherEnd;
					newClosureColls.PushBack(colls[k]);	
					newClosureLines.PushBack(actLine);
printf("Passende Coll gefunden: %u\n", colls[k]);
printf("new actLine: %u\n", actLine);
					if (actLine == firstLine) closed = true;					// Abbruchbedingung 3
					colls.Erase(k);
					break;	// break the for?
				  }
			  }
			  
			  // Zweites Coll-Ende probieren
			  if (Collinearities(colls[k])->line[1] == actLine)
			  {
				  unsigned otherEnd = Collinearities(colls[k])->line[0];
				  // wenn lines.Contains(actLine), dann
				  if (lines.Contains(otherEnd))
				  {
					actLine = otherEnd;
					newClosureLines.PushBack(actLine);
					newClosureColls.PushBack(colls[k]);			
printf("Passende Coll gefunden 2 %u\n", colls[k]);
printf("new actLine: %u\n", actLine);
					if (actLine == firstLine) closed = true;					// Abbruchbedingung 4
					colls.Erase(k);
					break;	// break the for?
				  }
			  }
		  }
	    }
		NewGestalt(new ExtClosure(idx, i, newClosureLines));
  	  		
		 
		//NewGestalt(new ExtClosure(idx, i, sharedLines));
  	  }	
	}
  }	    
*/
}


/*
**	Get all shared lines from two closures
**
*/
Array<unsigned> FormExtClosures::GetSharedLines(unsigned c0, unsigned c1)
{
  Array<unsigned> sharedLines;	// shared lines
	
  for(unsigned i=0; i<Closures(c0)->lines.Size(); i++)
  {
	unsigned lineC0 = Closures(c0)->lines[i];

	if (Closures(c1)->lines.Contains(lineC0))
	  sharedLines.PushBack(lineC0);
  }
  return sharedLines;
}

/*
**	Get all lines from two closures, without the shared lines
**
*/
Array<unsigned> FormExtClosures::GetLines(Array<unsigned> sharedLines, 
	unsigned c0, unsigned c1)
{
  Array<unsigned> lines; 	

  for (unsigned j=0; j<Closures(c0)->lines.Size(); j++)
  {
	if (!sharedLines.Contains(Closures(c0)->lines[j]))
	{
	   lines.PushBack(Closures(c0)->lines[j]);
	}
  }

  for (unsigned j=0; j<Closures(c1)->lines.Size(); j++)
  {
	if (!sharedLines.Contains(Closures(c1)->lines[j]))
	{
	  lines.PushBack(Closures(c1)->lines[j]);		  
	}
  }
  
  return lines;
}

/*
**	Get lines from one closure, without the shared lines
**
*/
Array<unsigned> FormExtClosures::GetOtherLines(Array<unsigned> sharedLines, 
	unsigned c0)
{
  Array<unsigned> lines; 	

  for (unsigned j=0; j<Closures(c0)->lines.Size(); j++)
  {
	if (!sharedLines.Contains(Closures(c0)->lines[j]))
	   lines.PushBack(Closures(c0)->lines[j]);
  }
  return lines;
}


/*
**	Get all L-junctions from two closures
**	
*/
Array<unsigned> FormExtClosures::GetLJunctions(unsigned c0, unsigned c1)
{
  Array<unsigned> jcts;		// all L-Junctions 
	
  for (unsigned j=0; j<Closures(c0)->jcts.Size(); j++)
  {
	if(Closures(c0)->jcts[j] != UNDEF_ID)
	{
		jcts.PushBack(Closures(c0)->jcts[j]);
	}
  }
  for (unsigned j=0; j<Closures(c1)->jcts.Size(); j++)
  {
	if(Closures(c1)->jcts[j] != UNDEF_ID)
	{
	  jcts.PushBack(Closures(c1)->jcts[j]);
	}
  }

  return jcts;
 }
  

/*
**	Get all collinearities from two closures
**
*/
Array<unsigned> FormExtClosures::GetCollinearities(unsigned c0, unsigned c1)
{
  Array<unsigned> coll;		// all collinearities

  for (unsigned j=0; j<Closures(c0)->colls.Size(); j++)
  {
	if(Closures(c0)->colls[j] != UNDEF_ID)
	{
	  coll.PushBack(Closures(c0)->colls[j]);
	}
  }
  for (unsigned j=0; j<Closures(c1)->colls.Size(); j++)
  {
	if(Closures(c1)->colls[j] != UNDEF_ID)
	{
	  coll.PushBack(Closures(c1)->colls[j]);
	}
  }

  return coll;
}	




}
