/**
 * @file FormEJunctions.cc
 * @author Richtsfeld Andreas
 * @date 2010
 * @version 0.1
 * @brief Header file of Gestalt principle FormEJunction (ellipse junction).
 **/


#include "FormEJunctions.hh"

namespace Z
{

enum GrowMethod {GROW_EQUAL, GROW_WEIGHTED, GROW_SMART};
static GrowMethod grow_method = GROW_WEIGHTED;

FormEJunctions::FormEJunctions(VisionCore *vc) : GestaltPrinciple(vc)
{
  vote_img = 0;
  first_op = true;
  SetupAdmissibilityMatrix();
	baseIndex = 8;
	baseOffset = 0;   // offset for ellipses (=number of estimated lines!)
}

FormEJunctions::~FormEJunctions()
{
  delete vote_img;
}

/**
 * @brief Defines, which combinations of search lines are valid and sets up
 * the admissibility matrix.
 */
void FormEJunctions::SetupAdmissibilityMatrix()
{
  int i, j;
  for(i = 0; i < 8; i++)
    for(j = 0; j < 8; j++)
      isct_ok[i][j] = false;

  isct_ok[VOTE_E][VOTE_EOTL] = true; 	// line edge itself & ellipse tangents
  isct_ok[VOTE_E][VOTE_EOTR] = true;	
  isct_ok[VOTE_E][VOTE_EITL] = true;
  isct_ok[VOTE_E][VOTE_EITR] = true;
  
  isct_ok[VOTE_TS][VOTE_EOTL] = true;	// line tangents - outer ellipse tangents
  isct_ok[VOTE_TS][VOTE_EOTR] = true;
  isct_ok[VOTE_TE][VOTE_EOTL] = true;
  isct_ok[VOTE_TE][VOTE_EOTR] = true;

  isct_ok[VOTE_TS][VOTE_EITL] = true;	// line tangents - inner ellipse tangents
  isct_ok[VOTE_TS][VOTE_EITR] = true;
  isct_ok[VOTE_TE][VOTE_EITL] = true;
  isct_ok[VOTE_TE][VOTE_EITR] = true;

  // now mirror along diagonal to fill lower left side of matrix
  for(i = 0; i < 8; i++)
    for(j = i + 1; j < 8; j++)
      isct_ok[j][i] = isct_ok[i][j];
}

/**
 * @brief Set the number of lines, for vote image initialisation.
 * Must be set, before extension of vote lines starts.
 * @return Returns true, if number of lines was set.
 */
bool FormEJunctions::SetNumLines()
{
	baseOffset = NumLines(core);
	if(baseOffset == 0) return false;
	else return true;
}

/**
 * @brief Reset the principle. Clear vote image.
 */
void FormEJunctions::Reset()
{
  if(core->HaveImage())
  {
    if(vote_img != 0 &&
       !(vote_img->width == core->GetImage()->width &&
         vote_img->height == core->GetImage()->height))
    {
      delete vote_img;
      vote_img = 0;
    }
    if(vote_img == 0)
      vote_img = new VoteImage(core->GetImage()->width, core->GetImage()->height);
  }
  vote_img->Clear();
  first_op = true;
}


/**
 * @brief Inform the Gestalt principle about new Gestalts.
 * When a new Ellipse comes up, we have to extend the vote image search line space.
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormEJunctions::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	StartRunTime();
	if(first_op)
	{
		printf("FormEJunctions::InformNewGestalt: Error: NumLines not set.\n");
		SetNumLines();
		return;
	}
	
	if(type == Gestalt::ELLIPSE)
	{
		vote_img->ExtendNumLines((baseOffset+idx+1)*8);
    InitEllipseSearchLines((Ellipse*)core->Gestalts(Gestalt::ELLIPSE, idx));
	}
	if(type == Gestalt::COLLINEARITY)
	{
		UpdateEJunctions(idx);
	}
	
	StopRunTime();
}


/**
 * @brief Operate the Gestalt principle
 * @param incremental Calculate incremental
 */
void FormEJunctions::Operate(bool incremental)
{
	StartRunTime();
	if(first_op) first_op == !SetNumLines();
  if(incremental)
    OperateIncremental();
  else
    OperateNonIncremental();
	StopRunTime();
}

/**
 * @brief Post operate the Gestalt principle.
 * Mask the e-junctions after processing.
 */
void FormEJunctions::PostOperate()
{
	StartRunTime();
	Mask();
	StopRunTime();
}

/**
 * @brief Operate the Gestalt principle incremental.
 * First initialise the vote image lines, then start growing of lines.
 */
void FormEJunctions::OperateIncremental()
{
  if(first_op)
  {
    // in first call just init search lines, don't create junctions
    vote_img->SetNumLines(NumLines(core)*8);
    for(unsigned r = 0; r < NumLines(core); r++)
      InitLineSearchLines((Line*)core->RankedGestalts(Gestalt::LINE, r));
    first_op = false;
  }
  else
  {
//     // for all subsequent calls 
//     // try to be smart about growing search lines
//     if(grow_method == GROW_SMART)
//     {
// 			// HACK: not implemented yet
// 			//int r = ExpSelect(NumArcs(core) - 1);
// 			//ExtendSmartLines((Arc*)core->RankedGestalts(Gestalt::ARC, r));

		float gest = FRand();  // ARI: Random number 0-1
		float LiToEl = (float)baseOffset / ((float)baseOffset + (float)NumEllipses(core));

		if(grow_method == GROW_SMART)
		{
			// TODO ARI: gest wählt per Zufallszahl ob Ellipse oder Line extended 
			// wird => diese Wahl wird je nach Anzahl von Linien und Ellipsen
			// gewählt
			if (gest < LiToEl)	
			{
				int r = ExpSelect(baseOffset-1);
				ExtendSmartLines(Gestalt::LINE, core->RankedGestalts(Gestalt::LINE, r)->ID());
			}
			else
			{
				int r = ExpSelect(NumEllipses(core) - 1);
				ExtendSmartLines(Gestalt::ELLIPSE, core->RankedGestalts(Gestalt::ELLIPSE, r)->ID());
			}
		}

    // lines grow probabilistically according to length
    else if(grow_method == GROW_WEIGHTED)
    {
			if (gest < LiToEl)	
			{
				int r = ExpSelect(baseOffset - 1);
				ExtendSearchLines((Line*) core->RankedGestalts(Gestalt::LINE, r));
			}
			else
			{
				int r = ExpSelect(NumEllipses(core) - 1);
				ExtendSearchLines((Ellipse*) core->RankedGestalts(Gestalt::ELLIPSE, r));
			}
    }

		// all search lines grow equally, this is less "anytime-ish"
    else if(grow_method == GROW_EQUAL)
    {
			if (gest < LiToEl)	
			{
				for(unsigned r = 0; r < baseOffset-1; r++)
					ExtendSearchLines((Line*) core->RankedGestalts(Gestalt::LINE, r));
			}
			else
			{
				for(unsigned r = 0; r < NumEllipses(core); r++)
					ExtendSearchLines((Ellipse*) core->RankedGestalts(Gestalt::ELLIPSE, r));
			}
    }
  }
}

/**
 * @brief Operates the principle non-incremental.
 * Operates, until all lines reached the maximum length.
 */
void FormEJunctions::OperateNonIncremental()
{
	printf("FormEJunctions::OperateNonIncremental: Not yet implemented!\n");
//   if(first_op)  // do only once
//   {
//     unsigned narcs = NumArcs(core);
//     vote_img->SetNumLines(narcs*8);
//     for(unsigned r = 0; r < narcs; r++)
//     {
//       Arc *arc = (Arc*)core->RankedGestalts(Gestalt::ARC, r);
//       double len = min(arc->Radius(), arc->ArcLength());
//       InitSearchLines(arc);
//       // draw all search lines to a length of l
//       for(int j = 0; j < len; j++)
//         ExtendSearchLines(arc);
//     }
//     first_op = false;
//   }
}


/**
 * @brief Initialisation of search lines for ellipse junctions.
 * We define only tangential search lines for lines and normal search lines 
 * for ellipses into the inner and outer direction at the vertices.
 */
void FormEJunctions::InitLineSearchLines(Line *line)
{
  double len = 1000;                         // (max) length	// TODO: replace this by "until image border"
  unsigned sline = line->ID()*baseIndex;     // search line 

  // draw edge itself
  vote_img->DrawLine(line->point[START], line->point[END], sline + VOTE_E);
  // initialise tangents for growing
  vote_img->InitLine(line->point[START], line->point[START] - len*line->dir, sline + VOTE_TS);
  vote_img->InitLine(line->point[END], line->point[END] + len*line->dir, sline + VOTE_TE);
}
	
	
/**
 * @brief Initialisation of search lines for ellipse junctions.
 * We define only tangential search lines for lines and normal search lines 
 * for ellipses into the inner and outer direction at the vertices. \n
 * When we are initialising the four search lines for the ellipses, we are
 * checking for intersections in the two vertices of the ellipse, because there 
 * could be already a line in the vote image. \n
 * NOTE: This initialisation works incrementally, because ellipses appear now
 * also incrementally.
 * @param ell Ellipse
 */
void FormEJunctions::InitEllipseSearchLines(Ellipse *ell)
{
  // calculate length of the major axis
  double majorAxis = (ell->vertex[1] - ell->vertex[0]).Norm();

  // limit the maximal length of the search lines
  unsigned len = (unsigned) majorAxis/4; 

  // search line index for this ellipse
  unsigned sline = (baseOffset + ell->ID())*baseIndex;	/// TODO TODO TODO baseOffset-1 ????

	// Check vertex points for intersections with the already drawn lines
	iscts.Clear();

	if(ell->vertex[0].x < core->IW() && ell->vertex[0].y < core->IH())
		vote_img->CheckPixel(ell->vertex[0].x, ell->vertex[0].y, sline, iscts);
	if(iscts.Size() > 0)
		CreateJunctions(sline+VOTE_EOTL, iscts);

	if(ell->vertex[1].x < core->IW() && ell->vertex[1].y < core->IH())
		vote_img->CheckPixel(ell->vertex[1].x, ell->vertex[1].y, sline, iscts);
	if(iscts.Size() > 0)
		CreateJunctions(sline+VOTE_EOTR, iscts);

	// initialise outer major axis search lines
  vote_img->InitLine(ell->vertex[LEFT], ell->vertex[LEFT] - len*ell->dir, sline + VOTE_EOTL);
  vote_img->InitLine(ell->vertex[RIGHT], ell->vertex[RIGHT] + len*ell->dir, sline + VOTE_EOTR);

  // initialise inner major axis search lines
  vote_img->InitLine(ell->vertex[LEFT], ell->vertex[LEFT] + len*ell->dir, sline + VOTE_EITL);
  vote_img->InitLine(ell->vertex[RIGHT], ell->vertex[RIGHT] - len*ell->dir, sline + VOTE_EITR);
}


/**
 * @brief Be somewhat smart about extending search lines.
 * @param type Type of Gestalt
 * @param idx Index of Gestalt
 */
void FormEJunctions::ExtendSmartLines(Gestalt::Type type, unsigned idx)								/// TODO Gibt es überhaupt so was wie smart hier?
{
 	printf("FormEJunctions::ExtendSmartLines: Not yet implemented!\n");
//   int end;
	
  // if type is a line
//   if (type == Gestalt::LINE)
//   {
//     unsigned line = idx;
	// Selecting which end to extend is given by the number of junctions already
	// present at that end: p(START)/p(END) = n(END)/n(START).
	// Regarding T-junctions: Each T-jct at one end is accompanied by two L-jcts.
	// Therefore reduce the count of jcts at this side by two.
	// Furthermore add 1 to the jct count on the other end to further favour this
	// end.
	
// 	int n_t[2] = {(Lines(core, idx)->t_jct[START] != UNDEF_ID ? 1 : 0),
//                 (Lines(core, idx)->t_jct[END] != UNDEF_ID ? 1 : 0)};
// 	int n_s = 1 + Lines(line)->coll[START].Size() +
//     	Lines(line)->l_jct[START][LEFT].Size() +
//     	Lines(line)->l_jct[START][RIGHT].Size() - 2*n_t[START] + n_t[END];

//   	int n_e = 1 + Lines(line)->coll[END].Size() +
//     	Lines(line)->l_jct[END][LEFT].Size() +
//     	Lines(line)->l_jct[END][RIGHT].Size() - 2*n_t[END] + n_t[START];

//   	int r = RandInt()%(n_s + n_e);
// 				
// 		if(r < n_s) 
// 			end = END;		// 0 .. n_s - 1
// 		else 
// 			end = START;	// n_s .. n_s + n_e - 1

// 		FollowEnd(line, end, line);		// TODO What is Follow End?
//   }
  
// 
//   // if type is an ellipse						/// TODO TODO TODO TODO Ellipsen sind nicht geranked!!!!! => daher hier grow-Equal!!!
//   if (type == Gestalt::ELLIPSE)
//   {
// 		int n_l = 1 + Ellipses(idx)->e_jct[LEFT].Size();	// E-Jcts LEFT
// 		int n_r = 1 + Ellipses(idx)->e_jct[RIGHT].Size();	// E-Jcts RIGHT
// 
// 		int r = RandInt()%(n_l + n_r);
// 		unsigned end;
// 		if(r < n_l)  // 0 .. n_l - 1
// 			end = LEFT;
// 		else         // n_l .. n_l + n_r - 1
// 			end = RIGHT;
// 		ExtendEllipseEnd(idx, end);	  
//   }	
}

/**
 * @brief Extends the tangential search lines of a line.
 * @param line Gestalt Line
 */
void FormEJunctions::ExtendSearchLines(Line *line)
{
  if(vote_img->ExtendLine(line->ID()*8 + VOTE_TS, iscts) > 0)
    CreateJunctions(line->ID()*8 + VOTE_TS, iscts);
  if(vote_img->ExtendLine(line->ID()*8 + VOTE_TE, iscts) > 0)
    CreateJunctions(line->ID()*8 + VOTE_TE, iscts);
}

/**
 * @brief Extends a normal search line of a ellipse.
 * @param line Gestalt Line
 */
void FormEJunctions::ExtendSearchLines(Ellipse *ell)
{
	unsigned min = (baseOffset + ell->ID() -1)*8 + VOTE_EOTL;
	unsigned max = (baseOffset + ell->ID() -1)*8 + VOTE_EITR;
	for(unsigned sline = min; sline<= max; sline++)
		if(vote_img->ExtendLine(sline, iscts) > 0)
			CreateJunctions(sline, iscts);
}

/**
 * @brief Create junction between line and ellipse search lines.
 * @param sline Is the search line which triggered the new intersections.
 * @param iscts ???
 */
void FormEJunctions::CreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts)
{
  unsigned i = sline/baseIndex;					// line number
  unsigned vtype_i = sline%baseIndex;		// type of vote line

	// sline type is a line search line
	if(vtype_i == VOTE_E || vtype_i == VOTE_TS || vtype_i == VOTE_TE)
	{
// printf("FormEJunctions::CreateJunctions: VOTE_E / VOTE_TS / VOTE_TE!\n");
		Line *line_i = Lines(core, i);
		int end_i = VOTE_END(vtype_i);

		for(unsigned k = 0; k < iscts.Size(); k++)
		{
			unsigned j = iscts[k].id/baseIndex;
			unsigned vtype_j = iscts[k].id%baseIndex;
			int end_j = VOTE_VERTEX(vtype_j);

			if(IsctTypeAdmissible(vtype_i, vtype_j))
			{
				Ellipse *ell_j = Ellipses(core, j-baseOffset);

				if(NoEJunctionYet(line_i, ell_j))
				{
					// recalculate line_end, if vote line was edge itself
					if(vtype_i == VOTE_E) 
					{
						// intersection point und start/end point
						Vector2 isct;
						Vector2 start = line_i->point[START];
						Vector2 end = line_i->point[END];
						try
						{
							isct = LineIntersection(line_i->point[end_i], line_i->tang[end_i], ell_j->vertex[end_j], ell_j->dir);
						}
						catch (Except &e)
						{
							printf("FormEJunctions::CreateJunctions: Lines do not intersect exception.\n");
						}
						
						if(Length(start-isct) < Length(end-isct)) end_j = 0;
						else end_j = 1;
					}
					
					
					core->NewGestalt(GestaltPrinciple::FORM_E_JUNCTIONS, new EJunction(core, line_i, ell_j, end_i, end_j));
					
				}
			}
		}
	}
	
	// sline type is a ellipse search line
	else if(vtype_i == VOTE_EOTL || vtype_i == VOTE_EOTR || vtype_i == VOTE_EITL || vtype_i == VOTE_EITR)							/// TODO TODO TODO Fehler? Segmentation fault?
	{
// printf("FormEJunctions::CreateJunctions: VOTE_EOTL / VOTE_EOTR / VOTE_EITL / VOTE_EITR!\n");
		int end_i = VOTE_VERTEX(vtype_i);
		Ellipse *ell_i = Ellipses(core, i-baseOffset);

// printf("Ellipse: %u: vote: %u\n", ell_i->ID(), vtype_i);
		
		for(unsigned k = 0; k < iscts.Size(); k++)
		{
			unsigned j = iscts[k].id/baseIndex;
			unsigned vtype_j = iscts[k].id%baseIndex;
			int end_j = VOTE_END(vtype_j);

			if(IsctTypeAdmissible(vtype_i, vtype_j))
			{
				Line *line_j = Lines(core, j);
				if(NoEJunctionYet(line_j, ell_i))
				{
					// recalculate line_end, if vote line was edge itself
					// printf("  line vote: %u\n", vtype_j);

					if(vtype_j == VOTE_E) 
					{
						Vector2 isct;
						Vector2 start = line_j->point[START];
						Vector2 end = line_j->point[END];
						try
						{
							isct = LineIntersection(line_j->point[end_j], line_j->tang[end_j], ell_i->vertex[end_i], ell_i->dir);
						}
						catch (Except &e)
						{
							printf("FormEJunctions::CreateJunctions: Lines do not intersect exception.\n");
						}
// printf("    lengt s: %4.3f	e: %4.3f\n", Length(start-isct), Length(end-isct));
						if(Length(start-isct) < Length(end-isct)) end_j = 0;
						else end_j = 1;
					}
// printf("  end: %u\n", end_i);
// if(end_j != changed) printf("  LINE END CHANGED\n");
					core->NewGestalt(GestaltPrinciple::FORM_E_JUNCTIONS, new EJunction(core, line_j, ell_i, end_j, end_i));
				}
			}
		}
	}

	// sline type is a ellipse search line
	else if(vtype_i == VOTE_NONE)
	{
		printf("FormJunctions::CreateJunctions: VOTE_NONE exception!\n");
// 		int end_i = VOTE_VERTEX(vtype_i);
// 		Ellipse *ell_i = Ellipses(core, i-baseOffset);
// 		
// 		for(unsigned k = 0; k < iscts.Size(); k++)
// 		{
// 			unsigned j = iscts[k].id/baseIndex;
// 			unsigned vtype_j = iscts[k].id%baseIndex;
// 			int end_j = VOTE_END(vtype_j);
// 
// 			if(IsctTypeAdmissible(vtype_i, vtype_j))
// 			{
// 				Line *line_j = Lines(core, j);
// 
// 				if(NoEJunctionYet(line_j, ell_i))
// 					core->NewGestalt(GestaltPrinciple::FORM_E_JUNCTIONS, new EJunction(core, line_j, ell_i, end_j, end_i));
// 			}
// 		}
	}
	else
		printf("FormEJunctions::CreateJunctions: Warning: Unknown search line type.\n");
}


/// TODO TODO TODO ONLY for debugging => delete later!
void FormEJunctions::PrintCreateJunctions(unsigned sline, Array<VoteImage::Elem> &iscts)
{
	printf("\nFormEJunctions::PrintCreateJunctions:\n");
	unsigned i = sline/8;					// line number
  unsigned vtype_i = sline%8;		// type of vote line

	printf("  id: %u vtype: %u\n", i, vtype_i);
	
	for(unsigned k = 0; k < iscts.Size(); k++)
	{
		unsigned j = iscts[k].id/8;
		unsigned vtype_j = iscts[k].id%8;
		int end_j = VOTE_VERTEX(vtype_j);
		
		printf("  iscts: %u - type: %u\n", j, vtype_j);
	}

}

/**
 * @brief Check whether there is no E-jct with line and ellipse.
 * @param line Line
 * @param ellipse Ellipse
 */
bool FormEJunctions::NoEJunctionYet(Line *line, Ellipse *ellipse)				/// TODO INEFFIZIENT: Ganzes Feld muss immer durchsucht werden!!! (eintrag in ellipsen?)
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		if(EJunctions(core, i)->ellipse->ID() == ellipse->ID() &&
			 EJunctions(core, i)->line->ID() == line->ID())
				return false;
	}
//  for(unsigned i = 0; i < l->e_jct[end].Size(); i++)
//    if(EJunctions(l->e_jct[end][i])->ellipse == ellipse)
//      return false;
	
//   // one line at one end (not allowed at both sides)
//   for(unsigned i = 0; i < line->e_jct[START].Size(); i++)
//     if(EJunctions(line->e_jct[START][i])->ellipse == ellipse)
//       return false;
//   for(unsigned i = 0; i < l->e_jct[END].Size(); i++)
//     if(EJunctions(line->e_jct[END][i])->ellipse == ellipse)
//       return false;
	
  return true;
}

/**
 * @brief Update all ellipse-junctions, when a new collinearity appears.
 * @param idx Index of new collinearity.
 */
void FormEJunctions::UpdateEJunctions(unsigned idx)
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		if(EJunctions(core, i)->IsColLine(Collinearities(core, idx)->line[0]) ||																	/// TODO Kann nicht funktionieren => Falsche Linie! (IsLine)
			 EJunctions(core, i)->IsColLine(Collinearities(core, idx)->line[1]))
			EJunctions(core, i)->UpdateColLines(EJunctions(core, i)->line, EJunctions(core, i)->lineEnd);
	}
}

/**
 * @brief Mask the bad results.
 * Mask E-Junctions, where the ellipse is masked.
 */
void FormEJunctions::Mask()
{
	for(unsigned i=0; i<NumEJunctions(core); i++)
	{
		EJunction *ejct = (EJunction*)core->RankedGestalts(Gestalt::E_JUNCTION, i);
		if(ejct->ellipse->IsMasked())
			ejct->Mask(10000);
	}
}


}

