/**
 * @file FormRectangle.cc
 * @author Richtsfeld Andreas
 * @date October 2009
 * @version 0.1
 * @brief Gestalt principle class for forming rectangles.
 **/

#include <stdio.h>
#include "Rectangle.hh"
#include "Closure.hh"
#include "Line.hh"
#include "FormRectangles.hh"

namespace Z
{

static int CmpRectangles(const void *a, const void *b)
{
  if( (*(Rectangle**)a)->sig > (*(Rectangle**)b)->sig )																														/// TODO Change to Rectangles, if finished
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

bool FormRectangles::NeedsOperate()
{
  return false;
}

/**
 * @brief Constructor of class FormRectangles
 * @param vc Vision core
 */
FormRectangles::FormRectangles(VisionCore *vc) : GestaltPrinciple(vc)
{
}

/** TODO ARI: Diese Funktion funktioniert nicht? => wieso????
 * @brief Comparison function for sorting inner angles of a closure, smallerst to largest.
 * @param a ???
 * @param b ???
 */
static int CmpAngles(const void *a, const void *b)
{
// printf("CmpAngles:\n");
//   // if a is undefined, move it to end
// 
// printf("%4.3f - %4.3f\n", (*(LJunction**)a)->sig, 1.111);


  // if a is undefined, move it to end
  if(*(LJunction**)a == 0)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(LJunction**)b == 0)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( (*(LJunction**)a)->OpeningAngle() <
           (*(LJunction**)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Rank Rectangles
 */
void FormRectangles::Rank()
{
  // TODO ARI: homogenous rectangles are better, smaller sum of gaps is better
  RankGestalts(Gestalt::RECTANGLE, CmpRectangles);
}


/**
 * @brief Making Rectangles
 */
void FormRectangles::Mask()
{
  for(unsigned i=0; i<NumRectangles(core); i++)
  {
		for(unsigned j=0; j<NumRectangles(core); j++)
		{
			if(!Rectangles(core, i)->IsMasked() && !Rectangles(core, j)->IsMasked())
				if(Rectangles(core, i)->sig < Rectangles(core, j)->sig)
					if(Rectangles(core, i)->IsInside(j))
						Rectangles(core, i)->Mask(j);	
		}
  }
}

/*
void FormRectangles::Reset(const Image *img)
{
  ClearHashTable();
  for(unsigned i = 0; i < u_left.Size(); i++)
    u_left[i].Clear();
  u_left.Clear();
}
*/

/** 
 * @brief InformNewGestalt
 */
void FormRectangles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
//   StartRunTime();																																									/// TODO Reimplement Start/Stop RunTime() in Gestalts.
  if (type == Gestalt::CLOSURE)
		CreateQuadrilateral(idx);
	Rank();
	Mask();
//   StopRunTime();
}


/**
 * HACK: this is still a bit hacky, using thresholds etc.
 * TODO: for now we only check for quadrilaterals
 * TODO: use different criterion, enable jumping over e.g. two small 45deg
 * angles to get one 90deg angle. how much of the closure is still covered when
 * taking out such small lines in order to get rectangle?
 */
void FormRectangles::CreateQuadrilateral(unsigned clos)
{

  // if closure has 4 L-junctions, it is definitely a quadrilateral
  if(Closures(core, clos)->NumLJunctions() == 4)
  {
		unsigned ljcts[4];																// TODO ARI: Ändern auf Array, damit gleicher Aufruf wie unten
    for(unsigned i = 0, j = 0; i < Closures(core, clos)->jcts.Size(); i++)
		{
			LJunction *lj = Closures(core, clos)->jcts[i];
      if(Closures(core, clos)->jcts[i] != 0)					// ARI: Be carefull => Zero holes in jcts[]
        ljcts[j++] = lj->ID();

		}

		if (IsConvexPolygon(ljcts))
		{
			double parallelity = IsRectangle(ljcts);
			if(parallelity > 5.)														// TODO ARI: parallelity-threshold
			{
				if(!RectangleExists(ljcts))
				{
					Rectangle *new_r = new Rectangle(core, Closures(core, clos), ljcts, parallelity, Closures(core, clos)->NumLJunctions());
					core->NewGestalt(new_r);
				}
			}
		}
	}


  // if closure has more than 4 L-Junctions, it is a quadriliteral, if the
  // sum of the greatest 4 angles is 2Pi +- delta
  else if (Closures(core, clos)->NumLJunctions() > 4)
  {
    const double delta = M_PI/4.;  												// threshold 
    double sum_angles = 0.;

		// get ljcts
		Array<unsigned> ordered_ljcts;

		for(unsigned i=0; i<Closures(core, clos)->jcts.Size(); i++)
			if(Closures(core, clos)->jcts[i] != 0)			// "without holes ;-)"
				ordered_ljcts.PushBack(Closures(core, clos)->jcts[i]->ID());

		// check if closure is a convex polygon
		if (IsConvexPolygon(ordered_ljcts))
		{

//       ordered_ljcts.Sort(CmpAngles);		// sort by angles
			// TODO Search the biggest four ordered LJunctions (by hand, because sort-function don't want work.
			for(unsigned a=0; a<4; a++)
			{
				unsigned biggest = 0;
				double comp = 0.0;
				for(unsigned b=a; b<ordered_ljcts.Size(); b++)
				{
					// suche größten Winkel
					if(LJunctions(core, ordered_ljcts[b])->OpeningAngle() > comp)
					{
						comp = LJunctions(core, ordered_ljcts[b])->OpeningAngle();
						biggest = b;
					}
				}
				// change biggest with current value a
				unsigned sav = ordered_ljcts[a];
				ordered_ljcts[a] = ordered_ljcts[biggest];
				ordered_ljcts[biggest] = sav;
			}

			// calculate sum of the 4 greatest angles
  	  for(unsigned i = 0; i < 4; i++)
        if(ordered_ljcts[i] != 0)
          sum_angles += LJunctions(core, ordered_ljcts[i])->OpeningAngle();

	  	// sum of angles is within 2Pi +- delta
			if(fabs(2*M_PI - sum_angles) <= delta)
      {
        // note that junctions in ordered_ljcts are not in counter-clockwise order
        // -> go through original junction list and collect those which are the
        // first 4 in ordered_ljcts
        unsigned ljcts[4];
        for(unsigned i = 0, j = 0; i < Closures(core, clos)->jcts.Size() && j < 4; i++)		// ARI: Size bigger than NumLJunctions()
				{
          // if junction i is among the first 4
          // TODO: note that this is a bit inefficient..
          // TODO: sometimes j can become > 3 (hence the check in for(..)
					if(Closures(core, clos)->jcts[i] != 0)					// HACK ARI: Be carefull: Zero-Holes in jcts[]
            if(ordered_ljcts.Find(Closures(core, clos)->jcts[i]->ID()) < 4)
              ljcts[j++] = Closures(core, clos)->jcts[i]->ID();
        }

				double parallelity = IsRectangle(ljcts);
        if(parallelity > 5.)   													// TODO ARI: parallelity-threshold 
				{
					// reject if rectangle exists	
					if (!RectangleExists(ljcts))
					{
						Rectangle *new_r = new Rectangle(core, Closures(core, clos), ljcts, parallelity, Closures(core, clos)->NumLJunctions());
						core->NewGestalt(new_r);
					}
				}
      }
    }
  }
}

/**
 * @brief Check if this rectangle with the four l-junctions already exists.
 * @param ljcts The four L-junctions of the rectangle.
 */
bool FormRectangles::RectangleExists(unsigned ljcts[4])
{
  bool rectangleExists = false;
  if (NumRectangles(core)<1) rectangleExists = false;

  for (unsigned i=0; i<NumRectangles(core); i++)
  {
		unsigned rectJcts[4];
		for(int j=0; j<4; j++) rectJcts[j] = Rectangles(core, i)->jcts[j];
	
		// if all 4 jcts same => rectangle exists already
		bool found[4];
		found[0]=false;	
		found[1]=false;	
		found[2]=false;	
		found[3]=false;	
		
		for(int j=0; j<4; j++) 
			for(int k=0; k<4; k++)
				if(ljcts[j] == rectJcts[k]) found[j]=true;
	
		// found all 4 jcts at other rectangle? => rectangle exists
		if (found[0] && found[1] && found[2] && found[3]) rectangleExists = true;
  }
  return rectangleExists;
}


/**
 * @brief Check if quadrilateral is a rectangle. Calculate therefore the parallelity of opposing edges.
 * @param ljcts The four L-Junctions of the rectangle.
 */
double FormRectangles::IsRectangle(unsigned ljcts[4])
{
  Vector2 isct[4];			// intersections of L-Junctions						// TODO ARI: Ganz schön viel Berechnung
  Vector2 line[4];			// lines between intersections
  Vector2 dir[4];			// direction of lines
  double phi[4];			// angle of lines

  // get the 4 corners (intersections)
  for(int i=0; i<4; i++)
		isct[i]=LJunctions(core, ljcts[i])->isct;

  // calc lines, direction, angle
  for(int i=0; i<4; i++)
	{
		int j=i+1;
		if (j==4) j=0;
		line[i].x=isct[i].x-isct[j].x;
		line[i].y=isct[i].y-isct[j].y;
		if (line[i].y!=0.) dir[i] = Normalise(line[i]);
		phi[i]= ScaleAngle_0_2pi(PolarAngle(dir[i]));
	}

  // calculate difference of angles from opposed edges
  double diff[2];
  diff[0] = fabs(fabs(phi[0]-phi[2])-M_PI);
  diff[1] = fabs(fabs(phi[1]-phi[3])-M_PI);

 	// TODO ARI: check this calculation of parallelity (0-100)
  // calculate parallelity of best (min) opposing edge-pair (in degree)
  // parallelity = 10*e^(-diff/20°)
  double parallelity = 10.*exp(0.-((fmin(diff[0], diff[1]))*(180./M_PI)/20.));

  // calculate parallelity of worst (max) opposing edge-pair (in degree)
  // parallelity = 10*e^(-diff/20°)
  parallelity *= 10.*exp(0.-((fmax(diff[0], diff[1]))*(180./M_PI)/20.));

  return parallelity;
}	

/*
void FormRectangles::HaveNewLJunction(unsigned idx)
{
  printf("%s\n", __func__);
}
*/

/*
**	TODO ARI: Was macht diese Funktion???
**
*/
/*
void FormRectangles::StepOperate()
{
  unsigned side, i, j, k;
  unsigned lines[4], jcts[4];
  unsigned left_arm, right_arm, left_jct, right_jct, cand;
  Line *l;
  unsigned MAX_HYPS=1000;
	
  u_left.Resize(NumLines());
  for (i=0;i<NumLines(); i++)
  {
    l = Lines(i);
    for(side = LEFT; side <= RIGHT; side++)
      for(j = 0; j < min(MAX_HYPS,l->l_jct[START][side].Size()); j++)
      {
        for(k = 0; k < min(MAX_HYPS,l->l_jct[END][OtherSide(side)].Size()); k++)
        {
         // if START end of center line is the RIGHT arm of a junction the
          // line at the START end is the first line of the U.
          if(side == RIGHT)
          {
            left_jct = l->l_jct[START][RIGHT][j];
            right_jct = l->l_jct[END][LEFT][k];
          }
          // else (START end of center line is LEFT arm of a junction) the 
          // line at the END end is the first line of the U.
          else // side == LEFT
          {
            left_jct = l->l_jct[END][RIGHT][k];
            right_jct = l->l_jct[START][LEFT][j];
          }
          left_arm = LJunctions(left_jct)->line[LEFT];
          right_arm = LJunctions(right_jct)->line[RIGHT];
          u_left[left_arm].PushBack(RectCand(i, right_arm, left_jct,
                right_jct));
          cand = u_left[right_arm].Find(RectCand(left_arm));
          while(cand != UNDEF_ID)
          {
            lines[0] = left_arm;
            lines[1] = i;
            lines[2] = right_arm;
            lines[3] = u_left[right_arm][cand].base;
            jcts[0] = left_jct;
            jcts[1] = right_jct;
            jcts[2] = u_left[right_arm][cand].jct[LEFT];
            jcts[3] = u_left[right_arm][cand].jct[RIGHT];
            if(IsConvexPolygon(jcts))
            {
              unsigned hash = Hash(lines);
              if(hashtable[hash] == 0)
              {
                // TODO: this check is not complete, and what does it actually
                // do?
                //if(!u_left[lines[1]].Contains(RectCand(lines[3])))
				
				// TODO ARI: Rectangle is defined by closure, jcts[4]
                //NewGestalt(new Rectangle(lines, jcts));
printf("StepOperate: NewGestalt\n");
printf(" jcts: %i - %i - %i - %i\n", jcts[0], jcts[1], jcts[2], jcts[3]);
printf(" lins: %i - %i - %i - %i\n", lines[0], lines[1], lines[2], lines[3]);
                NewGestalt(new Rectangle(0, jcts));	// 0=nrOfClosure
//				nrOfClosure++;
                hashtable[hash] = 1;
              }
            }
            cand = u_left[right_arm].Find(cand + 1, RectCand(left_arm));
          }
        }
      }
  }
  Rank();
  Mask();
}
*/

/*
 * Hash function.
 * The standard reference from Knuth's "The Art of Computer Programming",
 * volume 3 "Sorting and Searching", chapter 6.4.
 */
// unsigned FormRectangles::Hash(unsigned lines[4])
// {
//   // first find one distinct arc in the group: maximum arc id
//   unsigned i, i_max = 0, max = lines[0], hash = 4;
//   for(i = 1; i < 4; i++)
//     if(lines[i] > max)
//     {
//       i_max = i;
//       max = lines[i];
//     }
//   for(i = i_max; i < 4; i++) 
//     hash = ((hash<<5)^(hash>>27))^lines[i];
//   for(i = 0; i < i_max; i++) 
//     hash = ((hash<<5)^(hash>>27))^lines[i];
//   hash = hash % HASH_SIZE;
//   return hash;
// }
// 
// void FormRectangles::ClearHashTable()
// {
//   for(int i = 0; i < HASH_SIZE; i++)
//     hashtable[i] = 0;
// }

/**
 * @brief Check whether the set of junctions form a convex polygon.
 * If the cross products of vectors i-j and j-k for all i, j=i+1, k=j+1 has the
 * same sign, the polygon is convex.
 * @param jcts The four junctions of the rectangle.
 */
bool FormRectangles::IsConvexPolygon(unsigned jcts[4])
{
  Vector2 p[4], a, b;
  int i, j, k;
  int sig = 0;

  for(i = 0; i < 4; i++)
    p[i] = LJunctions(core, jcts[i])->isct;
  for(i = 0; i < 4; i++)
  {
    j = (i < 3 ? i + 1 : 0);
    k = (j < 3 ? j + 1 : 0);
    a = p[j] - p[i];
    b = p[k] - p[j];
    if(sig == 0)
      sig = Sign(Cross(a, b));
    else
    {
      if(sig != Sign(Cross(a, b)))
      	return false;
    }
  }
  return true;
}

/**
 * @brief ARI: Check whether the set of L-Junctions form a convex polygon.
 * If the cross products of neighbor-vectors have all the same sign, the
 * polygon is convex.
 * @param jcts The L-junctions of the rectangle as array.
 */
bool FormRectangles::IsConvexPolygon(Array<unsigned> ljcts)
{
  Vector2 a, b;	
  int i, j, k;
  int sig = 0;	// sign of cross product
  int m = 0;	// number of found LJunctions
	
  Vector2 intscts[ljcts.Size()];		// maximum LJunctions->isct Array

  // get every junction and check if LJunction (could also be a coll.)
  for (unsigned l=0; l<ljcts.Size(); l++)
	{
		try
		{
			LJunctions(core, ljcts[l])->isct;
			intscts[m++] = LJunctions(core, ljcts[l])->isct;
		}
		catch (Except &e){}
  }	
	
  // copy intersections to array with size m
  Vector2 isct[m];
  for (int n=0; n<m; n++)
    isct[n] = intscts[n];
	
  // check the sign of the cross product between neighboring lines
  for(i=0; i<m; i++)
  {
    j = (i < (m-1) ? i + 1 : 0);
    k = (j < (m-1) ? j + 1 : 0);
    a = isct[j] - isct[i];
    b = isct[k] - isct[j];

    if(sig == 0)
      sig = Sign(Cross(a, b));
    else
    {
      if(sig != Sign(Cross(a, b)))
        return false;
    }
  }
  return true;
}

}
