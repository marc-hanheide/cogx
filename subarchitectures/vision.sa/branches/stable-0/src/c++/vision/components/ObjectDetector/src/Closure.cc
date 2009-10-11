/**
 * $Id: Closure.cc,v 1.10 2007/04/14 20:50:59 mxz Exp mxz $
 */

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "TJunction.hh"
#include "Closure.hh"
#include "HCF.hh"

namespace Z
{

/**
 * HACK:
 * Comparison function for sorting inner angles of a closure, smallest to
 * largest.
 */
static int CmpAngles(const void *a, const void *b)
{
  // if a is undefined, move it to end
  if(*(unsigned*)a == UNDEF_ID)
    return 1;   // b is first
  // if b is undefined, move it to end
  else if(*(unsigned*)b == UNDEF_ID)
    return -1;  // a is first
  // both are defined, check their angles, move larger to end
  else if( LJunctions(*(unsigned*)a)->OpeningAngle() <
           LJunctions(*(unsigned*)b)->OpeningAngle() )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

Closure::Closure()
: Gestalt(CLOSURE)
{
  stability = 0.;
  energy = 0;
  label = DefaultDepth();
}

Closure::Closure(Array<unsigned> l, Array<unsigned> lj, Array<unsigned> c,
	Array<unsigned> s) : Gestalt(CLOSURE)
{
  lines = l;
  jcts = lj;
  colls = c;
  senses = s;

  // enter closur into lines
  for(unsigned i=0; i<lines.Size(); i++)
    Lines(lines[i])->closures.PushBack(id);												// TODO: passend einfügen
	
  stability = 0.;
  energy = 0;
  label = DefaultDepth();
	
  CalculateSignificance();
  Rank();
}

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

double Closure::GetJunctionSig(unsigned i)
{
  if(jcts[i] != UNDEF_ID && colls[i] != UNDEF_ID)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(jcts[i] != UNDEF_ID)
    return LJunctions(jcts[i])->sig;
  else if(colls[i] != UNDEF_ID)
    return Collinearities(colls[i])->sig;
  else
    throw Except(__HERE__, "need one of L-jct or collinearity");
}

void Closure::Draw(int detail)
{
  if(detail == 0)
  {
    // note: there are as many lines as junctions
    for(unsigned i = 0; i < lines.Size(); i++)
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
  }
  if(detail >= 1 && detail <= 2)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
      Lines(lines[i])->Draw(detail - 1);
  }
  if(detail >= 3)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
    {
      if(jcts[i] != UNDEF_ID)
        LJunctions(jcts[i])->Draw(detail - 3);
      else if(colls[i] != UNDEF_ID)
        Collinearities(colls[i])->Draw(detail - 3);
    }
  }
}

extern void PrintStabilityCommitted(int i);  // HACK

const char* Closure::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;

  n += snprintf(info_text + n, info_size - n, "%s%u lines:",
      Gestalt::GetInfo(), lines.Size());
  for(unsigned i = 0; i < lines.Size(); i++)
    n += snprintf(info_text + n, info_size - n, " %c%u",
      (senses[i] == SAME ? '+' : '-'), lines[i]);
  n += snprintf(info_text + n, info_size - n, "\n");

  for(unsigned i = 0; i < lines.Size(); i++)
    if(jcts[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " L(%d)", jcts[i]);
    else if(colls[i] != UNDEF_ID)
      n += snprintf(info_text + n, info_size - n, " C(%d)", colls[i]);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "neighbors:");
  for(set<unsigned>::iterator it = neighbors.begin();
      it != neighbors.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d", *it);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "aside:");
  for(map<unsigned, double>::iterator it = neighbors_aside.begin();
      it != neighbors_aside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first,
        it->second);
  n += snprintf(info_text + n, info_size - n, "  above:");
  for(map<unsigned, double>::iterator it = neighbors_above.begin();
      it != neighbors_above.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first,
        it->second);
  n += snprintf(info_text + n, info_size - n, "  below:");
  for(map<unsigned, double>::iterator it = neighbors_below.begin();
      it != neighbors_below.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first,
        it->second);
  n += snprintf(info_text + n, info_size - n, "  inside:");
  for(map<unsigned, double>::iterator it = neighbors_inside.begin();
      it != neighbors_inside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first,
        it->second);
  n += snprintf(info_text + n, info_size - n, "  outside:");
  for(map<unsigned, double>::iterator it = neighbors_outside.begin();
      it != neighbors_outside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first,
        it->second);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n,
      "label: %d energy: %f stability: %f\n", label, energy, stability);
  // HACK
  /*Array<unsigned> ordered_ljcts(jcts);
  double sum_angles = 0.;
  ordered_ljcts.Sort(CmpAngles);
  for(unsigned i = 0; i < ordered_ljcts.Size(); i++)
  {
    if(ordered_ljcts[i] != UNDEF_ID)
    {
      sum_angles += LJunctions(ordered_ljcts[i])->OpeningAngle();
      printf("%.3f  sum: %.3f\n", LJunctions(ordered_ljcts[i])->OpeningAngle(),
          sum_angles);
    }
  }
  printf("\n");*/
  // HACK END
  //PrintStabilityCommitted(id);  // HACK
  return info_text;
}

bool Closure::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    if(Lines(lines[i])->IsAtPosition(x, y))
      return true;
  return false;
}

void Closure::CalculateSignificance()
{
  /*double l = SumLines();
  double g = SumGaps();
  sig = l/(g+l);*/
  sig = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    sig += Lines(lines[i])->sig;
    sig += GetJunctionSig(i);
  }
}

unsigned Closure::NumLJunctions()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < jcts.Size(); i++)
    if(jcts[i] != UNDEF_ID)
      cnt++;
  return cnt;
}

unsigned Closure::NumCollinearities()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < colls.Size(); i++)
    if(colls[i] != UNDEF_ID)
      cnt++;
  return cnt;
}

double Closure::SumGaps()
{
  double sum = 0.;
  // note: there are as many lines as junctions
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    if(jcts[i] != UNDEF_ID)
      sum += LJunctions(jcts[i])->r;  // TODO: also use corrected distance here
    else if(colls[i] != UNDEF_ID)
      sum += Collinearities(colls[i])->gap_cor;
  }
  return sum;
}

double Closure::SumLines()
{
  double sum = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
    sum += Lines(lines[i])->Length();
  return sum;
}

double Closure::Circumference()
{
  double circ = 0.;
  // note: there are as many lines as junctions
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    unsigned j = (i < lines.Size()-1 ? i+1 : 0);
    Vector2 p = GetVertex(i);
    Vector2 q = GetVertex(j);
    circ += Distance(p, q);
  }
  return circ;
}

void Closure::FindTNeighbors()
{
  for(unsigned l = 0; l < lines.Size(); l++)
    for(int end = START; end <= END; end++)
    {
      unsigned t = Lines(lines[l])->t_jct[end];
      // if line l is the pole of a T-junction
      if(t != UNDEF_ID)
      {
        unsigned left = TJunctions(t)->line[LEFT];
        unsigned right = TJunctions(t)->line[RIGHT];
        // closures must share some contour line (left or right bar of T-jct)
        //if(left == lines.CircularNext(l) ||
        //   left == lines.CircularPrev(l) ||
        //   right == lines.CircularNext(l) ||
        //   right == lines.CircularPrev(l))
        if(lines.Contains(left) || lines.Contains(right))
        {
          // all closures j of which line 'left' and 'right' are part, occlude
          // this closur
          for(unsigned n = 0; n < Lines(left)->closures.Size(); n++)
          {
            unsigned j = Lines(left)->closures[n];
            // closures is not its own neighbour
            if(j != id)
            {
              unsigned pos_l = Closures(j)->lines.Find(left);
              unsigned next = Closures(j)->lines.CircularNext(pos_l);
              unsigned prev = Closures(j)->lines.CircularPrev(pos_l);
              // if right line of T-jct is the next line of closure j
              // we have a T-jct from outside, and j genuinely occludes this.
              if(right == Closures(j)->lines[next])
              {
                neighbors_above[j] += TJunctions(t)->sig;
                Closures(j)->neighbors_below[id] += TJunctions(t)->sig;
              }
              // if right line of T-jct is the previous line of closure j
              // we have a T-jct from inside and this lies "inside" j.
              else if(right == Closures(j)->lines[prev])
              {
                neighbors_outside[j] += TJunctions(t)->sig;
                Closures(j)->neighbors_inside[id] += TJunctions(t)->sig;
              }
            }
          }
        }
      }
    }
}

/**
 * Checks whether point p is inside closure.
 * Uses the Jordan curve theorem.
 * Note that points on the boundary are undefined.
 * Code thanks to
 *  W Randolph Franklin (WRF)
 *  http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 *  int pnpoly(int npol, float *xp, float *yp, float x, float y)
 *  {
 *    int i, j, c = 0;
 *    for (i = 0, j = npol-1; i < npol; j = i++) {
 *      if ((((yp[i]<=y) && (y<yp[j])) ||
 *           ((yp[j]<=y) && (y<yp[i]))) &&
 *          (x < (xp[j] - xp[i]) * (y - yp[i]) / (yp[j] - yp[i]) + xp[i]))
 *
 *        c = !c;
 *    }
 *    return c;
 *  } 
 */
bool Closure::Inside(Vector2 p)
{
  int i, j, npol = lines.Size();
  bool c = false;
  Vector2 pi, pj;
  for(i = 0, j = npol-1; i < npol; j = i++)
  {
    pi = GetVertex(i);
    pj = GetVertex(j);
    if((((pi.y <= p.y) && (p.y < pj.y)) ||
        ((pj.y <= p.y) && (p.y < pi.y))) &&
       (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x))

      c = !c;
  }
  return c;
}

/*
double Rectangle::SumGaps()
{
  double sum_gaps = 0.;
  for(unsigned i = 0; i < 4; i++)
  {
    sum_gaps += LJunctions(jcts[i])->Gap(LEFT) +
      LJunctions(jcts[i])->Gap(RIGHT);
  }
  return sum_gaps;
}

double Rectangle::Circumference()
{
  double sum = 0.;
  for(unsigned i = 0; i < 4; i++)
  {
    unsigned j = (i < 3 ? i + 1 : 0);
    sum += Length(LJunctions(jcts[j])->isct -
		  LJunctions(jcts[i])->isct);
  }
  return sum;
}

double Rectangle::Area()
{
  double area = 0.;
  Vector2 p1, p2;
  unsigned i, j;
  for(i = 0; i < 4; i++)
  {
    j = (i < 3 ? i + 1 : 0);
    p1 = LJunctions(jcts[i])->isct;
    p2 = LJunctions(jcts[j])->isct;
    area += p1.x*p2.y - p1.y*p2.x;
  }
  return area/2.;
}

// junction i has line i as left, line i+1 as right arm
Rectangle::Rectangle(unsigned ls[4], unsigned js[4])
  : Gestalt(RECTANGLE)
{
  unsigned i;
  for(i = 0; i < 4; i++)
    lines[i] = ls[i];
  for(i = 0; i < 4; i++)
  {
    jcts[i] = js[i];
    if(LJunctions(jcts[i])->near_point[LEFT] == START)
      sides[i] = RIGHT;
    else
      sides[i] = LEFT;
  }
  CalculateSignificance();
}

*/

}
