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
// HACK: this belongs to HCF2.cc, which is at the moment not built
static int min_label = -10;  // HACK: arbitrary number of labels
static int max_label = +10;
static const int UNCOMMITTED = INT_MAX;
static const int DEFAULT_DEPTH = 0;
const int UNDEF_DEPTH = UNCOMMITTED;
const int MASKED = max_label;  // maximum depth
// HACK END

int MinDepth()
{
  return min_label;
}

int MaxDepth()
{
  return max_label;
}

int DefaultDepth()
{
  return DEFAULT_DEPTH;
}

int NumLabels()
{
  return max_label - min_label + 1;
}

Closure::Closure(VisionCore *c)
: Gestalt(c, CLOSURE)
{
  stability = 0.;
  energy = 0;;
  label = DefaultDepth();
}

Vector2 Closure::GetVertex(unsigned i)
{
  if(jcts[i] != 0 && colls[i] != 0)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(jcts[i] != 0)
    return jcts[i]->isct;
  else if(colls[i] != 0)
    return colls[i]->vertex;
  else
    throw Except(__HERE__, "need one of L-jct or collinearity");
}

double Closure::GetJunctionSig(unsigned i)
{
  if(jcts[i] != 0 && colls[i] != 0)
    throw Except(__HERE__, "need either L-jct or collinearity");
  else if(jcts[i] != 0)
    return jcts[i]->sig;
  else if(colls[i] != 0)
    return colls[i]->sig;
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
        p = lines[i]->point[START];
        q = lines[i]->point[END];
      }
      else
      {
        p = lines[i]->point[END];
        q = lines[i]->point[START];
      }
      DrawLine2D(p.x, p.y, q.x, q.y, RGBColor::red);
      r = GetVertex(j);
      DrawLine2D(q.x, q.y, r.x, r.y, RGBColor::red);
      if(senses[j] == SAME)
        s = lines[j]->point[START];
      else
        s = lines[j]->point[END];
      DrawLine2D(r.x, r.y, s.x, s.y, RGBColor::red);
    }
  }
  if(detail >= 1 && detail <= 2)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
      lines[i]->Draw(detail - 1);
  }
  if(detail >= 3)
  {
    for(unsigned i = 0; i < lines.Size(); i++)
    {
      if(jcts[i] != 0)
        jcts[i]->Draw(detail - 3);
      else if(colls[i] != 0)
        colls[i]->Draw(detail - 3);
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
      (senses[i] == SAME ? '+' : '-'), lines[i]->ID());
  n += snprintf(info_text + n, info_size - n, "\n");

  for(unsigned i = 0; i < lines.Size(); i++)
    if(jcts[i] != 0)
      n += snprintf(info_text + n, info_size - n, " L(%d)", jcts[i]->ID());
    else if(colls[i] != 0)
      n += snprintf(info_text + n, info_size - n, " C(%d)", colls[i]->ID());
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "neighbors:");
  for(set<Closure*>::iterator it = neighbors.begin();
      it != neighbors.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d", (*it)->ID());
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n, "aside:");
  for(map<Closure*, double>::iterator it = neighbors_aside.begin();
      it != neighbors_aside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  above:");
  for(map<Closure*, double>::iterator it = neighbors_above.begin();
      it != neighbors_above.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  below:");
  for(map<Closure*, double>::iterator it = neighbors_below.begin();
      it != neighbors_below.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  inside:");
  for(map<Closure*, double>::iterator it = neighbors_inside.begin();
      it != neighbors_inside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "  outside:");
  for(map<Closure*, double>::iterator it = neighbors_outside.begin();
      it != neighbors_outside.end(); ++it)
    n += snprintf(info_text + n, info_size - n, " %d/%.2f", it->first->ID(),
        it->second);
  n += snprintf(info_text + n, info_size - n, "\n");

  n += snprintf(info_text + n, info_size - n,
      "label: %d energy: %f stability: %f\n", label, energy, stability);
  //PrintStabilityCommitted(id);  // HACK
  return info_text;
}

bool Closure::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    if(lines[i]->IsAtPosition(x, y))
      return true;
  return false;
}

void Closure::CalculateSignificance()
{
  /*double l = SumLines();
  double g = SumGaps();
  sig = l/(g+l);*/
  
  /*sig = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    sig += lines[i]->sig;
    sig += GetJunctionSig(i);
  }*/
  sig = Area();
}

unsigned Closure::NumLJunctions()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < jcts.Size(); i++)
    if(jcts[i] != 0)
      cnt++;
  return cnt;
}

unsigned Closure::NumCollinearities()
{
  unsigned cnt = 0;
  for(unsigned i = 0; i < colls.Size(); i++)
    if(colls[i] != 0)
      cnt++;
  return cnt;
}

double Closure::SumGaps()
{
  double sum = 0.;
  // note: there are as many lines as junctions
  for(unsigned i = 0; i < lines.Size(); i++)
  {
    if(jcts[i] != 0)
      sum += jcts[i]->r;  // TODO: also use corrected distance here
    else if(colls[i] != 0)
      sum += colls[i]->gap_cor;
  }
  return sum;
}

double Closure::SumLines()
{
  double sum = 0.;
  for(unsigned i = 0; i < lines.Size(); i++)
    sum += lines[i]->Length();
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
      TJunction *t = lines[l]->t_jct[end];
      // if line l is the pole of a T-junction
      if(t != 0)
      {
        Line *left = t->line[LEFT];
        Line *right = t->line[RIGHT];
        // closures must share some contour line (left or right bar of T-jct)
        //if(left == lines.CircularNext(l) ||
        //   left == lines.CircularPrev(l) ||
        //   right == lines.CircularNext(l) ||
        //   right == lines.CircularPrev(l))
        if(lines.Contains(left) || lines.Contains(right))
        {
          // all closures j of which line 'left' and 'right' are part, occlude
          // this closur
          for(unsigned n = 0; n < left->closures.Size(); n++)
          {
            Closure *j = left->closures[n];
            // closures is not its own neighbour
            if(j != this)
            {
              unsigned pos_l = j->lines.Find(left);
              unsigned next = j->lines.CircularNext(pos_l);
              unsigned prev = j->lines.CircularPrev(pos_l);
              // if right line of T-jct is the next line of closure j
              // we have a T-jct from outside, and j genuinely occludes this.
              if(right == j->lines[next])
              {
                neighbors_above[j] += t->sig;
                j->neighbors_below[this] += t->sig;
              }
              // if right line of T-jct is the previous line of closure j
              // we have a T-jct from inside and this lies "inside" j.
              else if(right == j->lines[prev])
              {
                neighbors_outside[j] += t->sig;
                j->neighbors_inside[this] += t->sig;
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

double Closure::Area()
{
  double area = 0.;
  Vector2 p1, p2;
  unsigned i, j;
  for(i = 0; i < jcts.Size(); i++)
  {
    j = jcts.CircularNext(i);
    p1 = jcts[i] != 0 ? jcts[i]->isct : colls[i]->vertex;
    p2 = jcts[j] != 0 ? jcts[j]->isct : colls[j]->vertex;
    area += p1.x*p2.y - p1.y*p2.x;
  }
  return area/2.;
}

/*
double Rectangle::SumGaps()
{
  double sum_gaps = 0.;
  for(unsigned i = 0; i < 4; i++)
  {
    sum_gaps += jcts[i]->Gap(LEFT) + jcts[i]->Gap(RIGHT);
  }
  return sum_gaps;
}

double Rectangle::Circumference()
{
  double sum = 0.;
  for(unsigned i = 0; i < 4; i++)
  {
    unsigned j = (i < 3 ? i + 1 : 0);
    sum += Length(jcts[j]->isct - jcts[i]->isct);
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
    p1 = jcts[i]->isct;
    p2 = jcts[j]->isct;
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
    if(jcts[i]->near_point[LEFT] == START)
      sides[i] = RIGHT;
    else
      sides[i] = LEFT;
  }
  CalculateSignificance();
}

*/

}

