/**
 * $Id: FormRectangles.cc,v 1.16 2007/02/04 23:53:03 mxz Exp mxz $
 */

#include <stdio.h>
#include "LJunction.hh"
#include "Rectangle.hh"
#include "Line.hh"
#include "FormRectangles.hh"

namespace Z
{

static int CmpRectangles(const void *a, const void *b)
{
  if( Rectangles(*(unsigned*)a)->sig > Rectangles(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormRectangles::FormRectangles(Config *cfg)
: GestaltPrinciple(cfg)
{
}

void FormRectangles::Rank()
{
  // TODO: homogenous rectangles are better, smaller sum of gaps is better
  RankGestalts(Gestalt::RECTANGLE, CmpRectangles);
}

void FormRectangles::Mask()
{
/*  bool survive;
  unsigned i, j, l;
  Array<unsigned> used_lines[2]; // which rect used which line left/right
  used_lines[LEFT].Resize(NumLines());
  used_lines[LEFT].Set(UNDEF_ID);
  used_lines[RIGHT].Resize(NumLines());
  used_lines[RIGHT].Set(UNDEF_ID);
  for(j = 0; j < NumRectangles(); j++)
  {
    i = RankedGestalts(Gestalt::RECTANGLE, j);
    Rectangle *r = Rectangles(i);
    survive = true;
    // first check if any previous (i.e. stronger) rect uses one of my lines
    for(l = 0; l < 4; l++)
    {
      if(used_lines[r->sides[l]][r->lines[l]] != UNDEF_ID)
      {
        survive = false;
        r->Mask(used_lines[r->sides[l]][r->lines[l]]);
      }
    }
    if(survive)
    {
      for(l = 0; l < 4; l++)
        used_lines[r->sides[l]][r->lines[l]] = i;
    }
  }*/
}

void FormRectangles::Reset()
{
  ClearHashTable();
  for(unsigned i = 0; i < u_left.Size(); i++)
    u_left[i].Clear();
  u_left.Clear();
}

void FormRectangles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  switch(type)
  {
    case Gestalt::L_JUNCTION:
      HaveNewLJunction(idx);
      break;
    default:
      break;
  }
}

void FormRectangles::HaveNewLJunction(unsigned idx)
{
  printf("%s\n", __func__);
}

/*void FormRectangles::StepOperate()
{
  unsigned side, i, j, k;
  unsigned lines[4], jcts[4];
  unsigned left_arm, right_arm, left_jct, right_jct, cand;
  Line *l;

  u_left.Resize(NumLines());
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
                NewGestalt(new Rectangle(lines, jcts));
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
}*/

/**
 * Hash function.
 * The standard reference from Knuth's "The Art of Computer Programming",
 * volume 3 "Sorting and Searching", chapter 6.4.
 */
unsigned FormRectangles::Hash(unsigned lines[4])
{
  // first find one distinct arc in the group: maximum arc id
  unsigned i, i_max = 0, max = lines[0], hash = 4;
  for(i = 1; i < 4; i++)
    if(lines[i] > max)
    {
      i_max = i;
      max = lines[i];
    }
  for(i = i_max; i < 4; i++) 
    hash = ((hash<<5)^(hash>>27))^lines[i];
  for(i = 0; i < i_max; i++) 
    hash = ((hash<<5)^(hash>>27))^lines[i];
  hash = hash % HASH_SIZE;
  return hash;
}

void FormRectangles::ClearHashTable()
{
  for(int i = 0; i < HASH_SIZE; i++)
    hashtable[i] = 0;
}

/**
 * Check whether the set of junctions form a convex polygon.
 * If the cross products of vectors i-j and j-k for all i, j=i+1, k=j+1 has the
 * same sign, the polygon is convex.
 */
bool FormRectangles::IsConvexPolygon(unsigned jcts[4])
{
  Vector2 p[4], a, b;
  int i, j, k;
  int sig = 0;

  for(i = 0; i < 4; i++)
    p[i] = LJunctions(jcts[i])->isct;
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

}

