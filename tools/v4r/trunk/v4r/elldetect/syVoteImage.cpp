//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Michael Zillich,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#include "syDebug.hpp"

#include <assert.h>
#include "syVoteImage.hpp"
#include "SMath.h"


NAMESPACE_CLASS_BEGIN( RTE )


////////////////////////////////////////////////////////////////////////////////
CzVoteImage::CzVoteImage(int w, int h)
{
   width = w;
   height = h;
   data = new CzElem*[width*height];
   if (data == 0)
      throw CzExcept(__HERE__, "Could not create instances of CzElem! Out of memory?");
   store_size = 2*width*height; // OHNO: this is an arbitrary choice
   store = new CzElem[store_size];
   if (store == 0)
      throw CzExcept(__HERE__, "Could not create instances of CzElem! Out of memory?");
   fill = 0;
}

////////////////////////////////////////////////////////////////////////////////
CzVoteImage::~CzVoteImage()
{
  delete[] data;
  delete[] store;
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::SetNumLines(unsigned n)
{
  lines.Resize(n);
  for(unsigned id = 0; id < lines.Size(); id++)
    lines[id].len = 0;
}

////////////////////////////////////////////////////////////////////////////////
CzVoteImage::CzElem *CzVoteImage::NewElem(unsigned id)
{
  if(fill < store_size)
  {
    CzElem *el = &store[fill++];
    el->id = id;
    el->next = 0;
    return el;
  }
  else
  {
    // OHNO: alloc more storage
    printf("Oh lala! out of store!\n");
    return 0;
  }
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::Clear()
{
  memset(data, 0, width*height*sizeof(CzElem*));
  fill = 0;
}

////////////////////////////////////////////////////////////////////////////////
/*inline*/ void CzVoteImage::SetPixel(int x, int y, unsigned id)
{
  CzElem *e = data[y*width + x];
  if(e == 0)
    data[y*width + x] = NewElem(id);
  else
  {
    while(e->next != 0)
      e = e->next;
    e->next = NewElem(id);
  }
}

////////////////////////////////////////////////////////////////////////////////
inline void CzVoteImage::SetAndCheckPixel(int x, int y, unsigned id, CzArray<CzElem> &iscts)
{
  CzElem *e = data[y*width + x];
  if (e == 0) {
    data[y*width + x] = NewElem(id);
  } else {
    CzElem *last = e;
    while (e != 0) {
      CreateIntersection(e, id, iscts);
      last = e;
      e = e->next;
    }
    if (last != 0)
       last->next = NewElem(id);
  }
}

////////////////////////////////////////////////////////////////////////////////
inline void CzVoteImage::CheckPixel(int x, int y, unsigned id, CzArray<CzElem> &iscts)
{
  CzElem *e = data[y*width + x];
  while (e != 0) {
    CreateIntersection(e, id, iscts);
    e = e->next;
  }
}

////////////////////////////////////////////////////////////////////////////////
// This CheckPixel function is called from FindLineEnd.
// Lines end only at other visible lines, not tangents or normals.
inline bool CzVoteImage::CheckPixel(int x, int y, unsigned id)
{
  CzElem *e = data[y*width + x];
  while (e != 0) {
    if (e->id%8 == VOTE_E)
      if (e->id != id || ((e->id%8==2 && id%8==3) || (id%8==2 && e->id%8==3))) //allow intersection of lines coming from same shape!
        return true;
    e = e->next;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
inline void CzVoteImage::CreateIntersection(CzElem *e, unsigned id, CzArray<CzElem> &iscts)
{
  // don't intersect edge with itself  OHNO: why not?
  if(e->id/8 != id/8 || ((e->id%8==2 && id%8==3) || (id%8==2 && e->id%8==3)))
    if(!iscts.ContainsBackwards(*e))
      iscts.PushBack(*e);
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::InitLine(int x1, int y1, int x2, int y2, unsigned id)
{
  CzLineStub &line = lines[id];
  line.x = line.y = 0;
  line.dx = line.dy = 0;
  line.inc_x = line.inc_y = 0;
  line.len = 0;
  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  line.dx = x2 - x1;
  line.dy = y2 - y1;
  if(line.dx == 0 && line.dy == 0)  // line might be clipped to length 0
    return;
  if(line.dx >= 0)
    line.inc_x = 1;
  else
  {
    line.dx = -line.dx;
    line.inc_x = -1;
  }
  if(line.dy >= 0)
    line.inc_y = 1;
  else
  {
    line.dy = -line.dy;
    line.inc_y = -1;
  }
  line.x = x1;
  line.y = y1;
  if(line.dx >= line.dy)
  {
    // first octant bresenham
    line.err = -line.dx/2;
    line.len = line.dx + 1;
  }
  else
  {
    // second octant bresenham
    line.err = -line.dy/2;
    line.len = line.dy + 1;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Extends the given (search) line.
// @param id  line to extend
// @param iscts  array contains created intersections after return
// Returns the number of created intersections or -1 if the line could not be
// further extended, i.e. its maximum length was reached.
int CzVoteImage::ExtendLine(unsigned id, CzArray<CzElem> &iscts)
{
  CzLineStub &line = lines[id];
  iscts.Clear();
  if(line.len > 0)
  {
    SetAndCheckPixel(line.x, line.y, id, iscts);
    line.len--;
    if(line.dx >= line.dy)
    {
      line.err += line.dy;
      if(line.err >= 0)
      {
        line.y += line.inc_y;
        line.err -= line.dx;
        if(line.len > 0)  // OHNO: this is always true except once
          // make line dense
          SetAndCheckPixel(line.x, line.y, id, iscts);
      }
      line.x += line.inc_x;
    }
    else
    {
      line.err += line.dx;
      if(line.err >= 0)
      {
        line.x += line.inc_x;
        line.err -= line.dy;
        if(line.len > 0)
          // make line dense
          SetAndCheckPixel(line.x, line.y, id, iscts);
      }
      line.y += line.inc_y;
    }
    return iscts.Size();
  }
  else
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::DrawLine(int x1, int y1, int x2, int y2, unsigned id)
{
  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      SetPixel(x, y, id);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          SetPixel(x, y, id);
      }
      x += inc_x;
    } while(x != x2); // OHNO: x2 is not coloured!
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      SetPixel(x, y, id);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetPixel(x, y, id);
      }
      y += inc_y;
    } while(y != y2);
  }
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::CheckLine(int x1, int y1, int x2, int y2, unsigned id, CzArray<CzElem> &iscts)
{
  int dx, dy, inc_x, inc_y, x, y, err;
  unsigned MAX_HYPS = 1000;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      CheckPixel(x, y, id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          CheckPixel(x, y, id, iscts);
      }
      x += inc_x;
    } while(x != x2 && iscts.Size() <= MAX_HYPS);
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      CheckPixel(x, y, id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          CheckPixel(x, y, id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}

////////////////////////////////////////////////////////////////////////////////
void CzVoteImage::DrawAndCheckLine(int x1, int y1, int x2, int y2, unsigned id, CzArray<CzElem> &iscts)
{
  int dx, dy, inc_x, inc_y, x, y, err;
  unsigned MAX_HYPS = 1000;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      SetAndCheckPixel(x, y, id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          SetAndCheckPixel(x, y, id, iscts);
      }
      x += inc_x;
    } while(x != x2 && iscts.Size() <= MAX_HYPS);
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      SetAndCheckPixel(x, y, id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetAndCheckPixel(x, y, id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Returns false if line was clipped to 0 length.
bool CzVoteImage::FindLineEnd(int x1, int y1, int x2, int y2, unsigned id, int *xe, int *ye)
{
  int dx, dy, inc_x, inc_y, x, y, err;

  if(!ClipLine(width-1, height-1, &x1, &y1, &x2, &y2))
    return false;
  dx = x2 - x1;
  dy = y2 - y1;
  if(dx == 0 && dy == 0)  // line might be clipped to length 0
    return false;
  x = x1;
  y = y1;
  if(dx >= 0)
    inc_x = 1;
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  if(dy >= 0)
    inc_y = 1;
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8
  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      if(CheckPixel(x, y, id))
      {
        *xe = x;
        *ye = y;
        return true;
      }
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          if(CheckPixel(x, y, id))
          {
            *xe = x;
            *ye = y;
            return true;
          }
      }
      x += inc_x;
    } while(x != x2);
    *xe = x;
    *ye = y;
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      if(CheckPixel(x, y, id))
      {
        *xe = x;
        *ye = y;
        return true;
      }
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          if(CheckPixel(x, y, id))
          {
            *xe = x;
            *ye = y;
            return true;
          }
      }
      y += inc_y;
    } while(y != y2);
    *xe = x;
    *ye = y;
  }
  return true;
}

NAMESPACE_CLASS_END()

