/**
 * $Id: VoteImage.cc,v 1.17 2007/02/04 23:51:47 mxz Exp mxz $
 * TODO: avoid duplicate code
 * TODO: deal with very flat intersections (see 28.11.2006)
 * TODO: avoid drawing the first pixel of EDGE, T, NL, NR four times.
 *       e.g. skip first pixel in InitLine
 */

#include <assert.h>
#include "VoteImage.hh"
#include <string.h>
#include <stdio.h>

namespace Z
{

VoteImage::VoteImage(int w, int h)
{
  width = w;
  height = h;
  data = new Elem*[width*height];
  assert(data != 0);
  store_size = 2*width*height; // TODO: this is an arbitrary choice
  store = new Elem[store_size];
  fill = 0;
}

VoteImage::~VoteImage()
{
  delete[] data;
  delete[] store;
}

void VoteImage::SetNumLines(unsigned n)
{
  lines.Resize(n);
  for(unsigned id = 0; id < lines.Size(); id++)
    lines[id].len = 0;
}

VoteImage::Elem *VoteImage::NewElem(/*Gestalt::Type type,*/ unsigned id)
{
  if(fill < store_size)
  {
    Elem *el = &store[fill++];
    /*el->type = type;*/
    el->id = id;
    el->next = 0;
    return el;
  }
  else
  {
    // TODO: alloc more storage
    printf("Oh lala! out of store!\n");
    return 0;
  }
}

void VoteImage::Clear()
{
  memset(data, 0, width*height*sizeof(Elem*));
  fill = 0;
}

inline void VoteImage::SetPixel(int x, int y, /*Gestalt::Type type,*/
    unsigned id)
{
  Elem *e = data[y*width + x];
  if(e == 0)
    data[y*width + x] = NewElem(/*type,*/ id);
  else
  {
    while(e->next != 0)
      e = e->next;
    e->next = NewElem(/*type,*/ id);
  }
}

inline void VoteImage::SetAndCheckPixel(int x, int y, /*Gestalt::Type type,*/
    unsigned id, Array<Elem> &iscts)
{
  Elem *e = data[y*width + x];
  if(e == 0)
    data[y*width + x] = NewElem(/*type,*/ id);
  else
  {
    Elem *last;
    while(e != 0)
    {
      CreateIntersection(e, /*type,*/ id, iscts);
      last = e;
      e = e->next;
    }
    last->next = NewElem(/*type,*/ id);
  }
}

inline void VoteImage::CheckPixel(int x, int y, /*Gestalt::Type type,*/
    unsigned id, Array<Elem> &iscts)
{
  Elem *e = data[y*width + x];
  while(e != 0)
  {
    CreateIntersection(e, /*type,*/ id, iscts);
    e = e->next;
  }
}

/**
 * This CheckPixel function is called from FindLineEnd.
 * Lines end only at other visible lines, not tangents or normals.
 */
inline bool VoteImage::CheckPixel(int x, int y, /*Gestalt::Type type,*/
    unsigned id)
{
  Elem *e = data[y*width + x];
  while(e != 0)
  {
    if(e->id%8 == VOTE_E)
      if(/*e->type != type ||*/ e->id != id)
        return true;
    e = e->next;
  }
  return false;
}

inline void VoteImage::CreateIntersection(Elem *e, /*Gestalt::Type type,*/
    unsigned id, Array<Elem> &iscts)
{
  // don't intersect edge with itself  TODO: why not?
  if(/*e->type != type ||*/ e->id/8 != id/8)
    if(!iscts.ContainsBackwards(*e))
      iscts.PushBack(*e);
}

void VoteImage::InitLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
    unsigned id)
{
  LineStub &line = lines[id];
  line.x = line.y = 0;
  line.dx = line.dy = 0;
  line.inc_x = line.inc_y = 0;
  line.len = 0;
  /*line.type = type;*/
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

/**
 * Extends the givne (search) line.
 * @param id  line to extend
 * @param iscts  array contains created intersections after return
 * Returns the number of created intersections or -1 if the line could not be
 * further extended, i.e. its maximum length was reached.
 */
int VoteImage::ExtendLine(unsigned id, Array<Elem> &iscts)
{
  LineStub &line = lines[id];
  iscts.Clear();
	
  if(line.len > 0)
  {
		SetAndCheckPixel(line.x, line.y, /*line.type,*/ id, iscts);
    line.len--;
    if(line.dx >= line.dy)
    {
      line.err += line.dy;
      if(line.err >= 0)
      {
        line.y += line.inc_y;
        line.err -= line.dx;
        if(line.len > 0)  // TODO: this is always true except once
          // make line dense
          SetAndCheckPixel(line.x, line.y, /*line.type,*/ id, iscts);
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
          SetAndCheckPixel(line.x, line.y, /*line.type,*/ id, iscts);
      }
      line.y += line.inc_y;
    }
    return iscts.Size();
  }
  else
    return -1;
}


/**
 *	AtImageBorder()
 *	if border of image with the vote line is reached, return 0..3
 */
unsigned VoteImage::AtImageBorder(unsigned id)
{
  LineStub &line = lines[id];
	if(line.x <= 0) return 0;					// left image border
	if(line.y <= 0) return 1;					// top image border
	if(line.x >= width) return 2;			// right image border
	if (line.y >= height) return 3;		// bottom image border
	else return UNDEF_ID;
}


void VoteImage::DrawLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
    unsigned id)
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
      SetPixel(x, y, /*type,*/ id);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          SetPixel(x, y, /*type,*/ id);
      }
      x += inc_x;
    } while(x != x2); // TODO: x2 is not coloured!
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      SetPixel(x, y, /*type,*/ id);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetPixel(x, y, /*type,*/ id);
      }
      y += inc_y;
    } while(y != y2);
  }
}

void VoteImage::CheckLine(int x1, int y1, int x2, int y2,
    /*Gestalt::Type type,*/ unsigned id, Array<Elem> &iscts)
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
      CheckPixel(x, y, /*type,*/ id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          CheckPixel(x, y, /*type,*/ id, iscts);
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
      CheckPixel(x, y, /*type,*/ id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          CheckPixel(x, y, /*type,*/ id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}

void VoteImage::DrawAndCheckLine(int x1, int y1, int x2, int y2,
    /*Gestalt::Type type,*/ unsigned id, Array<Elem> &iscts)
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
      SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        if(x + inc_x != x2)
          // make line dense
          SetAndCheckPixel(x, y, /*type,*/ id, iscts);
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
      SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
          // make line dense
          SetAndCheckPixel(x, y, /*type,*/ id, iscts);
      }
      y += inc_y;
    } while(y != y2 && iscts.Size() <= MAX_HYPS);
  }
}

/**
 * Returns false if line was clipped to 0 length.
 */
bool VoteImage::FindLineEnd(int x1, int y1, int x2, int y2,
    /*Gestalt::Type type,*/ unsigned id, int *xe, int *ye)
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
      if(CheckPixel(x, y, /*type,*/ id))
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
          if(CheckPixel(x, y, /*type,*/ id))
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
      if(CheckPixel(x, y, /*type,*/ id))
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
          if(CheckPixel(x, y, /*type,*/ id))
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

}
