/**
 * $Id: VoteImage.hh,v 1.9 2007/02/04 23:51:47 mxz Exp mxz $
 *
 * TODO: write some macros or inline methods of Link class for properly handling
 *       search line ids (search_line_id = 8*line_id + end_id)
 */

#ifndef Z_VOTE_IMAGE_HH
#define Z_VOTE_IMAGE_HH

#include <math.h>
#include "Gestalt.hh"
#include "Vector2.hh"

namespace Z
{

// types of search (vote) lines
#define VOTE_NONE  0x0
#define VOTE_E     0x1  // the edge itself
#define VOTE_TS    0x2  // tangent start
#define VOTE_TE    0x3  // tangent end
#define VOTE_NLS   0x4  // normal left start
#define VOTE_NLE   0x5  // normal left end
#define VOTE_NRS   0x6  // normal right start
#define VOTE_NRE   0x7  // normal right end

#define VOTE_IS_TANGENT(x) ((x) == VOTE_TS || (x) == VOTE_TE)
#define VOTE_IS_NORMAL(x) ((bool)((x) & 0x4))
// return START or END for a vote type
#define VOTE_END(x) ((x) & 0x1)

class VoteImage
{
public:
  class Elem
  {
  public:
    /* Gestalt::Type type;  // line, arc, also end types? */
    unsigned id;
    Elem *next;
    Elem() {}
    Elem(const Elem &e) : /*type(e.type),*/ id(e.id), next(0) {}
    bool operator==(const Elem &e) { return /*type == e.type &&*/ id == e.id; }
  };

  class LineStub
  {
  public:
    int x, y, dx, dy, inc_x, inc_y, err, len;
    /*Gestalt::Type type;*/
  };

  int width;
  int height;
  Elem **data;
  Elem *store;
  int store_size;
  int fill;
  Array<LineStub> lines;

  VoteImage(int w, int h);
  virtual ~VoteImage();
  void SetNumLines(unsigned n);
  Elem *NewElem(/*Gestalt::Type type,*/ unsigned id);
  void Clear();
  void SetPixel(int x, int y, /*Gestalt::Type type,*/ unsigned id);
  void CheckPixel(int x, int y, /*Gestalt::Type type,*/ unsigned id,
      Array<Elem> &iscts);
  bool CheckPixel(int x, int y, /*Gestalt::Type type,*/ unsigned id);
  void SetAndCheckPixel(int x, int y, /*Gestalt::Type type,*/ unsigned id,
      Array<Elem> &iscts);
  void CreateIntersection(Elem *e, /*Gestalt::Type type,*/ unsigned id,
      Array<Elem> &iscts);
  Elem *Pixel(int x, int y) {return data[y*width + x];}
  void InitLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
      unsigned id);
  void InitLine(const Vector2 &a, const Vector2 &b, /*Gestalt::Type type,*/
      unsigned id)
  {
    InitLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), /*type,*/ id);
  }
  int ExtendLine(unsigned id, Array<Elem> &iscts);
  void DrawLine(const Vector2 &a, const Vector2 &b, /*Gestalt::Type type,*/
      unsigned id)
  {
    DrawLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), /*type,*/ id);
  }
  void DrawLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
      unsigned id);
  void CheckLine(const Vector2 &a, const Vector2 &b, /*Gestalt::Type type,*/
      unsigned id, Array<Elem> &iscts)
  {
    CheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), /*type,*/ id,
        iscts);
  }
  void CheckLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
      unsigned id, Array<Elem> &iscts);
  void DrawAndCheckLine(const Vector2 &a, const Vector2 &b,
      /*Gestalt::Type type,*/ unsigned id, Array<Elem> &iscts)
  {
    DrawAndCheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), /*type,*/
        id, iscts);
  }
  void DrawAndCheckLine(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
      unsigned id, Array<Elem> &iscts);
  bool FindLineEnd(int x1, int y1, int x2, int y2, /*Gestalt::Type type,*/
      unsigned id, int *xe, int *ye);
};

}

#endif

