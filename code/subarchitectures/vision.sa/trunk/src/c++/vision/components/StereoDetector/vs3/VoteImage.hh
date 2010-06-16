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

// for junctions between ellipses and lines (lines without normal search lines)
#define VOTE_EOTL  0x4  // ellipse outer tangent left
#define VOTE_EOTR  0x5  // ellipse outer tangent right
#define VOTE_EITL  0x6  // ellipse inner tangent left
#define VOTE_EITR  0x7  // ellipse inner tangent right

#define VOTE_IS_TANGENT(x) ((x) == VOTE_TS || (x) == VOTE_TE)
#define VOTE_IS_NORMAL(x) ((bool)((x) & 0x4))
// return START or END for a vote type
#define VOTE_END(x) ((x) & 0x1)
#define VOTE_VERTEX(x) ((x) & 0x1)

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
    Elem(const Elem &e) : id(e.id), next(0) {}
    bool operator==(const Elem &e) { return id == e.id; }
  };

  class LineStub
  {
  public:
    int x, y, dx, dy, inc_x, inc_y, err, len;
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
	void ExtendNumLines(unsigned n);
  Elem *NewElem(unsigned id);
  void Clear();
  void SetPixel(int x, int y, unsigned id);
  void CheckPixel(int x, int y, unsigned id, Array<Elem> &iscts);
  bool CheckPixel(int x, int y, unsigned id);
  void SetAndCheckPixel(int x, int y, unsigned id, Array<Elem> &iscts);
  void CreateIntersection(Elem *e, unsigned id, Array<Elem> &iscts);
  Elem *Pixel(int x, int y) {return data[y*width + x];}
  void InitLine(int x1, int y1, int x2, int y2, unsigned id);
  void InitLine(const Vector2 &a, const Vector2 &b, unsigned id)
  {
    InitLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  int ExtendLine(unsigned id, Array<Elem> &iscts);
  void DrawLine(const Vector2 &a, const Vector2 &b, unsigned id)
  {
    DrawLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id);
  }
  void DrawLine(int x1, int y1, int x2, int y2, unsigned id);
  void CheckLine(const Vector2 &a, const Vector2 &b, unsigned id, Array<Elem> &iscts)
  {
    CheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id,
        iscts);
  }
  void CheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts);
  void DrawAndCheckLine(const Vector2 &a, const Vector2 &b, unsigned id, Array<Elem> &iscts)
  {
    DrawAndCheckLine(lrint(a.x), lrint(a.y), lrint(b.x), lrint(b.y), id, iscts);
  }
  void DrawAndCheckLine(int x1, int y1, int x2, int y2, unsigned id, Array<Elem> &iscts);
  bool FindLineEnd(int x1, int y1, int x2, int y2, unsigned id, int *xe, int *ye);
};

}

#endif

