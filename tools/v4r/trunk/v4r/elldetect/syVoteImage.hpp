//
// (C) 2010, Aitor Aldoma Buchaca,
//           Johann Prankl,
//           Michael Zillich,
//           Gerhard Obernberger <gerhard@obernberger.at>
//
//

#ifndef SYVOTEIMAGE_HPP
#define SYVOTEIMAGE_HPP

#include "multiplatform.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "syVector2.hpp"
#include "syArray.hpp"

// include Steinbichler libraries
#include "ipAOI.hpp"

// include OpenCV libraries
#include OCV_CV_H

NAMESPACE_CLASS_BEGIN( RTE )

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
#define VOTE_END(x) ((x) & 0x1)

//begin class///////////////////////////////////////////////////////////////////
//
//    CLASS DESCRIPTION:
//       RTE canny edge segment including bresenham line algorithm to extend lines
//
//    FUNCTION DESCRIPTION:
//
////////////////////////////////////////////////////////////////////////////////
class CzVoteImage
{
public:
  class CzElem
  {
  public:
    unsigned id;
    CzElem *next;
    CzElem() {}
    CzElem(const CzElem &e) : id(e.id), next(0) {}
    bool operator==(const CzElem &e) { return id == e.id; }
  };


  class CzLineStub
  {
  public:
    int x, y, dx, dy, inc_x, inc_y, err, len;
    
  };

  int width;
  int height;
  CzElem **data;
  CzElem *store;
  int store_size;
  int fill;
  CzArray<CzLineStub> lines;

  CzVoteImage(int w, int h);
  virtual ~CzVoteImage();
  void SetNumLines(unsigned n);
  CzElem *NewElem(unsigned id);
  void Clear();
  void SetPixel(int x, int y, unsigned id);
  void CheckPixel(int x, int y, unsigned id, CzArray<CzElem> &iscts);
  bool CheckPixel(int x, int y, unsigned id);
  void SetAndCheckPixel(int x, int y, unsigned id, CzArray<CzElem> &iscts);
  void CreateIntersection(CzElem *e, unsigned id, CzArray<CzElem> &iscts);
  CzElem *Pixel(int x, int y) {return data[y*width + x];}
  void InitLine(int x1, int y1, int x2, int y2, unsigned id);
  void InitLine(const CzVector2 &a, const CzVector2 &b, unsigned id)
  {
    InitLine(cvRound(a.x), cvRound(a.y), cvRound(b.x), cvRound(b.y), id);
  }
  int ExtendLine(unsigned id, CzArray<CzElem> &iscts);
  void DrawLine(const CzVector2 &a, const CzVector2 &b, unsigned id)
  {
    DrawLine(cvRound(a.x), cvRound(a.y), cvRound(b.x), cvRound(b.y), id);
  }
  void DrawLine(int x1, int y1, int x2, int y2, unsigned id);
  void CheckLine(const CzVector2 &a, const CzVector2 &b, unsigned id, CzArray<CzElem> &iscts)
  {
    CheckLine(cvRound(a.x), cvRound(a.y), cvRound(b.x), cvRound(b.y), id, iscts);
  }
  void CheckLine(int x1, int y1, int x2, int y2, unsigned id, CzArray<CzElem> &iscts);
  void DrawAndCheckLine(const CzVector2 &a, const CzVector2 &b, unsigned id, CzArray<CzElem> &iscts)
  {
    DrawAndCheckLine(cvRound(a.x), cvRound(a.y), cvRound(b.x), cvRound(b.y), id, iscts);
  }
  void DrawAndCheckLine(int x1, int y1, int x2, int y2, unsigned id, CzArray<CzElem> &iscts);
  bool FindLineEnd(int x1, int y1, int x2, int y2, unsigned id, int *xe, int *ye);
};
//end class/////////////////////////////////////////////////////////////////////


  inline bool operator==(const CzVoteImage::CzElem &a, const CzVoteImage::CzElem &b) { return a.id == b.id; }

NAMESPACE_CLASS_END()

#endif

