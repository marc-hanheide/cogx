/**
 * $Id: FormLines.cc,v 1.22 2007/02/18 18:02:48 mxz Exp mxz $
 */


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "Segment.hh"
#include "Line.hh"
#include "FormLines.hh"
#include "rosin_lines.hh"

namespace Z
{

static int CmpLines(const void *a, const void *b)
{
  // HACK: use weight instead of sig
  //if( Lines(*(unsigned*)a)->weight > Lines(*(unsigned*)b)->weight )
  if( Lines(*(unsigned*)a)->sig > Lines(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormLines::FormLines(Config *cfg)
: GestaltPrinciple(cfg)
{
  done = false;
  next_principles.PushBack(FORM_JUNCTIONS);
  //next_principles.PushBack(FORM_PARALLEL_LINES);
}

void FormLines::Reset(const Image *img)
{
  done = false;
}

void FormLines::Operate(bool incremental)
{ 
  StartRunTime();
  // note: we only want to run this once for repeated calls to Operate()
  if(!done)
  {
    Create();
    Rank();
    done = true;
  }
  StopRunTime();
}

/**
 * Create straight lines from segments.
 * Each segment is split up into straight line portions.
 */
void FormLines::Create()
{
  LineSegmenter ls;
  for(unsigned rank = 0; rank < NumSegments(); rank++)
  {
    unsigned seg = RankedGestalts(Gestalt::SEGMENT, rank);
    ls.RosinFitLinesToSegment(seg);
  }
}

void FormLines::Rank()
{
  RankGestalts(Gestalt::LINE, CmpLines);
}

}
