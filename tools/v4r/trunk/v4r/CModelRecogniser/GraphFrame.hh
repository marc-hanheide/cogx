/**
 * $Id$
 */

#ifndef P_GRAPH_FRAME_HH
#define P_GRAPH_FRAME_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"
#include "Plane.hh"


namespace P
{


class GraphFrame
{
public:
  unsigned id;
  unsigned idx;

  std::vector<cv::Ptr<Plane> > planes;

  GraphFrame();
  GraphFrame(std::vector<cv::Ptr<Plane> > &pl, unsigned _id, unsigned _idx=UINT_MAX);
  ~GraphFrame();
};






/*************************** INLINE METHODES **************************/


} //--END--

#endif

