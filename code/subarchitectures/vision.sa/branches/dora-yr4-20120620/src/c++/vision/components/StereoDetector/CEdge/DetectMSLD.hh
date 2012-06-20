/**
 * $Id$
 * Johann Prankl, 2010-03-30 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DETECT_MSLD_HH
#define P_DETECT_MSLD_HH

#include <limits.h>
//#include <GL/glut.h>
#include <dlfcn.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PNamespace.hh"
#include "Line.hh"
#include "Array.hh"
#include "CEdge.hh"
#include "FormLines.hh"
#include "CreateMSLD.hh"
#include "Math.hh"


namespace P
{


class DetectMSLD
{
private:
  IplImage *dx, *dy, *edge;
  
  unsigned cannyLow, cannyHigh;
  bool filter;            //filter lines without descriptor
  Array<Line*> lns;

  CEdge getEdge;
  FormLines formLines;
  CreateMSLD createMSLD;

  static float MIN_LINE_LENGTH;
  static bool SUB_PIXEL_LINES;

  
public:
  Array<Segment *> segments;

  DetectMSLD();
  ~DetectMSLD();

  void Detect(IplImage *img, Array<Line*> &lines);
  void Match(Array<Line*> &lines1, Array<Line*> &lines2, int (*matches)[2], int buf_size, int &num, double thrGlob=.55);
  void SetParameter(unsigned l, unsigned h, bool f){cannyLow=l; cannyHigh=h; filter=f;};

  void Draw(IplImage *img, Array<Line*> &lines);
};


/************************** INLINE METHODES ******************************/



}

#endif

