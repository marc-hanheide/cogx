/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_POSE_CV_HH
#define P_POSE_CV_HH

#include <limits.h>
#include <map>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include "PNamespace.hh"


namespace P
{


class PoseCv
{
private:    
  

public:
  CvMat *R;
  CvMat *t;
  CvMat *n;

  PoseCv();
  ~PoseCv();
};





/*********************************** INLINE *********************************/




} //--THE END--

#endif

