/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_POSE_HH
#define P_POSE_HH

#include <limits.h>
#include <map>
#include <opencv/cv.h>
#include "PNamespace.hh"
#include "matrix.h"


namespace P
{


class Pose
{
private:    
  

public:
  Matrix R;
  Matrix t;
  Vector n;

  Pose();
  Pose(Matrix &RR, Matrix &tt);
  Pose(Matrix &RR, Matrix &tt, Matrix &nn);
  ~Pose();

};





/*********************************** INLINE *********************************/




} //--THE END--

#endif

