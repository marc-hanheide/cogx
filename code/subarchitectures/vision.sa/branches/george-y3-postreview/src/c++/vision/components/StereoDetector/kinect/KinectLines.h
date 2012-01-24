/**
 * @file KinectLines.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D lines from Kinect data.
 */

#ifndef Z_KINECT_LINES_H
#define Z_KINECT_LINES_H

#include "KinectBase.h"
#include "VisionCore.hh"

#include "Line.hh"


namespace Z
{

/**
 * @brief Class KinectLines: Calculate 3D lines from kinect data.
 */
class KinectLines : public KinectBase
{
private:
  int numLines;              ///< Number of extracted 3D lines
  

public:
  KinectLines(KinectCore *kc, VisionCore *vc);
  ~KinectLines() {}

  int Num3DGestalts() {return numLines;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
  
};

}

#endif
