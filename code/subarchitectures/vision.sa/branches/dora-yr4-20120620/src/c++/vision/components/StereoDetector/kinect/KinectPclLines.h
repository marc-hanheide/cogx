/**
 * @file KinectPclLines.h
 * @author Andreas Richtsfeld
 * @date September 2011
 * @version 0.1
 * @brief Calculate 3D lines from Kinect data with Tom's RGBD-Segmenter.
 */

#ifndef Z_KINECT_PCL_LINES_H
#define Z_KINECT_PCL_LINES_H

#include "KinectBase.h"
#include "VisionCore.hh"
#include "IdImage.hh"
#include "FormSegments.hh"

#include "v4r/TomGine/tgEngine.h"
// #include "v4r/TomGine/tgImageProcessor.h"
#include "v4r/RGBDSegment/RGBDSegment.h"


namespace Z
{

/**
 * @brief Class KinectLines: Calculate 3D lines from kinect data.
 */
class KinectPclLines : public KinectBase
{
private:
  int numLines;                       ///< Number of extracted 3D lines
  
  TomGine::tgEngine *tgEngine;
  RGBDSegment *rgbdSegment;

public:
  KinectPclLines(KinectCore *kc, VisionCore *vc);
  ~KinectPclLines() {}

  int Num3DGestalts() {return numLines;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  void Process();
  
};

}

#endif
