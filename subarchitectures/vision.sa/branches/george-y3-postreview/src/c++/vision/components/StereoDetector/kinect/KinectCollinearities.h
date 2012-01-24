/**
 * @file KinectCollinearities.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculate 3D lines from Kinect data.
 */

#ifndef Z_KINECT_COLLINEARITIES_H
#define Z_KINECT_COLLINEARITIES_H

#include "KinectBase.h"
#include "VisionCore.hh"

#include "Collinearity.hh"
#include "Line3D.h"

namespace Z
{

/**
 * @brief Class KinectLines: Calculate 3D lines from kinect data.
 */
class KinectCollinearities : public KinectBase
{
private:
  int numColls;             ///< Number of extracted 3D lines
  double scale;             ///< scale between color-image-size and point-cloud-size
  
public:
  KinectCollinearities(KinectCore *kc, VisionCore *vc);
  ~KinectCollinearities() {}

  int Num3DGestalts() {return numColls;}
  void Get3DGestalt(Array<double> &values, int id) {}

  void ClearResults();
  
  bool CheckLineValidity(Z::Collinearity *col, Z::Line3D *line[2]);
  void Process();
  
};

}

#endif
