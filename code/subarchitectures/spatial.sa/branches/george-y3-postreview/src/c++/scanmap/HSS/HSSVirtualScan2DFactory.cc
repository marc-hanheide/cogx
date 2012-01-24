//
// = FILENAME
//    HSSVirtualScan2DFactory.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2006 Patric Jensfelt (Cure)
//                  2010 Patric Jensfelt (HSS)
//
/*----------------------------------------------------------------------*/

#include "HSSVirtualScan2DFactory.hh"
#include "HSSGridLineRayTracer.hh"
#include "HSSutils.hh"

namespace HSS {

namespace VirtualScan2DFactory {
  
int getVirtualScan2D(GridMap2D<char> &lgm,
                     GridMapFunctor<char> &func,
                     const Eigen::Vector3d &xs,
                     Scan2D &scan, 
                     double angleStepDeg, 
                     double startAngleDeg,
                     double FOVDeg,
                     double maxRange)
{
  HSS::CharGridMapFunctor gridfunc;
  HSS::GridLineRayTracer<char> rayTracer(lgm, gridfunc);

  if (angleStepDeg <= 0) {
    std::cerr << "VirtualScan2DFactory::getVirtualScan2D cannot accept "
              << "angleStep=" << angleStepDeg << "deg<=0deg" << std::endl;
    throw std::exception();
  }

  // Calculate number of scan points needed. The +1 at the end is to
  // account for the fact that if there are (N+1) beams there are only
  // N steps between them.
  int n = int(FOVDeg / angleStepDeg + 0.5) + 1;

  double angleStep = HSS::deg2rad(angleStepDeg);
  double startAngle = HSS::deg2rad(startAngleDeg);

  double rTmp[n];
  scan.alloc(n);

  for (int i = 0; i < n; i++) {
    scan.theta[i] = startAngle + angleStep * i;
    rayTracer.setStart(xs[0], xs[1], xs[2] + scan.theta[i]);

    bool assigned = false;
    while (!assigned &&
           !rayTracer.isOutside() && 
           rayTracer.getDistToStart() <= maxRange) {

      if (func.isOccupiedValue(rayTracer.data())) {
        rTmp[i] = rayTracer.getDistToStart();
        assigned = true;
      } else {
        rayTracer.stepRay();
      }

    }

    if (rayTracer.isOutside() ||
        rayTracer.getDistToStart() > maxRange) {
      rTmp[i] = maxRange + 1e-3;
    }
  }

  // Filter the data so that measurements are assigned to the smallest
  // of its neightbors if the range difference is large 
  const double maxRangeDiff = 2;
  // First we deal with the first and last data points
  if (rTmp[0] - rTmp[1] > maxRangeDiff) 
    scan.range[0] = rTmp[1];
  else 
    scan.range[0] = rTmp[0];
  if (rTmp[n-1] - rTmp[n-2] > maxRangeDiff) 
    scan.range[n-1] = rTmp[n-2];
  else 
    scan.range[n-1] = rTmp[n-1];
  for (int i = 1; i < (n-1); i++) {
    scan.range[i] = rTmp[i];
    if (rTmp[i-1] < rTmp[i+1]) {
      if (scan.range[i] - rTmp[i-1] > maxRangeDiff) {
        scan.range[i] = rTmp[i-1];
      }
    } else {
      if (scan.range[i] - rTmp[i+1] > maxRangeDiff) {
        scan.range[i] = rTmp[i+1];
      }
    }
  }

  scan.min_theta = startAngle;
  scan.max_theta = startAngle + HSS::deg2rad(FOVDeg);
  scan.min_range = 0;
  scan.max_range = maxRange;
             
  return 0; 
}

}; // namespace VirtualScan2DFactory

}; // namespace Cure
