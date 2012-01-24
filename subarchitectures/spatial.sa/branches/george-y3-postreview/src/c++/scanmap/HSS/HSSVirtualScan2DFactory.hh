//
// = FILENAME
//    HSSVirtualScan2DFactory.hh
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

#ifndef HSS_VirtualScan2DFactory_hh
#define HSS_VirtualScan2DFactory_hh

#include "HSSGridMap2D.hh"
#include "HSSScan2D.hh"
#include "Eigen/Core"

namespace HSS {

namespace VirtualScan2DFactory {

  /**
   * Different strategies for filling in data that is unknown when
   * creating virtual scans. We can put them to 0, max reading,
   * interpolate between neighbours, etc
   */
  enum VirtualUnknownStrategy {
    VIRTUAL_ZERO = 0,
    VIRTUAL_MAXRANGE,
    VIRTUAL_INTERPOLNEIGHBOUR,
    VIRTUAL_MAXNEIGHBOUR
  };

  /**
   * Use this function to create a virtual scan from the data in the
   * grid map. This function will set the data members theta, range,
   * min_theta, max_theta, min_range and max_range.
   *
   * @param lgm the GridMap2D to get the virtual scan from
   * @param func a functor that tells what is free/occupied and unknown
   * @param xs sensor pose   
   * @param scan scan object to put result in
   * @param angleStepDeg step in angle between scan points [deg]
   * @param startAngleDeg angle of first scan beam [deg]
   * @param FOVDeg field of view for teh scan [deg]
   * @param maxRange max range for a reading [m]
   */
  int getVirtualScan2D(HSS::GridMap2D<char> &lgm,
                       HSS::GridMapFunctor<char> &func,
                       const Eigen::Vector3d &xs,
                       HSS::Scan2D &scan,
                       double angleStepDeg = 0.5,
                       double startAngleDeg = -90,
                       double FOVDeg = 180,
                       double maxRange = 8);

}; // class VirtualScan2DFactory

}; // namespace HSS

#endif // HSS_VirtualScan2DFactory_hh
