//
// = FILENAME
//    HSSDisplayFunctions.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSDisplayFunctions_hh
#define HSSDisplayFunctions_hh

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "Eigen/Core"

#include "HSSScan2D.hh"
#include "HSSGridMap2D.hh"
#include "HSSutils.hh"
#include "HSSDoorExtractor.hh"

namespace HSS {

  void displayScan(peekabot::PointCloudProxy scanPts,
                   HSS::Scan2D &scan, 
                   const Eigen::VectorXd &X, const Eigen::Vector3d &xsR,
                   float r, float g, float b);
  
  void displayUncertainty(peekabot::GroupProxy &root,
                          const Eigen::Vector3d &X, const Eigen::Matrix3d &P,
                          double nsigma, float r, float g, float b);
  
  void addDoorPost(peekabot::GroupProxy &door, double width, 
                   float r, float g, float b);
  
  void displayDoorMeas(peekabot::GroupProxy &root,
                       const Eigen::VectorXd &X, const Eigen::Vector3d &xsR,
                       HSS::DoorExtractor &doorExtractor, std::string id);
  
  void displayStateWithUnc(peekabot::GroupProxy &refroot,
                           const Eigen::VectorXd &X, const Eigen::MatrixXd &P,
                           int statepos, HSS::Scan2D *scan, 
                           double nsigma,
                           float r, float g, float b);

  void displayPolygon(peekabot::ObjectProxyBase &root,
                      Eigen::Vector3d pose,
                      Eigen::MatrixXd polygon,
                      float r=0, float g=0, float b=0);

template<class MAPDATA>
void displayGrid(peekabot::GroupProxy &refroot, 
                 HSS::GridMap2D<MAPDATA> &grid,
                 HSS::GridMapFunctor<MAPDATA> &functor)
{
  peekabot::OccupancyGrid2DProxy gridp;
  gridp.add(refroot, "gridmap", grid.getCellSize(), 
            1,1,1,
            0,0,0, 
            peekabot::REPLACE_ON_CONFLICT);
  peekabot::OccupancySet2D set;
  for (int i = -grid.getSize(); i <= grid.getSize(); i++) {
    for (int j = -grid.getSize(); j <= grid.getSize(); j++) {
      double x, y;
      grid.index2WorldCoords(i,j,x,y);
      set.set_cell(x,y,functor.getBelief(grid(i,j)));
    }
  }
  gridp.set_cells(set);
  gridp.set_position(0,0,-0.001);
}

}; // namespace HSS

#endif // HSSDisplayFunctions_hh
