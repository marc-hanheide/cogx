//
// = FILENAME
//    
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

#ifndef HSSCandScan2D_hh
#define HSSCandScan2D_hh

#include "HSSScan2D.hh"

namespace HSS {

class CandScan2D : public Scan2D {

public:
  // True if this scan is from a corridor like environment
  bool m_Corridor;
  double m_MainDirS;
  double m_MainDirR;

  // The center of mass for the scan
  Eigen::Vector3d m_C;

  // The bounding box of the scan used to check for overlap. The
  // column contains the x,y coordinate of the lower left corner
  Eigen::MatrixXd m_BB;
  // The bounding box is also given as the lower left corner (first
  // column) in m_BB, the direction and the width height.
  double m_BBdir;
  double m_BBw;
  double m_BBh;
};

/**
 * This function will calculate the extra stuff added to a Scan2D to
 * make it a CandScan2D 
 *
 * @see CandScan2D
 */
void makeScanCandidate(HSS::CandScan2D &scan, const Eigen::Vector3d &xsR);

}; // namespace HSS

#endif // HSSCandScan2D_hh
