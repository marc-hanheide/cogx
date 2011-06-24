//
// = FILENAME
//    HSSScan2DMatcher.hh
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

#ifndef HSSScan2DMatcher_hh
#define HSSScan2DMatcher_hh

#include "HSSCandScan2D.hh"

#include <csm/csm_all.h>

namespace HSS {

/**
 * This class does scan matching.
 *
 * @author Patric Jensfelt
 * @see
 */
class Scan2DMatcher {
public:

  /**
   * This function performs scan matching between scanRef and
   * scanSens. The transformation that it calculates s such that it
   * transforms points in the scanSens scan into the frame of the
   * ref.scan. That is conceptually it does 
   * scanSens_in_ref_frame = T * scanSens
   * 
   * @param scanRef the reference scan
   * @param scanSens the scan that we want to relate to the ref scan
   * @param Tguess the initial guess for the transformation
   * @param T the transformation output
   * @param n the number of iterations
   * @param corrs correspondences of the scan matching, the first element of the
   * pair is the index for ref data and the second element the one for sens data
   * @ return true if match succeeded, else false
   */
  bool scanMatch(const HSS::Scan2D &scanRef, 
                 const HSS::Scan2D &scanSens,
                 const Eigen::Vector3d &Tguess, 
                 Eigen::Vector3d &T, int &n,
                 std::vector< std::pair<int, int> > *corrs);
protected:
  void setDefaultScanMatchParams(struct sm_params *p);

};

}; // namespace HSS

void operator<<(LDP sink, const HSS::Scan2D &src);

#endif // HSSScan2DMatcher_hh
