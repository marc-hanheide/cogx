//
// = FILENAME
//    HSSScan2DFilter.hh
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

#ifndef HSSScan2DFilter_hh
#define HSSScan2DFilter_hh

#include "HSSScan2D.hh"
#include "HSSScanMotionDetector.hh"

namespace HSS {

class Scan2DFilter {

public:

  /**
   * Filter the scan based on the ranges. This looks at the min_range
   * and max_range fields in the scan and put all valid flags to 0 for
   * any reading that falls outside this.
   */
  static void filterScanRange(Scan2D &scan);
  
  /**
   * Does mean filtering of the scan with the aim to remove some of
   * the noise from the scan data. It does this in a window that
   * extends width to the left and right of the current reading. It
   * will include all range readings in that window that is less than
   * maxRangeDiff*num_reading_away away from the current reading.
   */
  static void filterScanMean(Scan2D &scan, int width, double maxRangeDiff);
  
  /**
   * This function aims to remove small "objects" from the scan with
   * the aim to clean it from for example legs of chairs or tables. It
   * does this by looking for clusters of points where the consecutive
   * points do not differ in range more than maxRangeDiff. If these
   * clusters have at least minLen number of points they are kept
   * intact otherwise the valid flag is toogled to 0 for the scans in
   * the too small cluster.
   */
  static void filterScanSmall(Scan2D &scan, int minlen, double maxRangeDiff);

  /**
   * Uses the detected motion to put the valid flag for the
   * corresponding readings to 0. You can use the width parameter to
   * tell that you want to invalididate a the width neighbor of each
   * point as well.
   */
  static void filterScanMotion(HSS::Scan2D &scan, 
                               std::vector<ScanMotionDetector::Movement> &m,
                               int width = 0);
  /**
   * Detects motion (assumes frequent calls) and uses it to put the
   * valid flag for the corresponding readings to 0. You can use the
   * width parameter to tell that you want to invalididate a the width
   * neighbor of each point as well.
   */
  static void filterScanMotion(HSS::Scan2D &scan, 
                               ScanMotionDetector &motionDetector,
                               const Eigen::VectorXd &X, 
                               const Eigen::Vector3d &xsR,
                               int width = 0);
}; // class Scan2DFilter

}; // namespace HSS

#endif // HSSScan2DFilter_hh
