//
// = FILENAME
//    HSSScan2D.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSScan2D_hh
#define HSSScan2D_hh

#include "Eigen/Core"
#include <vector>

namespace HSS {

class Scan2D {
public:
  // The id of the scan
  long m_Id;

  // The scan data
  std::vector<double> range;
  std::vector<double> theta;
  std::vector<short> valid;
  double min_range;
  double max_range;
  double min_theta;
  double max_theta;
  double range_sigma;

  // The timestamp of the scan
  struct timeval tv;

  // The odometry reading at the time of the scan to be able to tell
  // if we moved far enough from the last added reference scan
  Eigen::Vector3d odom;

  /**
   * Allocate space for n data points and make all of them valid to
   * start with
   */
  void alloc(int n) {
    range.resize(n);
    valid.resize(n);
    theta.resize(n);
    for (unsigned int i = 0; i < valid.size(); i++) {
      valid[i] = 1;
    }
  }

  int nvalid() const {
    int n = 0;
    for (unsigned int i = 0; i < valid.size(); i++) {
      if (valid[i]) n++;
    }
    return n;
  }
};

inline bool readCureFileScan(std::istream &is, Scan2D &scan)
{
  scan.min_range = 0.01;
  scan.range_sigma = 0.05;

  std::string line;
  getline(is, line);
  std::istringstream str(line);

  double junk;
  int n;
  double da;
  double rangeres;
  if (str >> junk >> junk >> n >> scan.tv.tv_sec >> scan.tv.tv_usec
      >> junk >> junk >> junk >> junk >> junk >> junk >> junk 
      >> da >> scan.min_theta >> scan.max_range >> rangeres) {
    
    scan.alloc(n);

    for (int i = 0; i < n; i++) {
      if ( !(str >> scan.range[i]) ) {
        std::cerr << "Failed to read scan, only read "
                  << i << " pts of " << n << std::endl;
        return false;
      }

      scan.theta[i] = scan.min_theta + da * i;
    }

    return true;

  }

  return false;
}

}; // namespace HSS

#endif // HSSScan2D_hh
