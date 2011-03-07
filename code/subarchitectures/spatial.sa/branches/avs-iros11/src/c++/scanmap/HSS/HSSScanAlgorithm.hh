//
// = FILENAME
//    HSSScanAlgorithm.hh
//
// = AUTHOR
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSScanAlgorithm_hh
#define HSSScanAlgorithm_hh

namespace HSS {

/**
 * This is the baseclass for a group of methods that operate on scans
 *
 * @author Patric Jensfelt
 * @see
 */
class ScanAlgorithm {
public:

  /**
   * Constructor 
   */
  ScanAlgorithm();

  /**
   * Destructor
   */
  virtual ~ScanAlgorithm();

  /*==================================================*/
  // = ACCESS CONFIG
  /*==================================================*/

  /**
   * Get the max range to assume for the data [m]
   */
  double maxRange() const { return m_MaxRange; } 

  /**
   * Set standard deviation in the range measurements [m]
   */
  double sigmaR() const { return m_SigmaR; }

  /**
   * Set standard deviation in the angle measurements [rad]
   */
  double sigmaA() const { return m_SigmaA; }

  /*==================================================*/
  // = CHANGE CONFIG
  /*==================================================*/

  /**
   * Config std.dev in range and angle for the Hokuyo URG-04LX
   */
  virtual void configForHokuyoURG04LX();

  /**
   * Config std.dev in range and angle for the PLS 200
   */
  virtual void configForPLS200();

  /**
   * Config std.dev in range and angle for the LMS 200
   */
  virtual void configForLMS200();

  /**
   * Config std.dev in range and angle for the LMS 291
   */
  virtual void configForLMS291();

  /**
   * Set the max range to assume for the data [m]
   */
  void setMaxRange(double r) { m_MaxRange = r; } 

  /**
   * Set standard deviation in the range measurements [m]
   */
  void setSigmaR(double s) { m_SigmaR = s; }

  /**
   * Set standard deviation in the angle measurements [rad]
   */
  void setSigmaA(double s) { m_SigmaA = s; }
 
protected:

  /** The maximum range for a data point to be even considered. This
      is used in the initial selection phase when the indices of
      unused points are created. */
  double m_MaxRange;

  /** The uncertainty in range for the measurements */
  double m_SigmaR;

  /** The angular uncertainty for the measurements */
  double m_SigmaA;
};

}; // namespace HSS

#endif // HSSScanAlgorithm_hh
