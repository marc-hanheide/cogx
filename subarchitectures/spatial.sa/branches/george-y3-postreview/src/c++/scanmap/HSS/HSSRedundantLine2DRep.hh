//
// = FILENAME
//    HSSRedundantLine2DRep.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1999 Patric Jensfelt
//                  2009 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSRedundantLine2DRep_hh
#define HSSRedundantLine2DRep_hh

#include <Eigen/Core>

#ifndef DEPEND
#include <iostream>
#include <vector>
#endif

namespace HSS {

/**
 * Class that encapulates a line object using a redundant
 * representation and containing uncertainty information
 *
 * NOTE: We use the convention that if you stand on the line,
 * facing towards the end point of the line (away from the start
 * point) then the wall is visible from the left hand side
 *
 * @author Patric Jensfelt
 * @see
 */
class RedundantLine2DRep
{
public:

  enum EndPoints {
    NO_END_POINTS = 0,
    START_POINT   = (1<<0),
    END_POINT     = (1<<1)
  };

public:
  //==================================================
  //               CONSTRUCTORS
  //==================================================

  /** 
   * Constructor
   */
  RedundantLine2DRep();

  /** 
   * Constructor
   */
  RedundantLine2DRep(double xS, double yS, double xE, double yE);

  /** 
   * Copy constructor 
   *
   * @param src Object to replicate
   */
  RedundantLine2DRep(const RedundantLine2DRep& src);

  /**
   * Destructor
   */
  ~RedundantLine2DRep();

  /**
   * Set the start and end point of the line. This will 
   * initiate a calculation of params xC, yC, c, d, 
   * theta, alpha and h
   * The angle theta should be the angle from start to
   * end, i.e. alpha = theta + 0.5*M_PI is not guaranteed.
   * xE = xS + 2.0*h*cos(theta) should hold though
   * The d parameter is calculated so that
   * xC = c*cos(alpha) + d*cos(theta), the test for correctness
   * of sign of d is not guaranteed to be correct if d is very
   * small.
   */
  void setEndPts(double xS, double yS, double xE, double yE);

  /**
   * Set all params of the line at once, no check of 
   * them being valid is made
   */
  void setAll(double xS, double yS, double xE, double yE,
              double xC, double yC, double alpha, double theta, 
              double c, double h, double d);

  /**
   * Set the z paramter of the line.
   */
  void setZ(double z);

  /**
  * Will rotate the line, updating all parameters needed (c, d, h, z
  * should not be effected by a rotation), except R.
  * Definition of rotation angle: alpha = alpha + angle, 
  * thus rotating the line counter clockwise for angle > 0.
  */
  void rotate(double angle);

  //==================================================
  //              PROPERTY ACCESSORS
  //==================================================
 
  /** x-coordinate of start point   */
  double xS() const;

  /** y-coordinate of start point     */
  double yS() const;

  /** x-coordinate of end point     */
  double xE() const;

  /** y-coordinate of end point     */
  double yE() const;

  /** x-coordinate of center point of line */
  double xC() const;

  /** y-coordinate of center point of line */
  double yC() const;

  /* z-coordinate of the line assuming the line is horisontal. */
  double z() const;

  /**
   * Angle of the line [rad]
   * The angle is initially given (when calling set(...)) [-pi,pi), 
   * where the +-pi ambigouity is resolved by defining the angle 
   * from start to end point of the line
   */
  double theta() const;

  /**
   * Angle perpendicular to the line     
   * The angle is given [0, 2pi) when calling set(...)
   */
  double alpha() const;

  /**
   * Perpendicular distance to the line
   * This parameter is >=0
   */
  double c() const;

  /**
   * Tangential distance to the line center
   * The sign of this parameter tells us where the 
   * center of the line is. 
   * xC = c*cos(alpha)+d*cos(theta)
   */
  double d() const;

  /** Half-length of the line */
  double h() const;

  /** The length of the line */
  double length() const;

  /** 
   * The covariance matrix for the line parameters. It is up to the
   * user to know how many of the paramaters that are valid. The
   * matrix can be 4x4 at most 
   */
  Eigen::MatrixXd& R() { return m_R; }

  void print(std::ostream &os);

  /** 
   * Set the end point type for this line, i.e. have we seen none, one
   * or both of the end points and thus have uncertainty for these.
   */
  void setEndPointModel(int m) { m_EndPtModel = m; }

  /**
   * Get the end point model
   */
  int getEndPointModel() const { return m_EndPtModel; }

  /** 
   * Set a pointer to some kind of source for the line 
   */
  void setSource(void *s) { m_Source = s; }

  /** 
   * Get a pointer to the source object for the lines. It is up to
   * the user to known what kind of object this is
   */
  void* getSource() { return m_Source; }

  /**
   * Use this function to mark the line with a key
   */
  void setKey(long key) { m_Key = key; }

  /**
   * Use this function to mark the line w
   */
  long getKey() const { return m_Key; }

  void setWeight(double w) { m_Weight = w; }
  double getWeight() const { return m_Weight; }

private:
  /** x coordinate of start point */
  double m_Xs;

  /** y coordinate of start point */
  double m_Ys;

  /** x coordinate of end point */
  double m_Xe;

  /** y coordinate of end point */
  double m_Ye;

  /** x coordinate of centre point */
  double m_Xc;

  /** y coordinate of centre point */
  double m_Yc;

  /** The z coordinate of the line */
  double m_Z;

  /** The angle/direction of the line */
  double m_Theta;

  /** Angle perpendicular to the line */
  double m_Alpha;

  /** The perpendicular distance to the line */
  double m_C;

  /** The tangential distance to the line */
  double m_D;

  /** The half-length of the line */
  double m_H;

  /** The uncertainty in the line params */
  Eigen::MatrixXd m_R;

  /** The end points model */
  int m_EndPtModel;

  /**
   * Key/identifier for the line
   */
  long m_Key;

  /** Weight that could be for example the probability of the line,
      the number of points used to build it, etc*/
  double m_Weight;

  /** 
   * Pointer to something that represents the source for the
   * line. Could for example be a laser scan object.
   */
  void *m_Source;

};

inline double RedundantLine2DRep::xS() const { return m_Xs; }

inline double RedundantLine2DRep::yS() const { return m_Ys; }

inline double RedundantLine2DRep::xE() const { return m_Xe; }

inline double RedundantLine2DRep::yE() const{ return m_Ye; }

inline double RedundantLine2DRep::xC() const{ return m_Xc; }

inline double RedundantLine2DRep::yC() const{ return m_Yc; }

inline double RedundantLine2DRep::z() const{ return m_Z; }

inline double RedundantLine2DRep::theta() const { return m_Theta; }

inline double RedundantLine2DRep::alpha() const { return m_Alpha; }

inline double RedundantLine2DRep::c() const { return m_C; }

inline double RedundantLine2DRep::d() const { return m_D; }

inline double RedundantLine2DRep::h() const { return m_H; }

inline double RedundantLine2DRep::length() const { return 2.0*m_H; }

inline void RedundantLine2DRep::setZ(double z) { m_Z = z; }

}; // namespace HSS

std::ostream& operator<<(std::ostream& os, const HSS::RedundantLine2DRep& l);


#endif // HSSRedundantLine2DRep_hh

