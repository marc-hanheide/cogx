// = FILENAME
//    NewTurnMotion.hh 
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2004 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef NewTurnMotion_hh
#define NewTurnMotion_hh

#include <cast/architecture/ManagedComponent.hpp>
#include <Navigation/MotionAlgorithm.hh>
#include <Navigation/LocalMap.hh>

// Forward declaration(s)
class RoboLookProxy;

namespace Cure{

/**
 * This class helps you to turn the robot in tight spots. The
 * speciality is turning when it is close to an object on the side in
 * the front or the back. It will combine translation and rotation to
 * turn around without (hopefully) bumping into the obstacles.
 *
 * @author Patric Jensfelt
 * @see
 */
class NewTurnMotion : public MotionAlgorithm {
public:

  /**
   * Constructor
   */
  NewTurnMotion(cast::ManagedComponent* sc,double securityDistance = 0.05);

  /**
   * Destructor
   */
  ~NewTurnMotion();

  /**
   * Calculate motion command
   */
  int calcMotionCommand(LocalMap &m, double aGoal, 
                        double vMax, MotionAlgorithm::MotionCmd &cmd);

  /**
   * Configures the shape of the robot from a string of parameters
   * given the type of parameterization.
   */
  bool configRobotShape(int type, const std::string &params);

  /**
   * Use a rectangular model for the robot
   */
  void setRectangularShape(double robotLength, double robotWidth, 
                           double centerOffset);

  /**
   * Use a rectangular model for the robot
   */
  void setCircularShape(double radius, double centerOffset);  

  /**
   * Define the shape of the robot using a polygon. You specify the
   * number of verticies in the polygon. You should specify an open
   * polygon, i.e. the start and end point should not be the same. The
   * function will automatically connect the first and the last point.
   */
  void setPolygonShape(int n, double *lx, double *ly);

  /**
   * Set the shape function directly
   */
  void setShapeFunction(double *E);

  /**
   * Display situation in RoboLook
   */
  void displayRL(RoboLookProxy *rlp);

  double getFrontDist() const { return frontDist_; }
  double getBackDist() const { return backDist_; }
  double gethalfWidth() const { return halfWidth_; }

private:

  /** 
   * Goes through the obstacels and checks if any are close enough to
   * consider. This means that it has to be closer to the center than
   * the diagonal of the robot plus the security distance plus some
   * extra margin. The latter is to make the solution valid even after
   * moving the robot a bit
   *
   * NOTE this function assumes that the robot pose is given by
   * getX(), getY() and getA() in the map, i.e. that moveRobot has
   * been called on the map with the current pose.
   *
   * @return true if any close points were found, else false
   */
  bool findCloseObstacles(LocalMap &m);


  /**
   * This function goes through the close by obstacles and checks for
   * a stuck situation where the robot is unable to turn because there
   * are obstacles close by both in the front and the rear.
   *
   * @return true if stuck, else false
   */
  bool inStuckSituation();

  /**
   * Check if the obstacle configuration is such that we can turn on
   * the spot
   *
   * @return true if we can simply turn (for now at least), else false
   */
  bool canTurnOnTheSpot();

  /**
   * Check the violation points to determine how to turn
   */
  bool changeToMoveMode();

  /**
   * Calculate the speed to use when we are moving forward/backward
   */
  bool calcTranslationCommand(double &vel);

  /**
   * Look for violations between the robot and close by obstacles,
   * i.e. that the obstacles are inside the security zone. The idea
   * is to call this function for different configurations to look for
   * one that is allowed
   *
   * NOTE this function assumes that findCloseObstacles has been called
   *
   * @return true if there are violations, else false
   */
  bool findViolations(double x, double y, double a);

  /**
   * Get the sector for a given angle that is assumed to be (-pi, pi]
   */
  int getSector(double a);

private:
  cast::ManagedComponent* m_sc;
  // Distance from center of the robot to the front
  double frontDist_;

  // Distance from center of the robot to the back
  double backDist_;

  // Half width of the robot
  double halfWidth_;

  // Max distance from center to the periferi of the robot (max(E_))
  double maxRadius_;

  // Zone infront or behind or to the side of the robot that must be
  // free to say that there are no obstacles in that area
  double freeZoneFB_;

  // Zone to the side of the robot that must be free to say that there
  // are no obstacles in that area
  double freeZoneS_;

  /** Security distance */
  double ds_;  

  /** Mode of motion, turn, forward, backward */
  int mode_;

  /** Help variables for when finding what sector a certain angle
      corresponds to */
  int nHalf_;
  double sectPerRad_;

  typedef struct {
    LocalMap::ObstPt *o;
    double d;
    double a;
    int s;
  } CloseObstPt;

  /** Indices of obstacle points that are close enough to consider */
  int nClosePts_;
  CloseObstPt *closePts_;
  
  /** Distance to cloest obstacle */
  double minObstDist_;

  int nViolationPts_;
  CloseObstPt *violationPts_;

  int violFrontLeft_;
  int violFrontRight_;
  int violRearLeft_;
  int violRearRight_;

  bool clearBackward_;
  bool clearForward_;

  double robX_;
  double robY_;
  double robA_;

  double goalA_;
};

}; // namespace Cure

#endif // NewTurnMotion_hh
