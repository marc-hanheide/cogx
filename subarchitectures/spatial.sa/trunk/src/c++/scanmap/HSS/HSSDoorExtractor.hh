//
// = FILENAME
//    HSSDoorExtractor.h
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1999 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSDoorExtractor_hh
#define HSSDoorExtractor_hh

#include "HSSutils.hh"
#include "HSSScan2D.hh"
#include "HSSLsqLineExtractor.hh"

#ifndef DEPEND
#include <list>
#include <sys/time.h>
#endif

namespace HSS {

/**
 * This is a class used for extracting doors from x,y data acquired
 * from some sensor, preferrable with high angular resolution, which
 * means that it is likely to be from a laser. It has its own object
 * to perform least squares extracttion as it is very important that
 * the lines and the jump points etc are extratced form the same set
 * of data.
 *
 * Several different models are used for the doors making it easy to 
 * use only the models that fit you.
 *
 * @author Patric Jensfelt
 */
class DoorExtractor {
public:

  enum Algorithms {
    //EXTR_INSIDE    = 1<<0,   // findInsideDoor
    EXTR_2PAR      = 1<<1,   // findParLinesDoors
    EXTR_THICK2PAR = 1<<2,   // findThickWallDoor2ParLines
    EXTR_THICK3LIN = 1<<3,    // findThickWallRight/LeftDoor3Lines 
    //EXTR_IN2LINES  = 1<<4,   // findRight/LeftDoor2Lines
    //EXTR_OUT2LINES = 1<<5,   // findOutRight/LeftDoor2Lines
    //EXTR_3LINES    = 1<<6,   // findDoor3Lines
 };

  /** 
   * Constructor
   */
  DoorExtractor();

  /**
   * Destructor
   */
  ~DoorExtractor();

  /**
   * @return timestamp for last doors extracted
   */
  struct timeval timestamp() const { return m_Timestamp; }
 
  /**
   * Returns a reference to the list of doors. The doors
   * representations are that of a line. The line object contains all
   * the information that the door objects need.  The line is defined
   * so that the right side of the door is the startpoint of the line
   * (xS, yS) and the left side of the door is the endpoint (when it
   * was detected). The angle theta() is the angle from start to
   * endpoint, that if you want the direction through the door it is
   * theta - M_PI/2.
   *
   * @return list of extracted doors, might be empty
   */
  std::list<HSS::RedundantLine2DRep>& doors();

  /**
   * @return number of extracted doors in doors() list
   */
  int nDoors() const;

  /**
   * Function that does the extraction of doors. It returns the number
   * of extracted doors. One function for each door modeled is called
   * and at the end doors that are judge to result from the same
   * physical door based on different door models are fused.
   *
   * @param scan laser scan
   * @param algs specifies with a bitmap the algorithms to use (<0
   * means what is configured with setUsedAlgorithms
   * @return number of extracted doors
   */
  int extract(const HSS::Scan2D &scan, int algs = -1);
  
  void setUseAllAlgorithms() { m_Algorithms = 0xFFFF; }
  void setUsedAlgorithms(int algs) { m_Algorithms = algs; }
  int getUsedAlgorithms() const { return m_Algorithms; }
  
private: 
  /** timestamp for the data */
  struct timeval m_Timestamp;

  /** Array of x coord data */
  double* x_; 

  /** Array of y coord data */
  double* y_; 
  
  /** Array of range data, calc from x,y data */
  double* r_; 

  /** Array of angle data, calc from x,y data */
  double* a_; 

  /** Number of daa in x,y,r,a arrays */
  int nData_;

  int nDataSize_;

  /** Line extratcor using a least squares method */
  HSS::LsqLineExtractor *m_Lsq;

  /** list of extracted doors */
  std::list<HSS::RedundantLine2DRep> doors_;

  /** Indices of data points markig a jump from a short range reading
      to a long one, typically the right hand side of a door */
  std::list<int> shortLongJumps_;

  /** Indices of data points markig a jump from a long range reading
      to a short one, typically the left hand side of a door */
  std::list<int> longShortJumps_;

  /** Threshold for being a jump point [mm] */
  double jumpThreshold_;

  /** Lower limit for the door width to qualify as a door */
  double lowDoorWidth_;

  /** Upper limit for the door width to qualify as a door */
  double highDoorWidth_;

  /** The algoritms that will be used when extracting doors */
  int m_Algorithms;

  /** Returns a refenrence to the list of lines from the line extractor */
  std::list<HSS::RedundantLine2DRep>& lsqLines();

  /** Does init stuff, like creating the lineExtarctor, it is called
      form the constructor */
  void init();

  bool linesParallel(std::list<HSS::RedundantLine2DRep>::iterator &wallLineR, 
                     std::list<HSS::RedundantLine2DRep>::iterator &wallLineL);

  /**
   * Find the jump points in the data defined by the arrays x_, y_ of
   * length nData_.
   * The point of the jump will always be defined as the shorter point
   */
  void findJumps();
  
  /**
   * This medthod look for lines that are parallel and with end and
   * start points with approximately the door separation in
   * between. The points between these point must be behind the lines.
   */
  void findParLinesDoors();

  /**
   * This function specializes in finding doors in buildings with very
   * thick walls like at CVAP.
   * Model:
   * - Two lines that are close to parallel and of approximetley the
   *   wall thickness long, typically 500 mm. The right lines end point and
   *   the left ones start point is the door opening. The lines should be 
   *   close to parallel or slightly verged.
   * - To support this I require 2 jump point, one at each door post or
   *   a door leaf at one side and a jump point at the other
   */
  void findThickWallDoor2ParLines();
  
  /** 
   * This function specializes in finding doors in buildings with very
   * thich walls like at CVAP.
   * Model:
   * - 1 main wall line on the right
   * - 1 jump point at the end of the wall line
   * - 1 line from the wall thickness on the other side of the opening
   * - 1 supporting line on the left side starting close to end of thick wall
   */
  void findThickWallLeft3Lines();

  /** 
   * This function specializes in finding doors in buildings with very
   * thich walls like at CVAP.
   * Model:
   * - 1 main wall line on the left
   * - 1 jump point at the start of the wall line
   * - 1 line from the wall thickness on the other side of the opening
   * - 1 supporting line on the right side ending close to start of thick wall
   */
  void findThickWallRight3Lines();

  /**
   * This function will look for a right hand side door leaf going into the
   * room where the scan was taken. 
   * Model: 
   * - 1 door leaf where the end point is were the hinge is, length correct.
   * - 1 supporting line which angle is close to the angle of the line 
   *     between the door leaf hinge and the start of the line. 
   * - All points between door hing and start point of line lie behind 
   *     the line between the two door posts.
   */
  //void findInRightDoor2Lines();
  
  /**
   * This function will look for a left hand side door leaf going into the
   * room where the scan was taken. 
   * Model: 
   * - 1 door leaf where the start point is were the hinge is, length correct.
   * - 1 supporting line which angle is close to the angle of the line 
   *     between the start of the line and the door leaf hinge. 
   * - All points between start point of line and dor hinge lie behind 
   *     the line between the two door posts.
   */
  //void findInLeftDoor2Lines();
  
  /**
   * This function will look for a right hand side door leaf going into the
   * other room, away from the scanner.
   * Model: 
   * - 1 door leaf where the start point is were the hinge is, length correct.
   * - 1 supporting line which angle is close to the angle of the line 
   *     between the door leaf hinge and the start of the line. 
   * - All points between tip of the leaf and start point of line lie behind 
   *     the line between theses two points.
   *
   * NOTE that this is a very uncertainty door model. Use with care
   */
  //void findOutRightDoor2Lines();
  
  /**
   * This function will look for a left hand side door leaf going into the
   * other room, way from the scanner.
   * Model: 
   * - 1 door leaf where the end point is were the hinge is, length correct.
   * - 1 supporting line which angle is close to the angle of the line 
   *     between the start of the line and the door leaf hinge. 
   * - All points between start point of line and leaf tip lie behind 
   *     the line between these two points.
   *
   * NOTE that this is a very uncertainty door model. Use with care
   */
  //void findOutLeftDoor2Lines();
  
  /**
   * This function will look for 2 parallel lines with a door leaf in between
   * Model:
   * - 2 line representing the wall, separated with about the distance of 
   *     a normal door.
   * - 1 door leaf with its hinge at one of the door posts
   */
  //void findDoor3Lines();
  
  /**
   * Verifies that two points might be the two door posts of a door
   * with regards to the points inbetween being behind the two door
   * posts or very little infront. There is also a requirement that at
   * least one point in between the two door posts is very much
   * behind. The windowCheck is set to true in case you want to check
   * if it is a window that you have found.
   *
   * If you use ignoreSidePoints you need to feed in points that are
   * in fact defining the hypothetical door opening, otherside the
   * calculatation of the distance from the side will not be
   * correct. Even without this flag set the distances behind and
   * infront of the door will be a bit strange if you do not give
   * points corresponding to the door posts.
   *
   * @param xS start point x coord (right side of the door)
   * @param yS start point y coord (right side of the door)
   * @param xE end point x coord (left side of the door)
   * @param yE end point y coord (left side of the door)
   * @param ignoreSidePoints do not consider points that are on the of the door
   * @return true if door cand is believed to be a door still
   */
  bool doorOpeningVerified(double xS, double yS, 
			   double xE, double yE,
			   bool ignoreSidePoints = true);
  
  /**
   * Sometimes a door will be found by multiple methods, but should
   * not be reported twice so we clean up the list. We also set the
   * weight so tat it reflects the probability that this door is there in
   * reality.
   */
  void cleanUpDoorList();
    
  /**
   * Add a new door to the list, the door is defined by the right side
   * of teh door and the left. It is very imprtant to keep the order
   * of the points as the angle of teh door will be defined by it.
   */
  int addNewDoor(int method, double xR, double yR, double xL, double yL);
  
}; // class HSSDoorExtractor //

inline std::list<HSS::RedundantLine2DRep>&
DoorExtractor::doors()
{
  return doors_;
}

inline int
DoorExtractor::nDoors() const
{
  return int(doors_.size());
}

inline std::list<HSS::RedundantLine2DRep>&
DoorExtractor::lsqLines()
{
  return m_Lsq->lines();
}

}; // namespace HSS

#endif // HSSDoorExtractor_hh //

