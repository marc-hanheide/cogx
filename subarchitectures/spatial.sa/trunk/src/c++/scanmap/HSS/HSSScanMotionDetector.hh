//
// = FILENAME
//    HSSScanMotionDetector.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2004 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSScanMotionDetector_hh
#define HSSScanMotionDetector_hh

#include <peekabot.hh>

#ifndef DEPEND
#include <vector>
#endif

namespace HSS {

/**
 * This class helps you detect motion out of laser scans. There are no
 * constraints on the shape of the moving object. Points close by are
 * simply clustered and it is up to some other method to determine
 * what it is that is moving.
 *
 * The basic idea is to let old scans define a freespace area. When we
 * get a new scan we transform it to the frame of the old scan and
 * look for violations of the freespace. Such violations are likely to
 * be the result of motion. To avoid too many false positives we need
 * to add some margin to the free space, i.e. make it a bit smaller so
 * that mere sensor noise does not signal motion.
 *
 * @author Patric Jensfelt
 * @see
 */
class ScanMotionDetector {
public:

  /**
   * Movements can be detecting using the forward and the backward
   * algorith, The forward version is typically much more robust and
   * gives less false positives.
   */
  enum Algorithm {
    ALG_FORWARD = 0,
    ALG_BACKWARD,
  };

  /**
   * Class that holds information about one list of consecutive points
   * that move together. They do not necessarily have to have the same
   * direction vector. The Position part of the class holds the center
   * of the motion
   */
  class Movement {
  public:

    Movement()
    {
      xC = 0;
      yC = 0;
      timestamp = 0;
      nPts = 0;
      method = -1;
      state = -1;
    }

    Movement(const Movement &src)
    {
      *this = src;
    }

    void operator=(const Movement &src)
    {
      if (&src == this) return;

      xC = src.xC;
      yC = src.yC;
      timestamp = src.timestamp;
      nPts = src.nPts;
      for (int i = 0; i < nPts; i++) {
        scanIndex[i] = src.scanIndex[i];
      }
      method = src.method;
      state = src.state;
    }

    /** Center position of the moving cluster of points in the fixed frame */
    double xC;
    double yC;

    /** Time when motion was detected */
    double timestamp;

    /** Number of point that belong to this cluster */
    int nPts;

    /** Indices of the scan points that belong to this cluster */
    unsigned short scanIndex[1000]; 
    
    /** Flags that indicates what algorithm was used to detect the motion */
    int method;

    /** Variable used by users of this objects to denote the state of
        it. It is initialized to 0 here and then used by external
        programs to keep track of their progress in processing */
    int state;
  };

public:

  /**
   * Constructor
   *
   * @param useAlgorithmBackward true if you want to run the algorithm
   * backward as well to be able to detect motion away from the sensor
   * @maxNScans number of scans to store. Motion is check for between
   * first and last scan in the forward direction (-1 gives default 10)
   * @backwardRef in the backward checking mode it will use last scan
   * as the reference scan and use a scan backwardRef back in time to
   * check for violations (-1 mean maxNScan/2)
   */
  ScanMotionDetector(bool useAlgorithmBackward = true,
                        int maxNScans = -1,
                        int backwardRef = -1);

  /**
   * Destructor
   */
  ~ScanMotionDetector();

  /**
   * @return a vector with detected motions
   */
  std::vector<Movement> getMovments() const { return m_Movements; }

  /**
   * Use this function to reset the detector. This means that the
   * buffer of reference scans that it uses to compare scans agains to
   * detect motion will be cleared.
   */
  void reset();

  /**
   * Analyse a laser scan and check for movement.
   *
   * @param t timestamp
   * @param n numer of scan points
   * @param r array of range readings [m]
   * @param startAngle angle in laser frame to first range reading [rad]
   * @param angleStep angle between laser readings [rad]
   * @param xlP pose of laser scanner in the robot frame (x-coord)
   * @param ylP pose of laser scanner in the robot frame (y-coord)
   * @param alP pose of laser scanner in the robot frame (angle)
   * @param xrF pose of robot in fixed (e.g. odometry or world) frame (x-coord)
   * @param yrF pose of robot in fixed (e.g. odometry or world) frame (y-coord)
   * @param arF pose of robot in fixed (e.g. odometry or world) frame (angle)
   *
   * @return true if motion was detected, else false
   */
  bool checkForMotion(double t,
                      int n, double *r, double startAngle, double angleStep,
                      double xlR, double ylR, double alR,
                      double xrF, double yrF, double arF);

  /**
   * Us ethis function to tell if non-human like movement measurement
   * should be removed, or rather attempted to be removed
   */
  void useNonhumanRemoval(bool v) { m_TryRemoveNonhumanMotion = v; }

  /**
   * Use this function to set the parameters that defines the how
   * non-human like movement measurements are removed.
   * 
   * @param maxPtDistHumanSegment
   * @param maxHumanSize
   * @param minNumPtsOnHuman
   */
  void setNonhumanParams(double maxPtDistHumanSegment = 0.2,
                         double maxHumanSize = 0.6,
                         int minNumPtsOnHuman = 1)
  {
    m_MaxPtDistHumanSegment = maxPtDistHumanSegment;
    m_MaxHumanSize = maxHumanSize;
    m_MinNumPtsOnHuman = minNumPtsOnHuman;
  }

  /**
   * Display detected motion centers in RoboLook server 
   */
  void displayMotion(peekabot::GroupProxy &refroot);

  /** Display free space boundary in RoboLook server */
  void displayFreeSpace(peekabot::GroupProxy &refroot,
                          bool backwardFreeSpace = false,
                          bool forwardFreeSpace = true);

  /** displays all reference scan i RoboLook */
  void displayAllReferenceScans(peekabot::GroupProxy &refroot);
  
protected: 

  /**
   * Add a new scan to the memory. If the memory is full we will
   * overwrite the oldest scan.
   *
   * @param t timestamp
   * @param n numer of scan points
   * @param r array of range readings [m]
   * @param startAngle angle in laser frame to first range reading [rad]
   * @param angleStep angle between laser readings [rad]
   * @param xlF pose of laser in the fixed frame (x-coord)
   * @param ylF pose of laser in the fixed frame (y-coord)
   * @param alF pose of laser in the fixed frame (angle)
   */
  void renewFreeSpaceModel(double t, int n, double *r, 
                           double startAngle, double angleStep,
                           double xlF, double ylF, double alF);

  /**
   * Compare the last scan with the memory to check for motion
   *
   * If predictForward is set an attempt will be made to track the
   * motion forward in time to the last scan.
   *
   * @param refScan index of reference scan
   * @param predictForward if true motion will also be search for by 
   *        forward prediction.
   */
  void checkForMotion(int refScan, int scan,
                      bool predictForward = false);

  /**
   * Function that takes a motion detected in a certain scan and
   * brings it up to the last scan frame time. This makes it possible
   * to run the detection algorithm backward in time, i.e. compare and
   * older scan with a new one. This is necessary to detect motion
   * that is straight away from the laser scanner.
   */
  bool predictMotionForward(Movement &m, int scan);

  /**
   * Calculate the center of mass for the movement points from a
   * certain scan.
   *
   * @return true if the cluster is dense enough to be considered a cluster
   */
  bool calcCenterOfMass(Movement &m, int scan,
                        unsigned short *predAcc = 0);


  /**
   * Call this function to try to remove motion which corresponds to
   * non-humans
   */
  void tryRemoveNonhumanMotion();

protected:

  std::vector<Movement> m_Movements;

  /** The maximum distance at which we can detect motion */
  double m_MaxDetectableRange;

  /** The minimum number of consecutive points that need to violate
      the freespace of the reference scan to classify it as motion */
  int m_MinViolationPts;

  /** 
   * The closer the points are that are violating the freespace the
   * more points we demand to avoid triggering too many false alarms
   * that might come from table legs etc that are only sometimes
   * detected and thus will make th efreespace "flicker" in that
   * area. This parameters tell how many extra points are needed to
   * trigger motion detion per meter closer to the scanner from
   * maxDetectableRange_ at which point minViolationPts_ are needed.
   */
  double m_ViolationPtsPerM;
  
  /** 
   * If the depth discontinuity is larger than this distance we split
   * the cluster of points so that we do not cluster many different objects
   */
  double m_MaxDepthDiff;

  /**
   * true if we check for motion also by using the last scan as the
   * reference and use on old one to look for motion violations.
   */
  bool m_UseAlgorithmBackward;

  /** 
   * When we search backwards we use the last scan as the reference
   * scan and a scan this far back as the scan for which we check for
   * violations.
   */
  int m_BackwardRef;

  /** 
   * There are often problem with false positives at the border of the
   * scan caused by glass. We can therefore skip some of the first
   * points on each side of the scan.
   */
  int m_SideSkip;

  /**
   * Struct that is used to define the reference scans
   */
  typedef struct {
    double t;
    double xL;
    double yL;
    double aL;
    double aStart;
    double aStep;
    int n;
    float r[1000];
    float rRaw[1000];
    float xRaw[1000];
    float yRaw[1000];

    /** 
     * Points that have been classified as moving when running
     * algorithm forward. These are used to skip points when looking
     * for motion with the backward algorith as we already know that
     * they were moving 
     */
    int nForwardMotionPts;
    unsigned short forwardMotionPts[1000];
  } FreeSpaceScan;

  /** The free space scans */
  int m_MaxNScans;
  int m_NScans;
  FreeSpaceScan *m_FreeSpace;

  // Index of last reference scan
  int m_LastScan;

  /** 
   * When tracking how points move when using the backward algorithm
   * we need to keep track of how many points are mapped to each new
   * point to be able to correctly calculate the the center of mass
   */
  unsigned short m_PredAcc[1000];

  double m_TrigLookupRes;
  int m_TrigLookupSize;
  double *m_CosLookup;
  double *m_SinLookup;

  bool m_TryRemoveNonhumanMotion;

  // The max distance between two points to put two points
  // into the same segement when trying to find the size of a human
  double m_MaxPtDistHumanSegment;

  // The max allowed distance between first and last point in a segment
  double m_MaxHumanSize;

  // The minimum number of points on the human to say it is a human
  int m_MinNumPtsOnHuman;

}; // class ScanMotionDetector

}; // namespace HSS

#endif // HSSScanMotionDetector_hh

