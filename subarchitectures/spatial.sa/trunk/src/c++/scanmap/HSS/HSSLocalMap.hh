//
// = FILENAME
//    HSSLocalMap.hh
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

#ifndef HSSLocalMap_hh
#define HSSLocalMap_hh

#include <vector>

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "Eigen/Core"

#include "HSSMapRobotPose.hh"
#include "HSSMapScan2D.hh"
#include "HSSMapDoor.hh"
#include "HSSMeas.hh"
#include "HSSDoorExtractor.hh"
#include "HSSMapTransformation.hh"

namespace HSS {

/**
 * This class holds the data for a local map
 *
 * @author Patric Jensfelt
 * @see
 */
class LocalMap {

friend class Map;

public:

  /**
   * Will create a new local map with a robot pose as the first map
   * object
   */
  LocalMap(long id);

  ~LocalMap();

  void setLocalizeOnly(bool v) { m_LocalizeOnly = v; }

  /// Clears the map
  void finit();

  /**
   * This function should always be called when you create a new local
   * map to add the first reference scan which defines the origin of
   * the new frame of reference. This function will add the the robot
   * pose and a ref.scan to the map and perform an update.
   */
  void init(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan);

  /**
   * Save the state vector, state data (scans and door width) and
   * covariance matrix. The file is stored in ASCII and has the
   * following structure
   *
   * M (number of local maps in total)
   * id (map id)
   * N (number of state variables)
   * n (number of map objects)
   * InitScanID
   * 0 (if robot pose) x y a 
   * 1 (if scan) x y a id n_pts min_theta max_theta min_range max_range range_sigma tv_sec tv_usec odom.x odom.y odom.a r1 a1 v1 r2 a2 v2 ...
   * 2 (if door) x y a width
   * P11 P12 P13 P14 .... P1N-1 P1N
   * P22 P23 P24 P25 .... P2N (saving upper right triangle)
   * ....
   */  
  void save(int totMaps, const std::string &mapfile);

  /**
   * Initialize the map from a file which contains the map
   * information.
   */
  void load(const std::string &mapfile, const Eigen::Vector3d &xsR);

  /**
   * Get the robot pose in the local frame of reference
   */
  Eigen::Vector3d getXrL() const;

  /**
   * Get the covariance of the robot pose in the local frame of reference
   */
  Eigen::Matrix3d getPrrL() const;

  /**
   * Get the robot pose in the "global" frame of reference as defined
   * by setGlobalTransfT
   */
  Eigen::Vector3d getXrG() const;

  //double getMinDistToAddRefScan() const { return m_MinDistToAddRefScan; }
  //double getMinAngleToAddRefScan() const { return m_MinAngleToAddRefScan; }

  /**
   * Get the covariance of the robot pose in the global frame of
   * reference as defined by setGlobalTransfT/P.
   */
  Eigen::Matrix3d getPrrG() const;

  double getAuxValue() const { return m_AuxValue; }
  void setAuxValue(double v) { m_AuxValue = v; }

  long getAuxBitMap() const { return m_AuxBitMap; }
  void setAuxBitMap(long v) { m_AuxBitMap = v; }

  long getId() const { return m_Id; }

  /**
   * @return true if this LocalMap contains a robot pose
   */
  bool hasRobotPose() const;

  int getNumRefScans() { return m_MapScans.size(); }

  bool atBorderHeadingOut(const Eigen::Vector3d &xsR);

  /**
   * Call this to see if the robot is on its way into an existing
   * map. We define that as heading for an existing scan with the
   * requirement that that scan has |y|<m_MinDistToAddRefScan (where x
   * is the direction forward in the robot frame) and 
   * 0 < x < m_MinDistToAddRefScan
   */
  bool atBorderHeadingIn(const Eigen::Vector3d &xsR);

  /**
   * @return true if the prediction was made, else false which is
   * typically because this localmap does not contain a robotpose.
   */
  bool predict(const Eigen::Vector3d &delta, const Eigen::Matrix3d &Q);

  void updateScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan);

  void updateDoor(const Eigen::Vector3d &xsR, 
                  HSS::DoorExtractor &doorExtractor);

  /**
   * Add the current scan as a reference scan unless it has been added
   * already. Use this function when you want to force a scan to be a
   * reference scan. Can be used for example when you change to a new
   * local map for the first time and you want to establish a
   * transformation connection (having the same scan in both maps).
   *
   * NOTE: It is crucial that this scan actually comes from the
   * current position. Otherwise you may destroy the whole map
   */
  void addRefScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan);

  /**
   * Call this function to reset the robot position and clear the
   * correlations with the rest of the landmarks.
   */
  void resetRobotPoseState(const Eigen::Vector3d &Xr,
                           const Eigen::Matrix3d &Prr);

  void setUniformColored(bool v) { m_UniformColored = v; }
  
  /**
   * Specify a transformation that takes the coordinates in this map
   * into the global frame
   */
  void setGlobalTransfT(const Eigen::Vector3d &T) { m_GlobalT = T; }
  Eigen::Vector3d getGlobalTransfT() const { return m_GlobalT; }

  /**
   * Specify a covariance for the transformation that takes the
   * coordinates in this map into the global frame
   */
  void setGlobalTransfP(const Eigen::Matrix3d &P) { m_GlobalP = P; }
  Eigen::Matrix3d getGlobalTransfP() const { return m_GlobalP; }

  int getScanIndexFromId(int id) const;

  /**
   * Use this function to display the content of this local map in
   * peekabot. You can specifif if you want to display the map in the
   * global frame of reference or the local one.
   */
  void displayMap(peekabot::GroupProxy &peekabotRoot, bool globalCoords,
                  bool displayLabels);

protected:
  void extendStateScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan,
                       double variance);

  void extendStateDoor(const Eigen::Vector3d &xdS, const Eigen::Vector3d &xsR, 
                       double width);

  void extendState(MapObject *mapObject, const Eigen::Vector3d &xnW,
                   double variance);

  bool correct(const Eigen::Vector3d &xsR, HSS::Meas &z);

  void setCorridorR(Eigen::Matrix3d &R, double corrDirNewR, 
                    double angRob, double angLandm);

  /**
   * Return the minimum distance between the scanner and a reference
   * scan
   */
  double minScanner2ScanDist(const Eigen::Vector3d &xsR) const;

  /**
   * @pram mapScan the map scan 
   * @pram xsR scanner pose in robot frame
   * @pram newScan the new scan
   * @pram xns scanner pose for new scan in (local) world frame
   */
  double estimatedOverlap(const HSS::MapScan2D &mapScan, 
                          const HSS::CandScan2D &newScan,
                          const Eigen::Vector3d &xns);

protected:
  long m_Id;

  long m_InitScanId;

  /// The EKF state vector
  Eigen::VectorXd m_X;

  /// The EKF covariance matrix
  Eigen::MatrixXd m_P;

  /// Transformations to other local maps
  std::vector<HSS::MapTransformation*> m_Transfs;

  double m_StdMeasXY;
  double m_StdMeasDeg;

  // The reference scans in the map
  std::vector<MapScan2D*> m_MapScans;

  // The door objects in the map
  std::vector<MapDoor*> m_MapDoors;

  // All map objects in one vector
  std::vector<MapObject*> m_MapObjects;

  bool m_UniformColored;

  Eigen::Vector3d m_GlobalT;
  Eigen::Matrix3d m_GlobalP;

  //double m_MinDistToAddRefScan;
  //double m_MinAngleToAddRefScan;
  double m_MinMaxOverlapToAddRefScan;

  bool m_Localized;

  long m_AuxBitMap;
  double m_AuxValue;

  bool m_LocalizeOnly;
};

};

#endif // HSSLocalMap_hh
