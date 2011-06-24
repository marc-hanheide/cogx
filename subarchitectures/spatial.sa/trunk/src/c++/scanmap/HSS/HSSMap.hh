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

#ifndef HSSMap_hh
#define HSSMap_hh

#include "Eigen/Core"
#include <vector>

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "HSSCandScan2D.hh"
#include "HSSDoorExtractor.hh"
#include "HSSLocalMap.hh"
#include "HSSMapTransformation.hh"

namespace HSS {

class Map {

public:
  enum {
    STATE_EMPTY = 0,
    STATE_NORMAL,
    STATE_TRANS
  };

public:
  Map();

  ~Map();

  /**
   * Cleans up the map
   */
  void finit();

  /**
   * Initializes the map by making this scan define the original of
   * the map.
   */
  void init(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan);

  /**
   * Initialize the map from a file which contains the map
   * information. You can also supply an optional guess (0 for no
   * guess) for where the robot is now so that the robot pose can be
   * initialized, otherwise the old data stored in the file will be
   * used. Note that using the stored data only works if you know that
   * you did not move the robot in any way.
   */
  void init(const std::string &mapfileBasename, 
            const Eigen::Vector3d &xsR, 
            Eigen::Vector3d *xr=0,
            Eigen::Matrix3d *Prr=0);

  /**
   * Use this function to save the map. If there is only one map the
   * file will be called mapfileBasename000.hss. If you are using more
   * than one local map this function will save each local map in a
   * separate file called mapfileBasenameXXX.hss.
   */
  void save(const std::string &mapfileBasename);

  void setMaxLMapSize(int nScans) { m_MaxLMapSize = nScans; }

  /// @return robot pose in global frame (map 0 defines origin)
  Eigen::VectorXd getXrG() const;
  
  /// @return robot pose covariance in global frame (map 0 defines origin)
  Eigen::MatrixXd getPrrG() const;

  /**
   * Predict how the robot moves based on dead-reckoning information
   */
  bool predict(const Eigen::Vector3d &delta, const Eigen::Matrix3d &Q);

  /**
   * Update the map based on the data from a new scan
   */
  void updateScan(const Eigen::Vector3d &xsR, const HSS::CandScan2D &scan);
  
  /**
   * Update the map based on possible new door observations
   */
  void updateDoor(const Eigen::Vector3d &xsR, 
                  HSS::DoorExtractor &doorExtractor);


  /**
   * Call this function when you want to check if it is time to change
   * to a new local map. If it is time this function will in turn call
   * initTransition unless we are creating a completely new local map
   * in which case this is done directly.
   *
   * @return true if a transition was initiated.
   */
  bool checkForTransitions(const Eigen::Vector3d &xsR, 
                           const HSS::CandScan2D &scan);

  /**
   * Use this function to loop through all the local maps and set
   * the auxiliary value to v. This value can be used to for example keep
   * track of the cost value in a search.
   */
  void setAllLocalMapAuxValues(double v);

  /**
   * Use this function to loop through all the local maps and clear
   * the auxiliary bit map. This flag can be used to for example keep
   * track of which local maps you have visited in a display
   * operation.
   */
  void clearLocalMapAuxBitMaps();

  void displayMap(peekabot::GroupProxy &peekabotRoot, bool displayLabels);
  void displayLandmarksInSameFrame(peekabot::GroupProxy &peekabotRoot);
protected:

  /**
   * Look up the index in the m_LMaps vector for the local map with a
   * certain id. If the id is not found the function trows
   */
  int getIndexFromId(long id);

  /**
   * Call this function to create a new local map and create a
   * connection from the current local map to the new one.
   */
  void changeToNewLocalMap(const Eigen::Vector3d &xsR, 
                           const HSS::CandScan2D &scan);

  /**
   * Call this function to change to an existing local map and create/update a
   * connection from the current local map to the next one.
   */
  void changeToOldLocalMap(const Eigen::Vector3d &xsR, 
                           const HSS::CandScan2D &scan,
                           int nextIndex);

  /**
   * Call this function to update the information about where the
   * other maps are with respect to the current one. This is used to
   * determine if it is time to transition to a new local map or not.
   */
  void updateTransitionInfo(int fromIndex);

  /**
   * This function checks if we are entering an existing local map
   *
   * @return index of local map to enter
   */
  int enteringExistingMap(const Eigen::Vector3d &xsR);

  /**
   * Recursive function that should be called after calling
   * clearLocalMapAuxValues() and having made sure that m_LMapT has
   * the same size as m_LMaps and it should be called with the index
   * of the local map that you want to start from and cost 0 and
   * T=0. When it returns the cost (coded in the getAuxValue() in each
   * local map should give the order to transition between them
   */
  void wavePropagation(int index, double cost, const Eigen::Vector3d &T);

  /**
   * Goes through the local maps and put the position of the map
   * objects from all local maps into a common list in the same
   * coordinate system to be used for example when checking if we are
   * about to enter an existing local map. It also initializes the
   * vector with distances to the different local maps which is used
   * to keep track of transitions.
   */
  void buildInternals(int fromIndex);


  /**
   * @return true if there was a connection between maps with ids
   * fromMapId to toMapId else false
   */
  bool getTransformationBetweenMaps(long fromMapId, long toMapId,
                                    Eigen::Vector3d &T,
                                    Eigen::Matrix3d *P = 0);

  /**
   * @return true if there was a connection between maps with ids
   * fromMapId to toMapId and t gives this transformation else it
   * throws an expection.
   */
  bool getTransformationBetweenMaps(long fromMapId, long toMapId,
                                    HSS::MapTransformation *t,
                                    Eigen::Vector3d &T,
                                    Eigen::Matrix3d *P = 0);

protected:

  class OtherMapObject : public MapObject {
  public:
    // From which map does it stem
    int mapIndex;
    long mapId;

    // Pose of the object in the current frame of reference
    Eigen::Vector3d xL;

    OtherMapObject(const MapObject &src)
      :MapObject(src)
    {
      mapIndex = -1;
      mapId = -1;
      xL.setZero();
    }
  };

  enum {
    DIST_INSIDE = 0,
    DIST_LEAVING,
    DIST_OUTSIDE,
    DIST_ENTERING
  };

  class MapDistance {
  public:

    int state;

    // Index of closest scan
    int scanIndex;

    // Distance to closest ref scan
    double scanD;

    // Distance to the center of the map
    double centerD;
  };

  /**
   * Vector with the distances to the local maps
   */
  std::vector<MapDistance> m_Dists;

  /**
   * The local maps
   */
  std::vector<HSS::LocalMap*> m_LMaps;

  /**
   * Transformations that take local maps into the current frame of
   * reference which will typically be one of the local map (the
   * current one).
   */
  std::vector<Eigen::Vector3d> m_LMapsT;

  /**
   * The centers of the local maps
   */
  std::vector< Eigen::Vector3d > m_LMapsC;

  /**
   * Contains the location of map objects in the other local maps than
   * the current one
   */
  std::vector< std::vector<OtherMapObject> > m_LMapObjects;

  /**
   * The maximum number of features in the local map before we start
   * thinking about maing a new one.
   */
  int m_MaxLMapSize;

  /**
   * List with transformations
   */
  std::vector<HSS::MapTransformation*> m_Transfs;

  /**
   * Id for the next local map
   */
  long m_NextLocalMapId;

  int m_CurrLMap;

  double m_SwitchDist;
  double m_SwitchHystDist;
};

};

#endif // HSSMap_hh
