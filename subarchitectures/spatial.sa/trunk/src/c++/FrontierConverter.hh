//
// = FILENAME
//    FrontierConverter.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt, Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt, 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef FrontierConverter_hh
#define FrontierConverter_hh

#include "Navigation/NavController.hh"
#include "Navigation/LocalGridMap.hh"
#include "Navigation/FrontierFinder.hh"
#include "Navigation/NavGraph.hh"
#include "Navigation/NavGraphNode.hh"
#include "Navigation/NavGraphGateway.hh"
#include "Navigation/LocalMap.hh"

#include <list>

namespace Cure {

/**
 *
 * @author Patric Jensfelt, Kristoffer Sjöö
 * @see NavController
 * @see LocalMap
 * @see LocalGridMap
 */
class FrontierConverter: public NavControllerEventListener,
    public NavGraphEventListener {
public:
  /// List from fronts to choose between
  std::list<Cure::FrontierPt> m_Fronts;

  /// Number of frontiers classified as unreachable in m_Fronts
  int m_NumUnreachble;

  /// Number of frontiers skipped because they cross a gateway
  int m_NumGatewaySkipped;

  /// Iterator pointing to the current frontier we are heading for
  std::list<Cure::FrontierPt>::iterator m_CurrFront;

  /// Id used when registering the motion task to the
  /// NavController. Will be incremented at before calling
  /// NavController::execPath in startNextExplorationStep. This is
  /// done only if a froniter to go to is found.
  int m_NavCtrlTaskId;

public:

  /**
   * Constructor
   *
   * @param navCtrl reference to the NavController that takes care of moving
   * @param lgm reference to the LocalGridMap used for finding the
   * frontiers and the paths to them
   */
  FrontierConverter(NavController &navCtrl, LocalGridMap<unsigned char> &lgm);

  /**
   * Destructor
   */
  ~FrontierConverter();

  /**
   * Use this function to add an object from the list of objects being
   * called when areas are explored etc
   * 
   * @param l objects that want to listen to events
   */
  void addEventListener(FrontierConverterEventListener *l);

  /**
   * Use this function to remove an object from the list of objects
   * being called when areas are explored etc
   * 
   * @param l objects that want to listen to events
   */
  void delEventListener(FrontierConverterEventListener *l);

  /**
   * Call this function to start next exploration subtask. It will
   * extract frontiers and pick one to go to. It will use the provided
   * NavController to execute the motion and access the NavGraph.
   *
   * @param taskId id that will be sent back with the explorationDone
   * event
   *
   * @return 0 if froniter was found, 1 if no frontier was found and 2
   * if no froniter within this area was found (only used if
   * configured to stay with the same room).
   *
   * @see FrontierConverter::stopExploration
   */
  int startNextExplorationStep(int taskId, int explAreaId = -1);

  /**
   * Use this function to specify the min and max size of frontiers to
   * extract.
   *
   * @param min The minimum width for a frontier
   * @param max The maximum width for a frontier before it is cut in pieces
   */
  void setDesiredFrontSize(double min, double max) {
    m_MinFrontWidth = min;
    m_MaxFrontWidth = max;
  }

  /**
   * Use this function to specify if the exploration is supposed to be
   * confined by gateways. This way you can have the robot explore
   * within a single room first.
   *
   * @param val param that shoul dbe true to let gateways bound the
   * exploration.
   */
  void setExplorationConfinedByGateways(bool val) {
    m_PassGateways = !val;
  }

  /**
   * Use this function to stop exploration. Unless you call
   * startNextExplorationStep no missions will be started, but the
   * module can still interfer as it registers to events from the
   * NavGraph for new gateways being detected and could potentially
   * call a startNextExplorationStep if it is active if the taskIds
   * match
   */
  void stopExploration();

  /**
   * @return true if exporation is turned on
   */
  bool isActive() const {
    return m_Active;
  }

  /**
   * Function that traps task done from NavController
   */
  virtual void doneTask(int taskID);

  /**
   * Function that traps task aborted from NavController
   */
  virtual void abortTask(int taskID);

  /**
   * Function that traps task failure from NavController
   */
  virtual void failTask(int taskID, int error);

  /**
   * Function that traps events where new gateways are created
   */
  void newGateway(NavGraphGateway &gw);

protected:
  /**
   * This function checks whether or not a gateway is passed for the
   * specified path.
   *
   * @param path the list of cells indices from m_PathGrid that define the path
   * @param gws list of pointers to gateways in the current area. This is used
   * to limited the search for gateway crossings
   *
   * @return true if the path passes a gateway, else false
   * @see FrontierConverter::m_PathGrid
   */
  bool pathIsThroughGateway(Cure::ShortMatrix &path, std::list<
      Cure::NavGraphGateway*> &gws, Cure::NavGraph &graph);
  bool pathIsThroughGateway2(Cure::ShortMatrix &path, std::list<
      Cure::NavGraphGateway*> &gws, Cure::NavGraph &graph);

  /**
   * Use this function to compare a certain Frontier with the list of
   * frontiers that could not be reached.
   *
   * @param f reference to a FrontierPt
   * @return true if the frontier in question is unreachable, else false
   */
  bool frontierUnreachable(Cure::FrontierPt &f);

  /**
   * Function to call when exploration finished to report this to
   * event listeners
   *
   * @param taskId id of the exploration task that was done
   * @param status defined in FrontierConverterEventListener
   * @see FrontierConverterEventListener::explorationDone
   */
  void reportExplorationDone(int taskId, int status);

protected:
  /// List of event listeners
  std::list<FrontierConverterEventListener*> m_EventListeners;

  /// Reference to the NavController that will do the moving
  Cure::NavController &m_NavCtrl;

  /// Reference to LocalGridMap with obstcales, free and unknown
  Cure::LocalGridMap<unsigned char> &m_LGMap;

  /// Object that helps us find frontiers in the LocalGridMap
  Cure::FrontierFinder<unsigned char> *m_FrontFinder;

  /// Binary grid that is 0 for foree space and 1 for occupied
  Cure::BinaryMatrix m_NonFreeSpace;

  /// Version of m_NonFreeSpace where all real obstacles (not unknown)
  /// have been expanded to the size of the robot so that the robot
  /// can be treated as a point
  Cure::BinaryMatrix m_PathGrid;

  /// List of froniters that are unreachble
  std::list<Cure::FrontierPt> m_Unreachable;

  /// The minimum width for a frontier
  double m_MinFrontWidth;

  /// The maximum width for a frontier before it is cut in pieces
  double m_MaxFrontWidth;

  /// true when the explorer is active
  bool m_Active;

  /// True if exploration can go beyond a gateway
  bool m_PassGateways;

  /// ID of the area that should be explored.
  int m_ExplAreaId;

  /// Exploration task id that for the current exploration. Note that
  /// is is different from the m_NavCtrlTaskId which is the id for the
  /// sub tasks used within the exploration
  int m_ExplorationTaskId;
}; // class FrontierConverter

}
; // namespace Cure

#endif // FrontierConverter_hh
