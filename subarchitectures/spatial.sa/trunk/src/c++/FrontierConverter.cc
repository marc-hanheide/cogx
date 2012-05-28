// = FILENAME
//    FrontierExplorer.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "Navigation/FrontierExplorer.hh"

#include "Utils/CureDebug.hh"
#include "Utils/HelpFunctions.hh"
#include "Transformation/Pose3D.hh"

#include <algorithm>  // find for list
#include <iostream>

namespace Cure {

//===========================================================================

FrontierExplorerEventListener::FrontierExplorerEventListener(
    const std::string &n) {
  m_Name = n;
}

FrontierExplorerEventListener::~FrontierExplorerEventListener() {
}

//===========================================================================

FrontierExplorer::FrontierExplorer(Cure::NavController &navCtrl, LocalGridMap<
    unsigned char> &lgm) :
  NavControllerEventListener("FrontierExplorer"), NavGraphEventListener(
      "FrontierExplorer"), m_NumUnreachble(0), m_NumGatewaySkipped(0),
      m_NavCtrlTaskId(1000), m_NavCtrl(navCtrl), m_LGMap(lgm), m_MinFrontWidth(
          0.8), m_MaxFrontWidth(2.0), m_Active(false), m_PassGateways(false),
      m_ExplorationTaskId(-1) {
  m_CurrFront = m_Fronts.end();

  m_FrontFinder = new FrontierFinder<unsigned char> (lgm);

  m_NavCtrl.addEventListener(this);
  m_NavCtrl.m_Graph.addEventListener(this);
}

FrontierExplorer::~FrontierExplorer() {
}

void FrontierExplorer::addEventListener(FrontierExplorerEventListener *l) {
  // Make sure that this is not already a registered listener
  if (find(m_EventListeners.begin(), m_EventListeners.end(), l)
      != m_EventListeners.end()) {
    CureCERR(30) << l->getName() << " already added\n";
    return;
  }

  m_EventListeners.push_back(l);
}

void FrontierExplorer::delEventListener(FrontierExplorerEventListener *l) {
  std::list<FrontierExplorerEventListener*>::iterator li;
  li = find(m_EventListeners.begin(), m_EventListeners.end(), l);
  if (li != m_EventListeners.end()) {
    m_EventListeners.erase(li);
    return;
  }

  CureCERR(30) << l->getName() << " not registered\n";
}

void FrontierExplorer::stopExploration() {
  CureCERR(30) << "Stop exploration\n";
  m_Active = false;
  m_ExplorationTaskId = -1;
}

int FrontierExplorer::startNextExplorationStep(int taskId, int explAreaId) {
  bool firstExplorationStep = (m_Active == false);

  // If areaId was given, explore in this one.
  if (explAreaId >= 0)
    m_ExplAreaId = explAreaId;

  CureCERR(30) << "startNextExplorationStep(" << taskId << ") first="
      << int(firstExplorationStep) << std::endl;

  m_Active = true;
  m_Fronts.clear();
  m_NumUnreachble = 0;
  m_NumGatewaySkipped = 0;
  m_ExplorationTaskId = taskId;

  if (m_ExplorationTaskId < 1) {
    CureCERR(20) << "\n\nWARNING: You should use taskId > 0, not " << taskId
        << "\n\n";
  }

  CureCERR(60) << "Looking for fronts\n";
  int nf = m_FrontFinder->findFrontiers(m_MinFrontWidth, m_MaxFrontWidth,
      m_Fronts);
  CureCERR(60) << "Found " << nf << " fronts\n";

  // We make the number of columns of the BinaryMatrices a multiple
  // of 32 so that we get the benefit of the representation. This
  // will only really do anything the first time we get here since
  // the LocalGridMap is of constant size.
  int rows = 2 * m_LGMap.getSize() + 1;
  int cols = ((2 * m_LGMap.getSize() + 1) / 32 + 1) * 32;
  m_NonFreeSpace.reallocate(rows, cols);

  // First we create a binary matrix where all cells that
  // corresponds to know obstacles are set to "1".
  m_NonFreeSpace = 0;
  for (int x = -m_LGMap.getSize(); x <= m_LGMap.getSize(); x++) {
    for (int y = -m_LGMap.getSize(); y <= m_LGMap.getSize(); y++) {
      if (m_LGMap(x, y) == '1') {
        m_NonFreeSpace.setBit(x + m_LGMap.getSize(), y + m_LGMap.getSize(),
            true);
      }
    }
  }

  // Grow each occupied cell to account for the size of the
  // robot. We put the result in another binary matrix, m_PathGrid
  m_PathGrid = 0;
  m_NonFreeSpace.growInto(m_PathGrid, 0.5 * m_NavCtrl.getRobotWidth()
      / m_LGMap.getCellSize(), true);

  // We treat all unknown cells as occupied so that the robot only
  // uses paths that it knowns to be free. Note that we perfom this
  // operation directly on the m_PathGrid, i.e. the grid with the
  // expanded obstacle. The reasoning behind this is that we do not
  // want the unknown cells to be expanded as well as we would have
  // to recalculate the position of the frontiers otherwise, else
  // they might end up inside an obstacle (could happen now as well
  // from expanding the occupied cell but then it is known not to be
  // reachable).
  for (int x = -m_LGMap.getSize(); x <= m_LGMap.getSize(); x++) {
    for (int y = -m_LGMap.getSize(); y <= m_LGMap.getSize(); y++) {
      if (m_LGMap(x, y) == '2') {
        m_PathGrid.setBit(x + m_LGMap.getSize(), y + m_LGMap.getSize(), true);
      }
    }
  }

  /*// Prepare a list of gateways in this area so that we can stop
   // searching though gateways if configured to do so
   std::list<Cure::NavGraphGateway*> gatewayList;
   if (m_PassGateways == false) {
   long aid = m_NavCtrl.m_Graph.getCurrentAreaId();
   gatewayList = m_NavCtrl.m_Graph.getGatewaysInArea(aid);
   } */

  // Prepare a list of gateways in this area so that we can stop
  // searching though gateways if configured to do so
  std::list<Cure::NavGraphGateway*> gatewayList;
  Cure::Pose3D cp = m_NavCtrl.getPose();

  if (m_PassGateways == false) {
    NavGraphNode* closest_node = m_NavCtrl.m_Graph.getClosestNode(cp.getX(),
        cp.getY(), 0, 3); // 0 us angle and 3 is maximum distance
    long aid;
    if (m_ExplAreaId >= 0)
      aid = m_ExplAreaId; // was closest_node->getAreaId(); but failed when close to doorway
    else
      aid = closest_node->getAreaId();
    gatewayList = m_NavCtrl.m_Graph.getGatewaysInArea(aid);
    std::cout << "AREA:" << aid << std::endl;
  }

  if (nf > 0) {
    // Select the closest frontier
    std::list<Cure::FrontierPt>::iterator fi;
    double minCost = 1e10;
    Cure::ShortMatrix nextPath;
    m_CurrFront = m_Fronts.end();
    Cure::Pose3D cp = m_NavCtrl.getPose();
#ifdef FE_PRINT_FRONTS
    std::cerr << "Frontiers:\n";
#endif
    for (fi = m_Fronts.begin(); fi != m_Fronts.end(); fi++) {
#ifdef FE_PRINT_FRONTS
      std::cerr << "x=" << fi->getX()
      << " y=" << fi->getY()
      << " ";
#endif

      if (frontierUnreachable(*fi)) {
#ifdef FE_PRINT_FRONTS
        std::cerr << "UNREACHABLE\n";
#endif
        fi->m_State = FrontierPt::FRONTIER_STATUS_UNREACHABLE;
        m_NumUnreachble++;
        continue;
      }

      double cost = 2e10;

      double ga = atan2(fi->getY() - cp.getY(), fi->getX() - cp.getX());
      double da = Cure::HelpFunctions::angleDiffRad(cp.getTheta(), ga);

      // Use m_PathGrid to find path to the frontier
      int r1, c1, r2, c2;
      if (m_LGMap.worldCoords2Index(cp.getX(), cp.getY(), r2, c2) == 0
          && m_LGMap.worldCoords2Index(fi->getX(), fi->getY(), r1, c1) == 0) {
        c1 += m_LGMap.getSize();
        r1 += m_LGMap.getSize();
        c2 += m_LGMap.getSize();
        r2 += m_LGMap.getSize();

        Cure::ShortMatrix path;
        double d = (m_PathGrid.path(r1, c1, r2, c2, path, 20
            * m_LGMap.getSize()) * m_LGMap.getCellSize());
        if (d > 0) {

          if (!m_PassGateways && pathIsThroughGateway(path, gatewayList,
              m_NavCtrl.m_Graph)) {
#ifdef FE_PRINT_FRONTS
            std::cerr << " SKIPPING PATH THROUGH DOOR\n";
#endif
            fi->m_State = FrontierPt::FRONTIER_STATUS_GATEWAYBLOCKED;
            m_NumGatewaySkipped++;
            continue;
          }

          double c0 = 0; // cost for turning
          double c1 = 0; // the wider the cheaper

          // Add extra cost for turning much;
          c0 = 2.0 * fabs(da);

          // The larger the frontier is the better
          c1 = -3.0 * fi->m_Width;

          cost = d + c0 + c1;

#ifdef FE_PRINT_FRONTS
          std::cerr << " d=" << d
          << " c0=" << c0
          << " c1=" << c1
          << " tot=" << cost
          << " ";
#endif
          if (cost < minCost) {
            minCost = cost;
            m_CurrFront = fi;
#ifdef FE_PRINT_FRONTS
            std::cerr << " BEST SO FAR";
#endif
            nextPath = path;
          }

          fi->m_State = FrontierPt::FRONTIER_STATUS_OPEN;
        } else {
#ifdef FE_PRINT_FRONTS
          std::cerr << " NO PATH FOUND";
#endif
          fi->m_State = FrontierPt::FRONTIER_STATUS_PATHBLOCKED;
        }
      }

#ifdef FE_PRINT_FRONTS
      std::cerr << std::endl;
#endif
    }

    if (m_CurrFront != m_Fronts.end()) {
      m_NavCtrlTaskId++;
      m_CurrFront->m_State = FrontierPt::FRONTIER_STATUS_CURRENT;

      // Construct the path to follow. 

      // First we add final goal
      std::list<Cure::NavGraphNode> navPath;
      Cure::NavGraphNode n;
      n.setId(-42);
      n.setX(m_CurrFront->getX());
      n.setY(m_CurrFront->getY());
      n.setTheta(m_CurrFront->getTheta());
      navPath.push_front(n);

      // Then step through the path that we got from m_PathGrid and
      // add a new node every now and then
      const double nodeDist = 1.0;
      int rowStep = int(nodeDist / m_LGMap.getCellSize() + 0.5);
      for (int row = rowStep; row < nextPath.Rows; row += rowStep) {
        double x, y;
        m_LGMap.index2WorldCoords(nextPath(row, 0) - m_LGMap.getSize(),
            nextPath(row, 1) - m_LGMap.getSize(), x, y);
        n.setX(x);
        n.setY(y);
        n.setTheta(0);
        navPath.push_front(n);
      }

      CureCERR(40) << "Created path with " << navPath.size() << " nodes\n";

      m_NavCtrl.execPath(m_NavCtrlTaskId, navPath, true);
      return 0;

    } else {
      CureCERR(30) << "\n\nNo reachable fronts left\n\n";
      if (!m_PassGateways && m_NumGatewaySkipped > 0) {
        CureCERR(30) << "Skipped " << m_NumGatewaySkipped
            << " fronts outside room, still unexplored areas\n";
        if (!firstExplorationStep) {
          // We do not send the report that the exploration task is
          // done to all listeners there is no exploration at all, we
          // let the caller of this method decide what to do instead
          reportExplorationDone(m_ExplorationTaskId, 2);
          return 2;
        } else {
          stopExploration();
          return 2;
        }
      } else {
        CureCERR(30) << "No more unexplored frontiers\n";
        if (!firstExplorationStep) {
          // We do not send the report that the exploration task is
          // done to all listeners there is no exploration at all, we
          // let the caller of this method decide what to do instead
          reportExplorationDone(m_ExplorationTaskId, 1);
          return 1;
        } else {
          stopExploration();
          return 1;
        }
      }
    }
  } else {
    CureCERR(30) << "Did not find a single frontier\n";
    if (!firstExplorationStep) {
      // We do not send the report that the exploration task is
      // done to all listeners there is no exploration at all, we
      // let the caller of this method decide what to do instead
      reportExplorationDone(m_ExplorationTaskId, 1);
      return 1;
    } else {
      stopExploration();
      return 1;
    }
  }
}

bool FrontierExplorer::pathIsThroughGateway(Cure::ShortMatrix &path, std::list<
    Cure::NavGraphGateway*> &gws, Cure::NavGraph &graph) {
  if (gws.empty() || path.Rows <= 2)
    return false;

  CureCERR(50) << "Checking if node passes through door\n";

  // Find the gateway that is closest to the path
  Cure::NavGraphGateway *closest = 0;
  double minDist = 1e10;
  short r = 0, c = 0;
  std::list<Cure::NavGraphGateway*>::iterator gi;
  for (gi = gws.begin(); gi != gws.end(); gi++) {

    int xi, yi;
    m_LGMap.worldCoords2Index((*gi)->getX(), (*gi)->getY(), xi, yi);
    // Johns path are have indices that start from 0,0 in the
    // lower left corner and not in the center like the LocalGridMap
    xi += m_LGMap.getSize();
    yi += m_LGMap.getSize();

    CureCERR(50) << "Checking door at xi=" << xi << " yi=" << yi << std::endl;
    for (int row = 0; row < path.Rows; row++) {
      double d = hypot(path(row, 0) - xi, path(row, 1) - yi);
      if (d < minDist) {
        closest = *gi;
        minDist = d;
        r = path(row, 0);
        c = path(row, 1);
      }
    }
  }

  if (closest == 0) {
    CureCERR(20) << "WARNING: closest=0 should not happen!\n";
    return false;
  }

  CureCERR(50) << "Closest is node " << closest->getId() << " minDist="
      << minDist << std::endl;

  // Now check if the gateway is the closest node to the point on
  // the path closest to the gateway.
  Cure::NavGraphNode *cn;
  double xW, yW;
  m_LGMap.index2WorldCoords(r - m_LGMap.getSize(), c - m_LGMap.getSize(), xW,
      yW);
  cn = graph.getClosestNode(xW, yW, 0, -1);
  if (cn == 0) {
    CureCERR(20) << "WARNING: No closest node!!!\n";
    return false;
  }

  // Transform minDist from cells to meters
  minDist *= m_LGMap.getCellSize();

  if (cn == closest && minDist < 1.0) {
    return true;
  }

  return false;
}

bool FrontierExplorer::pathIsThroughGateway2(Cure::ShortMatrix &path,
    std::list<Cure::NavGraphGateway*> &gws, Cure::NavGraph &graph) {
  if (gws.empty() || path.Rows <= 2)
    return false;

  CureCERR(50) << "Checking if node passes through door (v2)\n";

  // Consider each segment in th epath and check if this crosses a gateway

  // Start point of the segment
  double xS, yS;
  m_LGMap.index2WorldCoords(path(0, 0) - m_LGMap.getSize(), path(0, 1)
      - m_LGMap.getSize(), xS, yS);

  for (int row = 1; row < path.Rows; row++) {
    double xE, yE;
    m_LGMap.index2WorldCoords(path(row, 0) - m_LGMap.getSize(), path(row, 1)
        - m_LGMap.getSize(), xE, yE);

    // We have a segment between (xS,yS) and (xE, yE)

    // Center of the segment
    double xC = 0.5 * (xS + xE);
    double yC = 0.5 * (yS + yE);

    // Now look for any gateway that is close enough (conservative
    // estimate) to the center of the segment (determined by the width
    // of the gateway).

    std::list<Cure::NavGraphGateway*>::iterator gi;
    for (gi = gws.begin(); gi != gws.end(); gi++) {
      if (fabs((*gi)->getX() - xC) < (*gi)->getWidth() && fabs((*gi)->getY()
          - yC) < (*gi)->getWidth()) {

        // Do more expensive check by looking if the segment crosses
        // the gateway

        if (Cure::HelpFunctions::segmentsIntersect(xS, yS, xE, yE,
            (*gi)->getX() + 0.5 * (*gi)->getWidth() * cos((*gi)->getTheta()),
            (*gi)->getY() + 0.5 * (*gi)->getWidth() * sin((*gi)->getTheta()),
            (*gi)->getX() - 0.5 * (*gi)->getWidth() * cos((*gi)->getTheta()),
            (*gi)->getY() - 0.5 * (*gi)->getWidth() * sin((*gi)->getTheta()))) {

          // The path crossed the gateway
          CureCERR(40) << "The path passes through gateway with id "
              << (*gi)->getId() << std::endl;
          return true;

        }
      }
    }

    // shift the end of the current segment to be the start of the next
    xS = xE;
    yS = yE;
  }

  return false;
}

bool FrontierExplorer::frontierUnreachable(Cure::FrontierPt &f) {
  for (std::list<Cure::FrontierPt>::iterator fi = m_Unreachable.begin(); fi
      != m_Unreachable.end(); fi++) {
    if (hypot(f.getY() - fi->getY(), f.getX() - fi->getX()) < 1.0) {
      CureCERR(50) << "Frontier unreachble, skipping it\n";
      return true;
    }
  }

  return false;
}

void FrontierExplorer::doneTask(int taskID) {
  if (m_Active == false)
    return;

  if (m_NavCtrlTaskId == taskID) {
    CureCERR(30) << "Task done\n";
    startNextExplorationStep( m_ExplorationTaskId);
  } else {
    CureCERR(30) << "TaskID did not match for doneTask " << taskID << "!="
        << m_NavCtrlTaskId << " we are no longer in exploration mode!!!\n";
    stopExploration();
  }
}

void FrontierExplorer::abortTask(int taskID) {
  if (m_Active == false)
    return;

  if (m_NavCtrlTaskId == taskID) {
    CureCERR(30) << "Task aborted, id=" << taskID << "\n";
  } else {
    CureCERR(30) << "TaskID did not match for abortTask " << taskID << "!="
        << m_NavCtrlTaskId << " we are no longer in exploration mode!!!\n";
    stopExploration();
  }
}

void FrontierExplorer::failTask(int taskID, int error) {
  if (m_Active == false)
    return;

  if (m_NavCtrlTaskId == taskID) {
    CureCERR(30) << "Navigation task failed, error=" << error << std::endl;
    int ret = startNextExplorationStep(m_ExplorationTaskId);
    if (ret != 0) {
      CureCERR(30) << "Need to report that we are done\n";
    }
  } else {
    CureCERR(30) << "TaskID did not match for failTask " << taskID << "!="
        << m_NavCtrlTaskId << " we are no longer in exploration mode!!!\n";
    stopExploration();
  }
}

void FrontierExplorer::newGateway(NavGraphGateway &gw) {
  if (m_Active == false)
    return;

  // A new gateway is created. If the exploraion is confined by
  // gateways this might mean that we have to stop here and backoff
  // and select a new goal
  if (m_PassGateways == true)
    return;

  // It also almost certainly where we are
  // right now, but we check just to make sure.
  Cure::Pose3D cp = m_NavCtrl.getPose();
  double d = hypot(cp.getY() - gw.getY(), cp.getX() - gw.getX());
  if (d > 1.0)
    return;

  CureCERR(30) << "Reached gateway, must move back\n";
  m_NavCtrl.stop();
  m_NavCtrlTaskId++;
  m_NavCtrl.backOff(m_NavCtrlTaskId, 0.5);
}

void FrontierExplorer::reportExplorationDone(int taskId, int status) {
  CureCERR(30) << "Reporting exploration task " << taskId << " done\n";

  if (m_ExplorationTaskId != taskId) {
    CureCERR(20) << "\nWARNING: task ids do not match " << m_ExplorationTaskId
        << "!=" << taskId << "!!!!!\n\n";
  }

  stopExploration();
  std::list<FrontierExplorerEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); li != m_EventListeners.end(); li++) {
    (*li)->explorationDone(taskId, status);
  }
}

} // namespace Cure
