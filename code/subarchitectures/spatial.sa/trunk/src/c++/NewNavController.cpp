//
// = FILENAME
//    NewNavController.cc
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

#include "NewNavController.hpp"

#include <Navigation/NavGraph.hh>
#include <Navigation/LocalMap.hh>
#include <Map/PoseProvider.hh>
#include <Utils/CureDebug.hh>
#include <Utils/HelpFunctions.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Transformation/Pose3D.hh>

#ifndef DEPEND
#include <iostream>
#include <iomanip>
#include <algorithm>
#endif

using namespace Cure;

//===========================================================================

NewNavControllerEventListener::NewNavControllerEventListener(const std::string &n)
{
  m_Name = n;
}

NewNavControllerEventListener::~NewNavControllerEventListener()
{}

//===========================================================================


NewNavController::NewNavController(NavGraph &graph, LocalMap &lmap, cast::ManagedComponent* sc) 
  :m_PP(0),
   m_Graph(graph),
   m_LMap(lmap),
   m_ND(8),
   m_TaskType(TASKTYPE_STOP),
   m_TaskID(-1),
   m_LastProgressTime(-1),
   m_ProgressTimeout(-1),
   m_sc(sc),
   m_Turn(sc,0.05)
{
  m_sc->debug("NewNavController started");  


  m_InterGoalTol = 0.75;
  m_FinalGoalTol = 0.25;
  m_GatewayGoalTol = 0.4;
  m_FinalGoalTolRot = Cure::HelpFunctions::deg2rad(2);

  m_MinNonZeroV = 0;
  m_MinNonZeroW = 0;

  m_TurnAngleIntoSpeed = false;
  m_AngleGain = 0.5;

  m_MaxGotoV = 0.5;
  m_MaxGotoW = 0.5;

  m_MaxTurnV = 0.2;
  m_MaxTurnW = 0.3;

  m_MaxGatewayV = 0.3;
  m_MaxGatewayW = 0.3;

  m_BackOffV = 0.1;
  m_BackOffDist = 0.1;

  m_MaxFollowV = 0.5;
  m_MaxFollowW = 0.5;
  m_FollowBackupThreshold = 0.75;
  m_FollowTolDist = 0.5;
  m_FollowTolRot = Cure::HelpFunctions::deg2rad(15);
  m_FollowDist = 1.5;
  m_FollowBeforeTargetDist = 0.3;
  m_FollowBackOffV = 0.2;

  m_MaxTrimDist = 5;
  m_UsePathTrimming = false;
  m_MinPathLengthForTrim = 2;
  m_CanTrimGateways = false;
}

NewNavController::~NewNavController()
{
  m_sc->debug("Destructing navctrl\n");
}

int 
NewNavController::config(const std::string &configfile)
{
  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) return RETVAL_ERROR;
  
  int type;
  std::string sparams;
  if (cfg.getRobotShape(type, sparams) == 0) {
    if (m_ND.configRobotShape(type, sparams) == false) {
      m_sc->error("Failed to config robot shape\n");
      return RETVAL_ERROR;
    }
  } else {
    m_sc->error("Failed to get robot shape\n");
    return RETVAL_ERROR;
  }

  if (m_Turn.configRobotShape(type, sparams) == false) {
    m_sc->error("Failed to config robot shape\n");
    return RETVAL_ERROR;
  }

  return RETVAL_OK;
}

void
NewNavController::setMinPathLengthForTrim(unsigned int len)
{
  if (len >= 1) {
    m_MinPathLengthForTrim = len;
  } else {
    m_sc->error("WARNING: m_MinPathLengthForTrim must be >= 1, not %d\n",len);
  }
}

void
NewNavController::setMinNonzeroSpeeds(double minV, double minW)
{
  m_MinNonZeroV = minV;
  m_MinNonZeroW = minW;
}

void
NewNavController::setTurnAngleIntoSpeed(bool doit, double gain)
{
  m_TurnAngleIntoSpeed = doit;
  m_AngleGain = gain;
}

void
NewNavController::setGotoMaxSpeeds(double maxV, double maxW)
{
  m_MaxGotoV = maxV;
  m_MaxGotoW = maxW;
}

void
NewNavController::setTurnMaxSpeeds(double maxV, double maxW)
{
  m_MaxTurnV = maxV;
  m_MaxTurnW = maxW;
}

void
NewNavController::setGatewayMaxSpeeds(double maxV, double maxW)
{
  m_MaxGatewayV = maxV;
  m_MaxGatewayW = maxW;
}

void
NewNavController::setFollowMaxSpeeds(double maxV, double maxW)
{
  m_MaxFollowV = maxV;
  m_MaxFollowW = maxW;
}

void
NewNavController::turnOn()
{
  m_sc->debug("Turning on NewNavController\n");
  m_TaskType = TASKTYPE_STOP;
}

void
NewNavController::turnOff()
{
  if (m_TaskType == TASKTYPE_OFF) {
    m_sc->error("NewNavController already turned off\n");
    return;
  }

  prepForTask("turnOff");

  m_TaskType = TASKTYPE_OFF;
}

void
NewNavController::addEventListener(NewNavControllerEventListener *l)
{
  // Make sure that this is not already a registered listener
  if (find(m_EventListeners.begin(), m_EventListeners.end(), l) != 
      m_EventListeners.end()) {
    m_sc->error("%s already added\n",l->getName().c_str());
    return;
  }

  m_EventListeners.push_back(l);
}

void
NewNavController::delEventListener(NewNavControllerEventListener *l)
{
  std::list<NewNavControllerEventListener*>::iterator li;
  li = find(m_EventListeners.begin(), m_EventListeners.end(), l);
  if (li != m_EventListeners.end()) {
    m_EventListeners.erase(li);
    return;
  }

  m_sc->error("%s not registered\n",l->getName().c_str());
}

void
NewNavController::setProgressTimeout(double t)
{
  m_ProgressTimeout = t; 
  m_LastProgressTime = -1;
}

int 
NewNavController::prepForTask(const std::string &task)
{
  if (m_TaskType == TASKTYPE_OFF) {
    m_sc->error("Must turn ON NewNavController before \"%s\"\n",task.c_str());
    return RETVAL_ERROR;
  } else if (m_TaskType > TASKTYPE_STOP && m_TaskID >= 0) {
    m_sc->error("Aborting ongoing task\n");
    reportAbort(m_TaskID);
  }

  m_Path.clear();
  return RETVAL_OK;
}

void
NewNavController::startProgressMonitoring()
{
  m_sc->debug("startProgressMonitoring\n");
  if (m_Path.empty()) return;

  if (m_ProgressTimeout <= 0) return;

  if (m_ProgressTimeout > 0) {
    Cure::Pose3D cp = getPose();
    m_LastProgressTime = Cure::HelpFunctions::getCurrentTime();
    m_MaxProgressDist = hypot(m_Path.front().getY() - cp.getY(),
                              m_Path.front().getX() - cp.getX());
  }
}

void
NewNavController::stopProgressMonitoring()
{
  m_LastProgressTime = -1;
}

int
NewNavController::stop()
{
  if (m_TaskType == TASKTYPE_OFF) return RETVAL_OK;

  if (m_TaskType != TASKTYPE_STOP) {
    // Tell that whatever was going on is now aborted
    reportAbort(m_TaskID);
  }

  m_TaskType = TASKTYPE_STOP;
  m_Path.clear();
  return RETVAL_OK;
}

int
NewNavController::setDirectSpeed(int taskID, double v, double w, double timeout)
{
  if (m_TaskType == TASKTYPE_OFF) return RETVAL_OK;

  m_TaskType = TASKTYPE_DIRECT;
  m_TaskID = taskID;
  m_DirectV = v;
  m_DirectW = w;
  m_DirectTimeout = HelpFunctions::getCurrentTime() + timeout;

  m_Path.clear();

  return RETVAL_OK;
}

int 
NewNavController::gotoNode(int taskID, int id)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("gotoNode")) return RETVAL_ERROR;
 
  NavGraphNode *to = m_Graph.getNode(id);
  if (to == 0) {
    m_sc->error("Failed to find node %d, cannot go there\n",id);
    return RETVAL_GOALNOTFOUND;
  }

  Cure::Pose3D cp = m_PP->getPose();
  //if (m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(), to,
  //                   m_Path, 0)) {
  
  if (findPath(to, m_Path)) {
    m_sc->error("Failed to plan path to node %d\n",id);
    return RETVAL_NOPATH;
  }

  m_PrevWasGateway = false;

  startProgressMonitoring();

  // Trim away parts from the path that we do not need to pass
  initTrimPath(cp, m_Path);

  m_TaskType = TASKTYPE_GOTOXY;
  m_TaskID = taskID;

  return RETVAL_OK;
}


int 
NewNavController::approachXY(int taskID, double x, double y)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("approachXY")) return RETVAL_ERROR;
 
  Cure::Pose3D cp = m_PP->getPose();


  double dx = x - cp.getX();
  double dy = y - cp.getY();

  double alpha = cp.getTheta() - atan2(dy,dx);

  Cure::NavGraphNode goal;
  goal.setX(x);
  goal.setY(y);
  goal.setTheta(cp.getTheta() + 2.0*alpha);

  stopProgressMonitoring();

  m_Path.push_back(goal);
  m_TaskType = TASKTYPE_APPROACH_POINT;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::approachXYA(int taskID, double x, double y, double a)
{
  if (prepForTask("approachXYA")) return RETVAL_ERROR;
 
  Cure::NavGraphNode goal;
  goal.setX(x);
  goal.setY(y);
  goal.setTheta(a);

  stopProgressMonitoring();

  m_Path.push_back(goal);
  m_TaskType = TASKTYPE_APPROACH_XYA;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::gotoXY(int taskID, double x, double y)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("gotoXY")) return RETVAL_ERROR;
 
  Cure::Pose3D cp = m_PP->getPose();
  //  if (m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(), 
  //                   x, y, 0,
  //                   m_Path, 0)) {
  if (findPath(x, y, 0, m_Path)) {
    m_sc->error("Failed to plan path to x=%f y=%f\n",x,y);
    return RETVAL_NOPATH;
  }

  m_PrevWasGateway = false;

  startProgressMonitoring();

  // Trim away parts from the path that we do not need to pass
  initTrimPath(cp, m_Path);

  m_TaskType = TASKTYPE_GOTOXY;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::gotoXYA(int taskID, double x, double y, double a)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("gotoXYA")) return RETVAL_ERROR;

  Cure::Pose3D cp = m_PP->getPose();
  //if (m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(), 
  //                   x, y, a,
  //                   m_Path, 0)) {
  if (findPath(x, y, a, m_Path)) {
    m_sc->error("Failed to plan path to x=%f y=%f a=%a\n",x,y,a);
    return RETVAL_NOPATH;
  }

  m_PrevWasGateway = false;

  startProgressMonitoring();

  // Trim away parts from the path that we do not need to pass
  initTrimPath(cp, m_Path);

  m_TaskType = TASKTYPE_GOTOXYA;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::gotoPolar(int taskID, double a, double r)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("gotoPolar")) return RETVAL_ERROR;

  Cure::Pose3D cp = m_PP->getPose();
  
  Cure::Pose3D goal = cp;
  goal.setX(cp.getX() + r * cos(cp.getTheta() + a));
  goal.setY(cp.getY() + r * sin(cp.getTheta() + a));
  
  //if (m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(), 
  //                   goal.getX(), goal.getY(), 0,
  //                   m_Path, 0)) {
  if (findPath(goal, m_Path)) {
    m_sc->error("Failed to plan path to r=%f a=%f\n",r,a);
    return RETVAL_NOPATH;
  }

  m_PrevWasGateway = false;

  startProgressMonitoring();

  // Trim away parts from the path that we do not need to pass
  initTrimPath(cp, m_Path);

  m_TaskType = TASKTYPE_GOTOXY;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::rotateRel(int taskID, double da)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("rotateRel")) return RETVAL_ERROR;

  Cure::Pose3D cp = m_PP->getPose();
  Cure::NavGraphNode goal;
  goal.setX(cp.getX());
  goal.setY(cp.getY());
  goal.setTheta(cp.getTheta() + da);

  startProgressMonitoring();

  m_Path.push_back(goal);
  m_TaskType = TASKTYPE_ROTATE;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int
NewNavController::rotateAbs(int taskID, double a)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("rotateAbs")) return RETVAL_ERROR;

  Cure::Pose3D cp = m_PP->getPose();
  Cure::NavGraphNode goal;
  goal.setX(cp.getX());
  goal.setY(cp.getY());
  goal.setTheta(a);

  startProgressMonitoring();

  m_Path.push_back(goal);
  m_TaskType = TASKTYPE_ROTATE;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int
NewNavController::backOff(int taskID, double dist)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (prepForTask("backOff")) return RETVAL_ERROR;

  Cure::Pose3D cp = m_PP->getPose();
  Cure::NavGraphNode start;
  start.setX(cp.getX());
  start.setY(cp.getY());

  startProgressMonitoring();

  m_BackOffDist = dist;
  m_Path.push_back(start);
  m_TaskType = TASKTYPE_BACKOFF;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int
NewNavController::followPerson(int taskID, double x, double y,
                            double dir, double speed, double visibility)
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (m_TaskType != TASKTYPE_FOLLOWPERSON) {
    if (prepForTask("followPerson")) return RETVAL_ERROR;
  }

  m_FollowX = x;
  m_FollowY = y;
  m_FollowDir = dir;
  m_FollowV = speed;

  Cure::Pose3D cp = m_PP->getPose();
  Cure::NavGraphNode target;
  double dir2Target = atan2(y - cp.getY(), x - cp.getX());
  double dist2Target = (hypot(y - cp.getY(), x - cp.getX()) - 
                        m_FollowBeforeTargetDist);

  // we always put the point infront of the robot even if we are
  // close. We let the obstacle avoidance handle not running into
  // things. we do not want the robot to turn around
  const double minDist = 0.05;
  if (dist2Target < minDist) dist2Target = minDist;

  target.setX(cp.getX() + dist2Target * cos(dir2Target));
  target.setY(cp.getY() + dist2Target * sin(dir2Target));
  
  stopProgressMonitoring();

  m_Path.clear(); // Make sure only one goal
  m_Path.push_back(target);
  m_TaskType = TASKTYPE_FOLLOWPERSON;
  m_TaskID = taskID;

  return RETVAL_OK;
}

int 
NewNavController::gotoArea(int taskID, int areaId) 
{
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  // Find the current path
  Cure::Pose3D cp = m_PP->getPose();

  // We begin by checking if we are already in the target area
  NavGraphNode *closest = m_Graph.getClosestNode(cp.getX(), cp.getY());
  if (closest && closest->getAreaId() == areaId) {
    // We are aloready in the right area and do not have to do
    // anything. To make sure that we get a DONE we go to the current
    // pose
    //CureCERR(30) << "We are already in the target area\n";
    return gotoXY(taskID, cp.getX(), cp.getY());
  }

  std::list<NavGraphNode> minPath;
  double minCost = -1;

  // We make a brute force search through all nodes and pick the node
  // that is in the target room but is not a door or adjecent to a
  // door and then we pick the shortest one of the ones we find
  for (int i = 0; i < 2; i++) {

    double cost;
    std::list<NavGraphNode> path;

    minCost = -1;

    for (std::list<NavGraphNode*>::iterator ni = m_Graph.m_Nodes.begin();
         ni != m_Graph.m_Nodes.end(); ni++) {

      if ((*ni)->getAreaId() == areaId) {

        // Make sure that this node is not a door 
        if ((*ni)->getType() == NavGraphNode::NODETYPE_GATEWAY) {
          continue;
        }

        // If this is the first time around the loop we demand that
        // the node is not adjecent to a door as well. If we do not
        // find a solution we relax this the second time
        if (i == 0) {
          // and not is not adjecent to a door
          if ((*ni)->isConnectedToType(NavGraphNode::NODETYPE_GATEWAY)) {
            continue;
          }
        }
        
        if (m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(), 
                             *ni, path, &cost) == 0) {

          if (minCost < 0 || cost < minCost) {

            minCost = cost;
            minPath = path;
          }
        }
      }
    }

    if (minCost >= 0) {
      break;
    }
  }

  if (minCost < 0) {
    m_sc->error("Failed to find a path to area %d\n",areaId);
    return RETVAL_ERROR;
  }

  // Now we have the path and it is just a matter of executing it
  return execPath(taskID, minPath, false);
}

int
NewNavController::execPath(int taskID,
                        const std::list<NavGraphNode> &path, bool finalTurn)
{
  if (finalTurn) {
    if (prepForTask("gotoXYA")) return RETVAL_ERROR;
    m_TaskType = TASKTYPE_GOTOXYA;
  } else {
    if (prepForTask("gotoXY")) return RETVAL_ERROR;
    m_TaskType = TASKTYPE_GOTOXY;
  }

  m_PrevWasGateway = false;
  m_TaskID = taskID;
  m_Path = path;

  startProgressMonitoring();

  return RETVAL_OK;
}

void
NewNavController::initTrimPath(const Cure::Pose3D &cp,
                            std::list<NavGraphNode> &path)
{
  if (!m_UsePathTrimming) return;

  m_sc->log("Trying an initial trim of the path\n");

  if (path.size() <= 1) return;

  // We check how far along the path we can move and still have
  // unobstructed free space to it
  std::list<NavGraphNode>::iterator ni = path.begin();

  // We start checking the second node along the path and then move
  // along as far as we can
  int numToSkip = 0;
  for (ni++; ni != path.end(); ni++) {    
    
    // We do not trim away nodes before a gateway
    if (!m_CanTrimGateways &&
        ni->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      m_sc->log("Reached gateway in trim search\n");
      break;
    }

    // Cannot skip nodes further than some distance away from us
    if (hypot(ni->getY() - cp.getY(), ni->getX() - cp.getX()) > m_MaxTrimDist){
      m_sc->log("Reached to far away in trim search\n");
      break;
    }

    if (m_LMap.obstacleFreeRegion(ni->getX(), ni->getY(), 1.0)) {
      std::list<NavGraphNode>::iterator prev = ni;
      prev--;
      m_sc->log("Can skip node %d\n",prev->getId());
      numToSkip++;
    } else {
      m_sc->log("Path not free to goal in trim search\n");
      break;
    }
  }
  
  if (numToSkip > 0) {
    // skip as many nodes as we are allowed to skip
    for (int i = 0; i < numToSkip; i++) {
      path.pop_front();
      if (!m_CanTrimGateways) {
        // We are not removing gateways so we can safely say that the
        // last was not a gateway
        m_PrevWasGateway = false;
      }
    }
    startProgressMonitoring();
    m_sc->error("Trimmed away %d nodes from init path, left with %d nodes\n",numToSkip,path.size());
  } else {
    m_sc->error("Could not trim anything from the initial path\n");
  }
}

void
NewNavController::trimPath(const Cure::Pose3D &cp, 
                        std::list<NavGraphNode> &path)
{
  if (!m_UsePathTrimming) return;

  if (path.size() <= m_MinPathLengthForTrim) return;

  if (!m_CanTrimGateways && 
      path.front().getType() == NavGraphNode::NODETYPE_GATEWAY) {
    m_sc->log("Not trimming gateways\n");
    return;
  }

  if (!m_CanTrimGateways && m_PrevWasGateway) {
    m_sc->error("Not trimming points after gateway\n");
    return;
  }


  // We check how far along the path we can look and still have the
  // path from teh current position pass within the desired
  // intermediate reach radius of all skipped points

  std::list<NavGraphNode>::iterator ni = path.begin();

  // We start checking the second node along the path and then move
  // along as far as we can
  int numToSkip = 0;
  for (ni++; ni != path.end(); ni++) {    
    
    // We do not trim away nodes before a gateway
    if (!m_CanTrimGateways &&
        ni->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      m_sc->error("Reached gateway in trim search\n");
      break;
    }

    // Cannot skip nodes further than some distance away from us
    if (hypot(ni->getY() - cp.getY(), ni->getX() - cp.getX()) > m_MaxTrimDist){
      m_sc->error("Reached too far in trim search\n");
      break;
    }

    // we do not skip a point unless the point to keep is reachable
    // from the current position
    if (!m_LMap.goalReachable(ni->getX(), ni->getY(), 
                              0.5 * getRobotWidth(), m_MaxTrimDist + 1.0)) {
      m_sc->error("Cannot trim path so that next goal is unreachble\n");
      break;
    }

    // Check if the line going from the current position to the
    // current start of the trimmed path is closer than the reach
    // radius to each of the points that will be skipped. This is
    // testing that we do not deviate too much from the desired path
    for (std::list<NavGraphNode>::iterator ni2 = path.begin(); 
         ni2 != ni; ni2++) {
      double d = HelpFunctions::distPt2LinePts(cp.getX(), cp.getY(),
                                               ni->getX(), ni->getY(),
                                               ni2->getX(), ni2->getY());
      if (d > m_InterGoalTol) {
        m_sc->error("Distance d=%f too large to node %d in trim search\n",d,ni2->getId());
        break;
      }
    }

    // If we got here it means that we can skip this node
    std::list<NavGraphNode>::iterator prev = ni;
    prev--;
    m_sc->error("Can skip node %d\n", prev->getId());
    numToSkip++;
  }
  
  if (numToSkip > 0) {
    // skip as many nodes as we are allowed to skip
    for (int i = 0; i < numToSkip; i++) {
      path.pop_front();
      if (!m_CanTrimGateways) {
        // We are not removing gateways so we can safely say that the
        // last was not a gateway
        m_PrevWasGateway = false;
      }
    }
    startProgressMonitoring();
    m_sc->error("Trimmed away %d nodes, left with %d nodes\n",numToSkip,path.size());
  } else {
     m_sc->error("Could not trim anything from the path\n");
  }
}

int
NewNavController::updateCtrl()
{
  if (m_PP == 0) {
     m_sc->error("Must call setPoseProvider first\n");
    return RETVAL_ERROR;
  }

  if (m_TaskType == TASKTYPE_OFF) return RETVAL_OK;

  Cure::MotionAlgorithm::MotionCmd cmd;
  cmd.type = Cure::MotionAlgorithm::CMD_TYPE_STOP;

  m_sc->log("m_TaskType=%d\n",m_TaskType);

  bool turnMode = (m_TaskType == TASKTYPE_ROTATE);
  bool gatewaySpeed = false;

  if (m_TaskType == TASKTYPE_DIRECT &&
      HelpFunctions::getCurrentTime() > m_DirectTimeout) {
    m_TaskType = TASKTYPE_STOP;    
    reportDone(m_TaskID);
  }

  // Auto navigation mode
  if (m_TaskType == TASKTYPE_STOP) {
    // Nothing to do

  } else if (m_TaskType == TASKTYPE_DIRECT) {
    cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;

    cmd.v = m_DirectV;
    cmd.w = m_DirectW;
    
  } else if (m_Path.empty()) {

    // Stop
    m_TaskType = TASKTYPE_STOP;
    
  } else {

    // Current best estimate of the robot pose
    Cure::Pose3D cp = m_PP->getPosePrediction();
      
    bool largeTurn = false;  // Set to true in goto mode for large turns
    bool followBackup = false;

    if (m_TaskType == TASKTYPE_GOTOXY ||
        m_TaskType == TASKTYPE_GOTOXYA ||
        m_TaskType == TASKTYPE_FOLLOWPERSON) {          
      
      double goalX, goalY, d;
      
      // Find the next goal
      do {

        trimPath(cp, m_Path);

        goalX = m_Path.front().getX();
        goalY = m_Path.front().getY();
        d = hypot(cp.getY() - goalY, cp.getX() - goalX);

        if (m_TaskType == TASKTYPE_GOTOXY ||
            m_TaskType == TASKTYPE_GOTOXYA) {
          // Check if we should look for timeouts        
          if (m_ProgressTimeout > 0 && m_LastProgressTime > 0) {
            
            if (d < m_MaxProgressDist) {
              m_MaxProgressDist = d;
              m_LastProgressTime = Cure::HelpFunctions::getCurrentTime();
            } else if(Cure::HelpFunctions::getCurrentTime() - 
                      m_LastProgressTime > m_ProgressTimeout) {
              m_sc->error("Got timeout trying to reach node %d\n",m_Path.front().getId());
              m_TaskType = TASKTYPE_STOP;
              reportFail(m_TaskID, Cure::MotionAlgorithm::MCTRL_ERROR_TIMEOUT);
              return RETVAL_OK;
            }
          }
        }

        m_sc->log("Heading for node %d of type %d m_PrevWasGateway=%d d=%f\n",m_Path.front().getId(),m_Path.front().getType(),m_PrevWasGateway,d);
        
        bool goalReached = false;
        if ( m_Path.size() > 1 ) {
          
          // Check if we are heading for a node before a gateway, in it
          // or just after it in which case we should use gateway
          // tolerance instead of the normal intermediate node
          // tolerance.
          std::list<NavGraphNode>::iterator next;
          next = m_Path.begin(); next++;
          if (next->getType() == NavGraphNode::NODETYPE_GATEWAY ||
              m_Path.front().getType() == NavGraphNode::NODETYPE_GATEWAY ||
              m_PrevWasGateway) {

            goalReached = (d < m_GatewayGoalTol);

          } else {

            goalReached = (d < m_InterGoalTol);

          }

          // Check if we should drive slower because we are in a gateway
          if (m_Path.front().getType() == NavGraphNode::NODETYPE_GATEWAY ||
              m_PrevWasGateway) {
            gatewaySpeed = true;
          }

        } else {

          if (m_TaskType == TASKTYPE_FOLLOWPERSON) {

            double pd = hypot(cp.getY() - m_FollowY, cp.getX() - m_FollowX);

            const double slowPersonSpeed = 0.2;
            if (pd < m_FollowBackupThreshold ||
                ((m_FollowV < slowPersonSpeed ||
                  fabs(Cure::HelpFunctions::angleDiffRad(cp.getTheta() + M_PI,
                                                         m_FollowDir)) < 
                  M_PI_4) &&
                 (pd < m_FollowDist - m_FollowTolDist) ) ) {
              // If we are too close we should back up
              followBackup = true;
              m_sc->error("FOLLOW: Too close back up, pd=%f\n",pd);
                           
            } else if ((m_FollowV < slowPersonSpeed ||
                        (fabs(Cure::HelpFunctions::angleDiffRad(cp.getTheta() +
                                                                M_PI,
                                                                m_FollowDir)) >
                         M_PI / 3.0 || 
                         fabs(Cure::HelpFunctions::angleDiffRad(cp.getTheta(),
                                                                m_FollowDir)) >
                         M_PI / 3.0)) &&
                       fabs(pd - m_FollowDist) < m_FollowTolDist) {
              
              // Use not moving much or not torwards the robot or away
              // from and we are at the right distance, then we only
              // turn.
              goalReached = true;
              m_sc->error("FOLLOW: Close enough, pd=%f\n",pd);
            } else {
              m_sc->error("FOLLOW: pd=%f\n",pd);
            }

          } else {
            goalReached = (d < m_FinalGoalTol);
          }

        }

        if (goalReached) {

          m_sc->log("Goal %d reached with d=%f m\n",m_Path.front().getId(),d);

          if (m_Path.front().getType() == NavGraphNode::NODETYPE_GATEWAY) {
            m_PrevWasGateway = true;
          } else {
            m_PrevWasGateway = false;
          }
          
          if (m_Path.size() == 1) {
            if (m_TaskType == TASKTYPE_GOTOXYA) {
              m_sc->log("Doing final turning\n");
              m_TaskType = TASKTYPE_ROTATE;
              turnMode = true;
            } else if (m_TaskType == TASKTYPE_FOLLOWPERSON) {
              turnMode = true;

              largeTurn = true;

              // Point to the target
              cmd.dir = atan2(m_FollowY - cp.getY(), 
                              m_FollowX - cp.getX());

            } else {
              m_sc->log("We reached the goal\n");
              
              m_Path.pop_front();
              execCtrl(cmd);
              m_TaskType = TASKTYPE_STOP;
              reportDone(m_TaskID);
              return RETVAL_OK;
            }
          } else {
            m_sc->log("Reached intermediate goal\n");
            m_Path.pop_front();
            startProgressMonitoring();
          }
        } else {
          break;
        }
      } while (m_Path.size() > 1);
      
      if (m_TaskType != TASKTYPE_ROTATE && !turnMode && !followBackup) {
        m_sc->log("Translating\n");
      
        double altGX, altGY;
        int err = m_ND.calcMotionCommand(m_LMap, goalX, goalY, 
                                         m_MaxGotoV, cmd, largeTurn,
                                         true, altGX, altGY);
        if (err == 0) {
          // We limit the speed based on the distance to the goal or
          // so depending on the mode
          if (m_TaskType == TASKTYPE_FOLLOWPERSON) {
            double distErr = (hypot(m_FollowY - cp.getY(),
                                    m_FollowX - cp.getX()) -
                              m_FollowDist);
            
            double relAng = Cure::HelpFunctions::angleDiffRad(cp.getTheta(),
                                                              m_FollowDir);

            double forwVel = m_FollowV * cos(relAng);

            if (forwVel > 0.2) {
              Cure::HelpFunctions::limitAndSetValue(cmd.v, 0, 
                                                    forwVel + 0.5 * distErr);
            } else if (distErr > 0) {
              Cure::HelpFunctions::limitAndSetValue(cmd.v, 0, 0.5 * distErr);
            } else {
              Cure::HelpFunctions::limitAndSetValue(cmd.v, 0, m_MaxTurnV);
            }

            Cure::HelpFunctions::limitAndSetValue(cmd.v, 0, m_MaxFollowV);

          } else {
            Cure::HelpFunctions::limitAndSetValue(cmd.v, 0, 0.5 * d);
          }
        }
      }
    }

    if (m_TaskType == TASKTYPE_ROTATE || largeTurn) {
      
      m_sc->log("Rotating (largeTurn=%d)\n",int(largeTurn));
      cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;
      cmd.v = 0;

      // Check if we should look for timeouts
      if (m_ProgressTimeout > 0 && m_LastProgressTime > 0) {
        // For rotation and backing off we use timeout without the
        // progress
        if(Cure::HelpFunctions::getCurrentTime() - 
           m_LastProgressTime > m_ProgressTimeout) {
          m_sc->error("Got timeout for task %d\n",m_TaskType);

          m_TaskType = TASKTYPE_STOP;
          reportFail(m_TaskID, Cure::MotionAlgorithm::MCTRL_ERROR_TIMEOUT);
          return RETVAL_OK;
        }
      }

      double goalA = m_Path.front().getTheta();
      if (largeTurn) goalA = cmd.dir;
      
      double da = Cure::HelpFunctions::angleDiffRad(goalA, cp.getTheta());

      if (m_TaskType == TASKTYPE_ROTATE && fabs(da) < m_FinalGoalTolRot) {
        m_sc->log("We reached the goal angle\n");
        m_Path.pop_front();
        m_TaskType = TASKTYPE_STOP;
        reportDone(m_TaskID);
      } else if (m_TaskType == TASKTYPE_FOLLOWPERSON && 
                 fabs(da) < m_FollowTolRot) {
        cmd.w = 0;
      } else {
        m_sc->debug("Still have to turn  da=%f deg cp.a=%f goalA=%f\n",Cure::HelpFunctions::rad2deg(da),cp.getTheta(),goalA);
        
        int err = m_Turn.calcMotionCommand(m_LMap, goalA, m_MaxTurnV, cmd);
        if (err) {
          m_sc->error("Failed to find turn motion command, err=%d  state=%d\n",err,m_TaskType);
        }                
      }                
    }

    if (m_TaskType == TASKTYPE_APPROACH_POINT ||
        m_TaskType == TASKTYPE_APPROACH_XYA) {
      
      Cure::NavGraphNode goal = m_Path.front();
      
      double dx = goal.getX() - cp.getX();
      double dy = goal.getY() - cp.getY();

      double distance = hypot(dx,dy);

      double goalA = goal.getTheta();
      double da = Cure::HelpFunctions::angleDiffRad(goalA, cp.getTheta());

      if(distance < m_ApproachTolDist) {
	    m_sc->error("ApproachPoint \n");
        
        if ( (m_TaskType == TASKTYPE_APPROACH_POINT) ||
             (fabs(da) < m_ApproachTolRot) ) {
          m_sc->error("We are already there \n");
          m_Path.pop_front();
          execCtrl(cmd);
          m_TaskType = TASKTYPE_STOP;
          reportDone(m_TaskID);
          return RETVAL_OK;
        } else {
          m_sc->error("Still turning %f degs\n",Cure::HelpFunctions::rad2deg(da));
          cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VA;
          cmd.v = 0;
          cmd.w = goalA;
        }
      } else {
	double alpha = atan2(dy,dx) - cp.getTheta();
	double radius = 0.5*hypot(dx,dy) / sin(alpha);
        
	cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;
	cmd.v = 0.4;
	cmd.w = cmd.v / radius;	
        
	m_sc->log("ApproachPoint \n");
	m_sc->log("   robot pos    %f %f %f \n",cp.getX(),cp.getY(),cp.getTheta());
	m_sc->log("   goal pos     %f %f %f\n",goal.getX(),goal.getY(),goal.getTheta());
	m_sc->log("   radius alpha %f %f\n",radius,alpha);
	m_sc->log("   v omega      ,%f %f",cmd.v,cmd.w);
	m_sc->log("----------------\n");
      }
    }

    // Adaptations for follow mode
    if (m_TaskType == TASKTYPE_FOLLOWPERSON) {
      if (followBackup) {

        cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;

        // Check if it is safe to backup

        // The robot width
        double w = m_ND.getRobotWidth();
        // The distance from the center of the robot to the back of it
        // and some margin
        double d = m_Turn.getBackDist() + 0.2;
        if (m_LMap.obstacleFreeRegion(m_LMap.getX(), m_LMap.getY(),
                                      m_LMap.getX() - d * cos(m_LMap.getA()),
                                      m_LMap.getY() - d * sin(m_LMap.getA()),
                                      w)) {
          
          cmd.v = -m_FollowBackOffV;
        } else {
          m_sc->error("Not safe to backup!!!\n");
          cmd.v = 0;
        }

        // Correct the angle
        Cure::NavGraphNode goal = m_Path.front();
        double dx = m_FollowX - cp.getX();
        double dy = m_FollowY - cp.getY();
        
        double dir = atan2(dy,dx);
        double da = Cure::HelpFunctions::angleDiffRad(dir, cp.getTheta());
        cmd.w = da * m_AngleGain;
      }
    }

    if (m_TaskType == TASKTYPE_BACKOFF) {
      Cure::NavGraphNode start = m_Path.front();
      
      double dx = start.getX() - cp.getX();
      double dy = start.getY() - cp.getY();

      double distance = hypot(dx,dy);


      // Check if we should look for timeouts
      if (m_ProgressTimeout > 0 && m_LastProgressTime > 0) {
        
        // For rotation and backing off we use timeout without the
        // progress
        if(Cure::HelpFunctions::getCurrentTime() - 
           m_LastProgressTime > m_ProgressTimeout) {
          m_sc->error("Got timeout for task %d\n", m_TaskType);
          m_TaskType = TASKTYPE_STOP;
          reportFail(m_TaskID, Cure::MotionAlgorithm::MCTRL_ERROR_TIMEOUT);
          return RETVAL_OK;
        }
      }

      if (distance >= m_BackOffDist) {          
        m_Path.pop_front();
        m_TaskType = TASKTYPE_STOP;
        reportDone(m_TaskID);
      } else {
        cmd.v = -m_BackOffV;
        cmd.w = 0;
        cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;
      }
    }
  
    if (cmd.type == Cure::MotionAlgorithm::CMD_TYPE_VA) {
      if (m_TurnAngleIntoSpeed) {        
        cmd.type = Cure::MotionAlgorithm::CMD_TYPE_VW;
        cmd.w = (m_AngleGain * 
                 Cure::HelpFunctions::angleDiffRad(cmd.dir, cp.getTheta()));
        m_sc->log("Turned angle into speed, w=%f\n",cmd.w);
      } else {
        m_sc->log("NOT turning nagle into speed\n");
      }
    }

    if (turnMode) {
      m_sc->log("Limiting speeds in turn mode\n");
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.v, m_MaxTurnV);
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.w, m_MaxTurnW);
    } else if (gatewaySpeed) {
      m_sc->log("Limiting speeds in gateway mode\n");
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.v, m_MaxGatewayV);
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.w, m_MaxGatewayW);
    } else {
      m_sc->log("Limiting speeds in goto mode\n");
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.v, m_MaxGotoV);
      Cure::HelpFunctions::limitAndSetValueSymm(cmd.w, m_MaxGotoW);
    }

    m_sc->log("Speed command now type=%d v=%f w=%f dir=%f", cmd.type,cmd.v, cmd.w,Cure::HelpFunctions::rad2deg(cmd.dir));

    if (cmd.v > 0 && cmd.v < m_MinNonZeroV) cmd.v = m_MinNonZeroV;
    if (cmd.v < 0 && cmd.v > -m_MinNonZeroV) cmd.v = -m_MinNonZeroV;
    if (cmd.w > 0 && cmd.w < m_MinNonZeroW) cmd.w = m_MinNonZeroW;
    if (cmd.w < 0 && cmd.w > -m_MinNonZeroW) cmd.w = -m_MinNonZeroW;

    m_sc->log("Speed command now type=%d v=%f w=%f dir=%f\n",cmd.type,cmd.v,cmd.w,Cure::HelpFunctions::rad2deg(cmd.dir));
  }

  execCtrl(cmd);
  return RETVAL_OK;
}

void
NewNavController::reportAbort(int taskID)
{
  std::list<NewNavControllerEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->abortTask(taskID);
  }
}

void
NewNavController::reportDone(int taskID)
{
  std::list<NewNavControllerEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->doneTask(taskID);
  }
}

void
NewNavController::reportFail(int taskID, int error)
{
  std::list<NewNavControllerEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->failTask(taskID, error);
  }

}

bool
NewNavController::getCurrentNode(Cure::NavGraphNode &n) const
{
  if (m_Path.empty()) {
    m_sc->log("Path is empty, no node to return\n");
    return false;
  }

  n = m_Path.front();
  return true;
}

bool
NewNavController::getTargetNode(Cure::NavGraphNode &n) const
{
  if (m_Path.empty()) {
    m_sc->log("Path is empty, no node to return\n");
    return false;
  }

  n = m_Path.back();
  return true;
}

Cure::Pose3D
NewNavController::getPose(bool usePrediction) const {
  if (m_PP == 0) {
    m_sc->error("Must call setPoseProvider first\n");
    Cure::Pose3D ret;
    ret.setTime(0);
    return ret;
  }

  if (usePrediction) {
    return m_PP->getPosePrediction();
  } else {
    return m_PP->getPose();
  }
}

int
NewNavController::findPath(const Cure::Pose3D &goal,std::list<NavGraphNode> &path)
{
  if (m_LMap.goalReachable(goal.getX(), goal.getY(), 0.8, 8)) {
    NavGraphNode n;
    n.setX(goal.getX());
    n.setY(goal.getY());
    n.setTheta(goal.getTheta());
    n.setId(-42);
    path.clear();
    path.push_back(n);
    m_sc->error("Heading directly for the goal as it is reachable\n");
    return RETVAL_OK;
  }

  Cure::Pose3D cp;
  cp = getPose();
  return m_Graph.findPath(cp.getX(), cp.getY(), cp.getTheta(),
                          goal.getX(), goal.getY(), goal.getTheta(),
                          path, 0);
}

int
NewNavController::findPath(NavGraphNode *goal, std::list<NavGraphNode> &path)
{
  if (goal == 0) {
    m_sc->error("Null-pointer goal\n");
    return RETVAL_ERROR;
  }

  Cure::Pose3D g;
  g.setX(goal->getX());
  g.setY(goal->getY());
  g.setTheta(goal->getTheta());
  return findPath(g, path);
}

int
NewNavController::findPath(double x, double y, double a,
                        std::list<NavGraphNode> &path)
{
  Cure::Pose3D g;
  g.setX(x);
  g.setY(y);
  g.setTheta(a);
  return findPath(g, path);
}

//void
//NewNavController::displayRL(RoboLookProxy *rlp, bool clearEllipseTarget,
//                         bool displayLocalMap, bool displayGraph)
//{
//  if (rlp == 0) return;
//
//
//  if (displayGraph) m_Graph.displayRL(rlp, true, true);
//  if (displayLocalMap) m_LMap.displayRL(*rlp, true);
//      
//  if (!m_Path.empty()) {
//
//    RL_LineItem edges[m_Path.size()+1];
//    int ei = 0;
//    edges[ei].xS = getPose().getX();
//    edges[ei].yS = getPose().getY();
//    edges[ei].zS = 0.01;
//    for (std::list<NavGraphNode>::iterator ni = m_Path.begin();
//         ni != m_Path.end(); ni++) {
//      if (ei > 0) {
//        edges[ei].xS = edges[ei-1].xE;
//        edges[ei].yS = edges[ei-1].yE;
//        edges[ei].zS = edges[ei-1].zE;
//      }
//      edges[ei].xE = ni->getX();
//      edges[ei].yE = ni->getY();
//      edges[ei].zE = 0.01;
//      edges[ei].color = 12;
//      edges[ei].width = 1;
//      m_sc->log("edge[%d]: xS=%f yS=%f xE=%f yE=%f\n",ei,edges[ei].xS,edges[ei].yS,edges[ei].xE,edges[ei].yE);
//
//      ei++;
//    }
//    rlp->addLines(RL_ENV_EST, edges, ei);
//
//    RL_EllipseItem ell[2];
//    ell[0].xC = m_Path.front().getX();
//    ell[0].yC = m_Path.front().getY();
//    ell[0].z = 0;
//    ell[0].majorAngle = 0;
//    ell[0].style = 0;
//    if (m_Path.size() == 1) {
//      ell[0].color = 2;
//      ell[0].major = m_FinalGoalTol;
//      ell[0].minor = m_FinalGoalTol;
//    } else {
//      // Check we are heading for a node before a gateway, in it
//      // or just after it in which case we should use gateway
//      // tolerance instead of the normal intermediate node
//      // tolerance.
//      std::list<NavGraphNode>::iterator next;
//      next = m_Path.begin(); next++;
//      if (next->getType() == NavGraphNode::NODETYPE_GATEWAY ||
//          m_Path.front().getType() == NavGraphNode::NODETYPE_GATEWAY ||
//          m_PrevWasGateway) {               
//        ell[0].color = 6;      
//        ell[0].major = m_GatewayGoalTol;
//        ell[0].minor = m_GatewayGoalTol;
//      } else {
//        ell[0].color = 7;      
//        ell[0].major = m_InterGoalTol;
//        ell[0].minor = m_InterGoalTol;
//      }
//    }
//    
//    int ne = 1;
//    if (m_TaskType == TASKTYPE_FOLLOWPERSON) {
//      memcpy(ell + 1, ell + 0, sizeof(RL_EllipseItem));
//      
//      // The position we aim to move to which is not the thing we are
//      // following but something slightly ahead of it
//      ell[0].xC = m_FollowX;
//      ell[0].yC = m_FollowY;
//      ell[0].color = 1;
//      ne++;
//    }
//
//    rlp->addEllipses(RL_ENV_EST, ell, ne, clearEllipseTarget);
//  } else {
//    if (clearEllipseTarget) {
//      rlp->clearEllipses(RL_ENV_EST);
//    }
//  }
//}
