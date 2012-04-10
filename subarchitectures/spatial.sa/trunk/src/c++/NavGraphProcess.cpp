//
// = FILENAME
//    NavGraphProcess.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "NavGraphProcess.hpp"

#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <Utils/CureDebug.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/CureDebug.hh>
#include <SensorData/LaserScan2d.hh>
#include <AddressBank/RLDisplayHelper.hh>
#include <Navigation/NavGraphNode.hh>
#include <Navigation/NavGraphEdge.hh>
#include <Navigation/NavGraphGateway.hh>
#include <VisionData.hpp>
#include <FrontierInterface.hpp>
#include <cfloat>

#include <sstream>

using namespace cast;
using namespace std;
using namespace boost;
using namespace navsa;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new NavGraphProcess();
  }
}

NavGraphProcess::NavGraphProcess()
  : NavGraphEventListener("NavGraphProcess")
{
  m_TopRobPos = 0;
  m_TopRobPosWMid = "";
  m_InDoor = false;

  m_MaxNumFNodes = 10000;

  m_DontWriteFiles = false;

  m_MotionDetector = 0;
  m_RemoveMotionBeams = false;
  m_LaserFreqEstNScans = 0;

  m_MinDoorWidth = 0.6;
  m_MaxDoorWidth = 1.05;

  m_DoorDetector.setUsedAlgorithm(2);
  m_DoorDetector.setMaxGap(0.02);
  m_DoorDetector.setMinDoorPostSize(0.05);
  m_DoorDetector.setClearanceThreshold(1.5);
  m_DoorDetector.setMinNumLongForClearance(50);

  // Only mode supported now
  m_UniqueObjects = true;
}

NavGraphProcess::~NavGraphProcess() 
{
  if (!m_DontWriteFiles) saveGraphToFile(m_cureNavGraphFile);
}

long NavGraphProcess::makeEdgeId(int nodeId1, int nodeId2)
{
  if (nodeId1 > nodeId2) {
    return (m_MaxNumFNodes * nodeId1 + nodeId2);
  } else {
    return (m_MaxNumFNodes * nodeId2 + nodeId1);
  }
}

void NavGraphProcess::saveGraphToFile(const std::string &filename) 
{
  if(filename != ""){
    std::fstream fs;
    fs.open(filename.c_str(), std::ios::out);
    if (fs.is_open()) {

      // Write NODES
      fs << "NODES " << m_FreeNodes.size() << std::endl;
      for (std::map<long, FNodeHolder>::iterator iter = m_FreeNodes.begin();
           iter != m_FreeNodes.end(); iter++) {

        char buf[128];
        if (iter->second.m_data->type.empty()) {
          sprintf(buf, "Area00");
        } else {
          sprintf(buf, "%s%02ld", 
                  iter->second.m_data->type[0].name.c_str(),
                  (long)iter->second.m_data->type[0].id);
        }

        fs << iter->second.m_data->gateway << " "
           << iter->second.m_data->nodeId << " "
           << iter->second.m_data->x << " "
           << iter->second.m_data->y << " "
           << iter->second.m_data->theta << " "
           << buf << " "
           << iter->second.m_data->areaId << " ";

        if (iter->second.m_data->gateway) {
          if (iter->second.m_data->width.empty()) {
            fs << "1";
          } else {
            fs << iter->second.m_data->width[0];
          }
        }
        fs << std::endl;
      }
      
      // Write EDGES
      fs << "EDGES " << m_AccessEdges.size() << std::endl;
      for (std::map<long, AEdgeHolder>::iterator iter = m_AccessEdges.begin();
           iter != m_AccessEdges.end(); iter++) {
        fs << iter->second.m_data->startNodeId << " "
           << iter->second.m_data->endNodeId << " " 
           << iter->second.m_data->cost << std::endl;
      }

      // Write OBJECTS
      fs << "OBJECTS " << m_Objects.size() << std::endl;
      for (std::map<long,ObjDataHolder>::iterator iter = m_Objects.begin();
           iter != m_Objects.end(); iter++) {
        fs << iter->second.m_data->x << " "
           << iter->second.m_data->y << " "
           << iter->second.m_data->z << " "
           << iter->second.m_data->areaId << " "
           << iter->second.m_data->category << " "
           << iter->second.m_data->probability << " "
           << iter->second.m_data->objectId << " "
           << iter->second.m_data->camX << " "
           << iter->second.m_data->camY << std::endl;
      }

      // Write AREAS
      fs << "AREAS " << m_Areas.size() << std::endl;
      for (std::list<NavGraphProcess::Area>::iterator ai = m_Areas.begin();
           ai != m_Areas.end(); ai++) {

        fs << ai->m_data->id << " ";

        if (ai->m_data->type.empty()) {
          fs << "Area 0" << std::endl;
        } else {
          fs << ai->m_data->type[0].name << " "
             << ai->m_data->type[0].id << std::endl;
        }
      }
    }
  }
  
}

void NavGraphProcess::configure(const map<string,string>& _config) 
{
  NavData::NavGraphInterfacePtr mapservant = new NavGraphServer(this);
  registerIceServer<NavData::NavGraphInterface, NavData::NavGraphInterface>(mapservant);

  m_WriteFirstTopologicalPose = false;
  m_WriteFirstGraph = false;
  m_NavGraphWMid = "";
  m_NextObjectNodeId = m_MaxNumFNodes;

  map<string,string>::const_iterator it;

  it = _config.find("--no-file-output");  
  if(it != _config.end()) {
    m_DontWriteFiles = true;
  }
  if (!m_DontWriteFiles) log("Will output files");

  m_UseDoorHypotheses = false;
  it = _config.find("--filter-doors");  
  if(it != _config.end()) {
    m_UseDoorHypotheses = true;
  }

  // You can instruct the graph to automatically try to merge areas
  bool autofix = (_config.find("--auto-merge-areas") != _config.end());
  
  m_cureNavGraph.setAutoFix(autofix);
  if (autofix) {
    log("Will autofix area merging");
  } else {
    log("Will NOT autofix area merging");
  }

  // Check if we have specified a map in which case we will load the
  // corresponding graph from file
  it = _config.find("-m"); 
  if (it != _config.end()) {
    m_cureNavGraphFile = it->second;
    fstream f(m_cureNavGraphFile.c_str(), ios::in);
    if(f.is_open()){
      f.close();
      // Make backup
      if (!m_DontWriteFiles) {
        string backup = m_cureNavGraphFile + ".bak";
        std::ofstream(backup.c_str()) << 
          std::ifstream(m_cureNavGraphFile.c_str()).rdbuf(); 
      }
      loadGraphFromFile(m_cureNavGraphFile);
    }
  } else {
    m_cureNavGraphFile = "tmpmap.graph";
  }
  
  it = _config.find("-c");
  if (it == _config.end()) {
    printf("NavGraphProcess::configure need config file (use -c option)\n");
    //nah: made this stupid but strict
    std::abort();
  }
  string configfile = (it->second);  

  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) {
    printf("configure failed to config with \"%s\"\n",
           configfile.c_str());
  }  

  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    println("configure Failed to get laser pose on robot from \"%s\"\n", 
            configfile.c_str());
    std::abort();
  }

//   if (cfg.getSensorPose(2, 0, m_CamPoseR)) {
//     println("configure Failed to get cam pose on robot from \"%s\"\n", 
//             configfile.c_str());
//     std::abort();
//   }

  it = _config.find("--min-door-width");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_MinDoorWidth;
  }

  it = _config.find("--max-door-width");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxDoorWidth;
  }
  log("minDoorWidth=%fm maxDoorWidth=%fm", m_MinDoorWidth, m_MaxDoorWidth);

  m_RemoveMotionBeams = (_config.find("--remove-motion") != _config.end());

  // configure the cure nav graph
  m_cureNavGraph.addEventListener(this);
}

double  NavGraphProcess::NavGraphServer::getPathLength(double xS, double yS, double aS, double xG, double yG, double aG, const Ice::Current &_context){
  std::list<Cure::NavGraphNode> path;
  double d;

  IceUtil::Mutex::Lock lock(m_pOwner->m_GraphMutex);

  m_pOwner->m_cureNavGraph.findPath(xS,yS,aS,xG,yG,aG, path, &d);
  return d;
}

int NavGraphProcess::NavGraphServer::getAreaId(double x, double y, double a, double maxDist, const Ice::Current &_context){
  int id;

  IceUtil::Mutex::Lock lock(m_pOwner->m_GraphMutex);

  Cure::NavGraphNode* node;
  node = m_pOwner->m_cureNavGraph.getClosestNode(x,y,a,maxDist); 

  if (node != NULL)
    id = node->getAreaId();
  else
    id = -1;

  return id;
}

int NavGraphProcess::NavGraphServer::getClosestNodeId(double x, double y, double a, double maxDist, const Ice::Current &_context){
  int id;

  IceUtil::Mutex::Lock lock(m_pOwner->m_GraphMutex);

  Cure::NavGraphNode* node;
  node = m_pOwner->m_cureNavGraph.getClosestNode(x,y,a,maxDist); 

  if (node != NULL)
    id = node->getId();
  else
    id = -1;

  return id;
}
void NavGraphProcess::loadGraphFromFile(const std::string &filename)
{
  IceUtil::Mutex::Lock lock(m_GraphMutex);

  // Load the cure part of the file
  log("Loading file %s", filename.c_str());
  //m_cureNavGraph.loadGraph(filename);

  m_cureNavGraph.clear();
  
  std::fstream is;
  is.open(filename.c_str(), std::ios::in);
  if (is <= 0) {
    CureCERR(20) << "Failed to open graph file \""
                 << filename << "\"\n";
    return;
  }

  std::string line, key;
  int n;

  // Read NODES
  int lineno = 0;
  if (getline(is, line)) {

    std::istringstream str(line);
    str >> key >> n;

    println("Reading %d nodes from file %s", n, filename.c_str());

    for (int i = 0; i < n; i++) {
      lineno++;
      getline(is, line);
      std::istringstream istr(line);
      int type, id, aid = 0;
      std::string areaclass = "Area00";
      double x,y,a;
      if ( istr >> type >> id >> x >> y >> a >> areaclass >> aid) {

        int areaclassno=0;
        if (areaclass.size() >= 2) {
          sscanf(areaclass.substr(areaclass.size()-2,2).c_str(), 
                 "%d", &areaclassno);
          areaclass = areaclass.substr(0,areaclass.size()-2);
        } else {
          areaclass = "Area";
        }

        if (type == Cure::NavGraphNode::NODETYPE_GATEWAY) {
          // Attempt to read the width of the door
          double width;
          if ( !(istr >> width) ) {
            width = 1.0;
            debug("No width for gateway id=%d assuming %fm", id, width);
          }
          
          m_cureNavGraph.addDoorToNodeList(id, aid, x, y, a, width, "-", 2.0);

          addFreeNode(id, 
                      x, y, 0.0, a,
                      aid, areaclass, areaclassno,
                      1,     // a gateway
                      2.0,   // max speed
                      width, // door width
                      "-");  // name of node


        } else {
          
          m_cureNavGraph.addNodeToNodeList(id, aid, x,y,a, "-", 2.0);
          
          addFreeNode(id, 
                      x, y, 0.0, a,
                      aid, areaclass, areaclassno,
                      0,   // not gateway
                      2.0, // max speed
                      0.0, // door width
                      "-");// name of node

        }
      } else {
        println("Failed to read node data on line %d", lineno);
      }
    }
  }

  // Read EDGES
  lineno++;
  if (getline(is, line))  {
    std::istringstream str(line);
    str >> key >> n;

    println("Reading %d edges from file %s", n, filename.c_str());

    for (int i = 0; i < n; i++) {
      lineno++;
      getline(is, line);
      std::istringstream istr(line);
      long id1, id2;
      if ( istr >> id1 >> id2 ) {

        if (m_cureNavGraph.addEdgeToEdgeList(id1, id2) == 0) {

          // Read cost if it is given in the file
          double cost = 1;
          if (istr >> cost) {
            m_cureNavGraph.m_Edges.back().setCost(cost);
          }

          addAccessEdge(id1, id2, cost);

        } else {
          CureCERR(20) << "WARNING: Problems with edge on line " 
                       << lineno << "\n";
        }
        
      } else {
        CureCERR(20) << "WARNING: Failed to read edge data on line " 
                     << lineno << "\n";
      }
    }
  }

  m_cureNavGraph.connectNodes();

  // Read Areas
  lineno++;
  if (getline(is, line)) {

    std::istringstream str(line);
    str >> key >> n;

    println("Reading %d objects from file %s", n, filename.c_str());

    for (int i = 0; i < n; i++) {

      lineno++;
      getline(is, line);
      std::istringstream istr(line);

      double x,y,z, robX, robY;
      long areaId, objId;
      std::string category;
      double probability;

      if (istr >> x >> y >> z >> areaId >> category >> probability >> objId >> robX >> robY)
      {
        addObject(x, y, z, robX, robY,
                  areaId, category, objId, probability);
        log("object added: %s: at area %i with probability %f", category.c_str(), areaId, probability);
      }
    }
  }
   
  lineno++;
  if (getline(is, line)) {
    std::istringstream str(line);
    str >> key >> n;

    println("Reading %d Areas from file %s", n, filename.c_str());
    
    for (int i = 0; i < n; i++) {
      
      lineno++;
      getline(is, line);
      std::istringstream istr(line);
      
      std::string tmp;
      
      NavGraphProcess::Area newArea;
      newArea.m_data = new NavData::Area;
      newArea.m_data->type.resize(1);
      if (istr >> newArea.m_data->id
          >> newArea.m_data->type[0].name
          >> newArea.m_data->type[0].id) {
        
        m_AreasFromFile.push_back(newArea);
        
      }
    }
  } 

  if (m_NewFNodes.size() > 0 || m_NewObjData.size() > 0) {
    m_WriteFirstTopologicalPose = true;
    m_WriteFirstGraph = true;
  }
}

void NavGraphProcess::start() {

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                  new MemberFunctionChangeReceiver<NavGraphProcess>(this,
                                         &NavGraphProcess::newRobotPose));
  
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<NavGraphProcess>(this,
                                         &NavGraphProcess::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::ObjObs>(cdl::ADD),
                  new MemberFunctionChangeReceiver<NavGraphProcess>(this,
                                         &NavGraphProcess::newObjObs));
  
  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject>(cdl::ADD),
                  new MemberFunctionChangeReceiver<NavGraphProcess>(this,
                                         &NavGraphProcess::newVisualObject));

  addChangeFilter(createLocalTypeFilter<NavData::ObjObs>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<NavGraphProcess>(this,
                                         &NavGraphProcess::newObjObs));

  addChangeFilter(createLocalTypeFilter<FrontierInterface::DoorHypothesis>(cdl::ADD),
      new MemberFunctionChangeReceiver<NavGraphProcess>(this,
	&NavGraphProcess::newDoorHypothesis));

  log("NavGraphProcess started");
}

void NavGraphProcess::checkAndAddArea(int aid, 
                                      const std::string &areaClass,
                                      long areaClassNo)
{
  // First we check if this area already exists
  std::list<NavGraphProcess::Area>::iterator aIter;
  for (aIter = m_Areas.begin(); aIter != m_Areas.end(); aIter++) {
    if (aIter->m_data->id == aid) {
      break;
    }
  }

  if (aIter != m_Areas.end()) {

    if (areaClass != "") {

      // Now we check if the area class is the same as before
      if (!aIter->m_data->type.empty() &&
          (aIter->m_data->type[0].name == areaClass) ) {
        
        // We found an exiting areas so we do not have to add anything
        log("Area %d already exist and class is already %s, no need to update WM", aid, areaClass.c_str());
        
      } else {

        aIter->m_data->type.resize(1);
        aIter->m_data->type[0].name = areaClass;
        aIter->m_data->type[0].id = areaClassNo;
        
        overwriteWorkingMemory<NavData::Area>(aIter->m_WMid,
                                              aIter->m_data);
        
        log("Area %d changed to type %s, WM updated", aid, areaClass.c_str());
        
      }
    } else {
      log("Area %d not updating in WM, since no area class given", aid);
    }

  } else {
    // Create a new area objects
    NavGraphProcess::Area newArea;
    newArea.m_data = new NavData::Area;
    newArea.m_WMid = newDataID();
    newArea.m_data->id = aid;

    if (areaClass != "") {
      newArea.m_data->type.resize(1);
      newArea.m_data->type[0].name = areaClass.c_str();
      newArea.m_data->type[0].id = areaClassNo;
    }

    addToWorkingMemory<NavData::Area>(newArea.m_WMid,
                                      newArea.m_data);

    m_Areas.push_back(newArea);

    log("Created new area %d in WM", aid);
  }
}
  
void NavGraphProcess::changedArea(int aid)
{
  log("Changed to area %d", aid);
  checkAndAddArea(aid);
  writeTopologicalPosToWorkingMemory(aid);
}

void NavGraphProcess::mergedAreaIds(int aid1, int aid2)
{
  log("Merging areas, id=%d will now be id=%d", aid2, aid1);

  // Find the area that will be removed (the second one)
  std::list<NavGraphProcess::Area>::iterator aIter;
  for (aIter = m_Areas.begin(); aIter != m_Areas.end(); aIter++) {
    if (aIter->m_data->id == aid2) {
      break;
    }
  }
  
  if (aIter != m_Areas.end()) {
    log("Deleting area %d", aid2);
    // We found the area to remove and do so
    deleteFromWorkingMemory(aIter->m_WMid);
    m_Areas.erase(aIter);
  } else {
    log("Did not find area %d to delete", aid2);
  }

  // Go through the nodes from area aid2 and change their area id to aid1
  std::map< long, FNodeHolder >::iterator ni;
  for (ni = m_FreeNodes.begin(); ni != m_FreeNodes.end(); ni++) {
    if (ni->second.m_data->areaId == aid2) {
      ni->second.m_data->areaId = aid1;
      log("Changing area id for node %d", (int)ni->second.m_data->nodeId);
      overwriteWorkingMemory<NavData::FNode>(ni->second.m_WMid,
                                             ni->second.m_data);      
    }
  }
  log("Done merging the graph");

  writeGraphToWorkingMemory();
}

void NavGraphProcess::areaIdConflict(Cure::NavGraphNode &currNode, 
                                     Cure::NavGraphGateway &gw)
{
  log("Area conflict detected for node %d in area %d, door with id %d may not be  door", currNode.getId(), currNode.getAreaId(), gw.getId());
}

void NavGraphProcess::changedCurrentNode(int fromId, int toId)
{
  log("Current node changed from node %d to node %d", fromId, toId);
  
  bool publishChangesNow = false;

  int nEdgesAdded = 0;

  // Got through the list of edges and check if a new one has been
  // added, if it has been added it should be the last one in the list
  while (m_AccessEdges.size() + m_NewAEdges.size() < 
         m_cureNavGraph.m_Edges.size()) {


//CHECK FOR DOORWAY NODE HERE
    
    // Check if the last added edge in the cure nav graph is the
    // missing edge
    if ( (makeEdgeId(m_cureNavGraph.m_Edges.back().getNodeId1(),
                     m_cureNavGraph.m_Edges.back().getNodeId2()) ==
          makeEdgeId(fromId, toId)) &&
         (m_AccessEdges.find(makeEdgeId(m_cureNavGraph.m_Edges.back().getNodeId1(), m_cureNavGraph.m_Edges.back().getNodeId2())) == m_AccessEdges.end()) ) {
      
      debug("Adding edge between nodes %d and %d", fromId, toId);
      
      // add the new edge
      addAccessEdge(fromId, toId, m_cureNavGraph.m_Edges.back().getCost());
      
      // check if both ends of the edge is already in working
      // memory so that the guys that want life to be easy can
      // assume that the edge comes after the two nodes it
      // connects;-)
      if (areNodesInWM(fromId, toId)) publishChangesNow = true;

      nEdgesAdded++;
        
    } else {
      
      debug("Have to do a brute force search for the new edge since it was not the last one added");
      
      // We have to do a brute force search for the missing node
      std::list<Cure::NavGraphEdge>::iterator ei;
      for (ei = m_cureNavGraph.m_Edges.begin();
           ei != m_cureNavGraph.m_Edges.end(); ei++) {
        
        bool foundMatch = false;
        
        std::map<long, AEdgeHolder>::iterator aei;
        for (aei = m_AccessEdges.begin(); aei != m_AccessEdges.end(); aei++) {
          
          if (makeEdgeId(ei->getNodeId1(), ei->getNodeId2()) ==
              makeEdgeId(aei->second.m_data->startNodeId, 
                         aei->second.m_data->endNodeId)) {
            foundMatch = true;
            break;
          }
        }
        
        if (!foundMatch) {

          // add the new edge
          addAccessEdge(ei->getNodeId1(), ei->getNodeId2(),ei->getCost());

          // check if both ends of the edge is already in working
          // memory so that the guys that want life to be easy can
          // assume that the edge comes after the two nodes it
          // connects;-)
          if ( areNodesInWM(ei->getNodeId1(), ei->getNodeId2()) ) {
            publishChangesNow = true;
          }
          
          nEdgesAdded++;
        }
      }
    }
  }
  
  if (nEdgesAdded > 1) {
    log("WARNING: More than one edge added at a time, might get edge without both edns in WM");
  }
  
  if (publishChangesNow) writeGraphToWorkingMemory();
}

bool NavGraphProcess::areNodesInWM(long id1, long id2)
{
  
  std::map<long,FNodeHolder>::iterator n1, n2;
  n1 = m_FreeNodes.find(id1);
  n2 = m_FreeNodes.find(id2);
  
  if (n1 != m_FreeNodes.end() && n2 != m_FreeNodes.end()) {
    return true;
  } else {
    if (n1 == m_FreeNodes.end()) {
      debug("Node with id %d not in WM yet, not adding edge", 
          m_cureNavGraph.m_Edges.back().getNodeId1());
    } 
    if (n2 == m_FreeNodes.end()) {
      debug("Node with id %d not in WM yet, not ading edge", 
          m_cureNavGraph.m_Edges.back().getNodeId2());
    } 
  }

  return false;
}

void NavGraphProcess::changedNodeType(int id)
{
  Cure::NavGraphNode *node = m_cureNavGraph.getNode(id);

  log("Node type changed for node %d",id);
  
  if (node == 0) return;

  //log("Changed the type of node %d", id);
  for (std::list<FNodeHolder>::iterator fni = m_NewFNodes.begin();
       fni != m_NewFNodes.end(); fni++) {

    if (fni->m_data->nodeId != id) continue;

    if ( ( (node->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) &&
           (fni->m_data->gateway == 1) ) ||
         ( (node->getType() != Cure::NavGraphNode::NODETYPE_GATEWAY) &&
           (fni->m_data->gateway == 0) ) ) break;
    
    fni->m_data->gateway = node->getType();
    fni->m_data->x = node->getX();
    fni->m_data->y = node->getY();
    fni->m_data->theta = node->getTheta();
    fni->m_data->areaId = node->getAreaId();
    
    if (node->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) {
      fni->m_data->width.resize(1);
      fni->m_data->width[0] = 
        ((Cure::NavGraphGateway*)&(*node))->getWidth();
    }
    
    if (fni->m_WMid != "") {
      overwriteWorkingMemory<NavData::FNode>(fni->m_WMid,
                                             fni->m_data);      
    }
  }

  for (std::map<long,FNodeHolder>::iterator fni = m_FreeNodes.begin();
       fni != m_FreeNodes.end(); fni++) {

    if (fni->second.m_data->nodeId != id) continue;

    if ( ( (node->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) &&
           (fni->second.m_data->gateway == 1) ) ||
         ( (node->getType() != Cure::NavGraphNode::NODETYPE_GATEWAY) &&
           (fni->second.m_data->gateway == 0) ) ) break;
    
    fni->second.m_data->gateway = node->getType();
    fni->second.m_data->x = node->getX();
    fni->second.m_data->y = node->getY();
    fni->second.m_data->theta = node->getTheta();
    fni->second.m_data->areaId = node->getAreaId();
    
    if (node->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) {
      fni->second.m_data->width.resize(1);
      fni->second.m_data->width[0] = 
        ((Cure::NavGraphGateway*)&(*node))->getWidth();
    }
    
    if (fni->second.m_WMid != "") {
      overwriteWorkingMemory<NavData::FNode>(fni->second.m_WMid,
                                             fni->second.m_data);      
    }
  }
}

void NavGraphProcess::writeGraphToWorkingMemory(bool forceWrite)
{
  debug("writeGraphToWorkingMemory");

  debug("Writing %d fnodes to working memory", m_NewFNodes.size());

  bool wroteAnyPart = false;

  for (std::list<FNodeHolder>::iterator fni = m_NewFNodes.begin();
       fni != m_NewFNodes.end();) {    
    
    if (fni->m_data->gateway || fni->m_data->type.empty()) {
      checkAndAddArea(fni->m_data->areaId);
    } else {
      checkAndAddArea(fni->m_data->areaId,
                      fni->m_data->type[0].name,
                      fni->m_data->type[0].id);
    }
    fni->m_WMid = newDataID();

    debug("About to write to WM");

    addToWorkingMemory<NavData::FNode>(fni->m_WMid,
                                       fni->m_data); 
    
    log("Writing free node %d of type %d in area %d to WM", 
        (int)fni->m_data->nodeId, 
        fni->m_data->gateway, 
        (int)fni->m_data->areaId);

    
    m_FreeNodes.insert(std::make_pair(fni->m_data->nodeId, *fni));


    fni = m_NewFNodes.erase(fni);
    
    wroteAnyPart = true;
  }

  // Overwrite nodes that changed
  while (!m_ChangedNodes.empty()) {

    std::list< FNodeHolder* >::iterator i;
    i = m_ChangedNodes.begin();
    
    log("Write change for node %d to WM", (*i)->m_data->nodeId);
    overwriteWorkingMemory<NavData::FNode>((*i)->m_WMid,
                                           (*i)->m_data);    
    
    // Remove all instances of this so that we do not send out more
    // than one message about it
    for (std::list< FNodeHolder* >::iterator j = m_ChangedNodes.begin();
         j != m_ChangedNodes.end(); ) {
      if (*i == *j) {
        j = m_ChangedNodes.erase(j);          
      } else {
        j++;
      }
    }
  }

  // If we read area from file we make sure that they all exist and
  // that the area class is what it should be
  if (!m_AreasFromFile.empty()) {
    for (std::list<Area>::iterator ai = m_AreasFromFile.begin();
         ai != m_AreasFromFile.end(); ai++) {
      if (ai->m_data->type.empty()) {
        checkAndAddArea(ai->m_data->id);
      } else {
        checkAndAddArea(ai->m_data->id,
                        ai->m_data->type[0].name,
                        ai->m_data->type[0].id);
      }
    }
    m_AreasFromFile.clear();
  }

  debug("Writing %d edges to working memory", m_NewAEdges.size());
  for (std::list<AEdgeHolder>::iterator aei = m_NewAEdges.begin();
       aei != m_NewAEdges.end();) {

    log("Writing access edge, connecting nodes %d and %d to WM",
        (int)aei->m_data->startNodeId, 
        (int)aei->m_data->endNodeId);

    aei->m_WMid = newDataID();
    addToWorkingMemory<NavData::AEdge>(aei->m_WMid,
                                       aei->m_data); 
    
    m_AccessEdges.insert(std::make_pair(makeEdgeId(aei->m_data->startNodeId,
                                                   aei->m_data->endNodeId), 
                                        *aei));
    
    aei = m_NewAEdges.erase(aei);

    wroteAnyPart = true;
  }

  debug("Writing %d objects to working memory", m_NewObjData.size());
  for (std::list<ObjDataHolder>::iterator oni = m_NewObjData.begin();
       oni != m_NewObjData.end();) {
    oni->m_WMid = newDataID();
    addToWorkingMemory<NavData::ObjData>(oni->m_WMid,
                                         oni->m_data); 

    m_Objects.insert(std::make_pair(oni->m_data->objectId, *oni));

    oni = m_NewObjData.erase(oni);

    wroteAnyPart = true;
  }


  // Overwrite objects that changed
  log("Will write %d changed object to WM", m_ChangedObjects.size());
  while (!m_ChangedObjects.empty()) {

  std::list< ObjDataHolder* >::iterator i;
  i = m_ChangedObjects.begin();
  try {
	  lockEntry((*i)->m_WMid, cdl::LOCKEDO);
	  overwriteWorkingMemory<NavData::ObjData>((*i)->m_WMid,
			  (*i)->m_data);    
  }
  catch (DoesNotExistOnWMException) {
	  log("Error! Object data struct missing!");
  }
unlockEntry((*i)->m_WMid);

    // Remove all instances of this so that we do not send out more
    // than one message about it
    for (std::list< ObjDataHolder* >::iterator j = m_ChangedObjects.begin();
         j != m_ChangedObjects.end(); ) {
      if (*i == *j) {
        log("Written object of category %s to WM, can remove it now",
            (*i)->m_data->category.c_str());
        j = m_ChangedObjects.erase(j);          
      } else {
        j++;
      }
    }
  }
  log("May have written changed objects");

  // Write the composite graph to working memory
  if ((wroteAnyPart || forceWrite) && 
      (!m_FreeNodes.empty() || !m_Objects.empty()) ) {

    NavData::NavGraphPtr graph = new NavData::NavGraph;

    graph->objects.clear();

    if (!m_FreeNodes.empty()) {
      graph->fNodes.clear();
      for (std::map<long,FNodeHolder>::iterator iter = m_FreeNodes.begin();
           iter != m_FreeNodes.end(); iter++) {
        graph->fNodes.push_back(iter->second.m_data);
      }
    } else {
      graph->fNodes.clear();
    }
    if (!m_AccessEdges.empty()) {
      graph->aEdges.clear();
      for (std::map<long,AEdgeHolder>::iterator iter = m_AccessEdges.begin();
           iter != m_AccessEdges.end(); iter++) {
        graph->aEdges.push_back(iter->second.m_data);
      }
    } else {
      graph->aEdges.clear();
    }

    if (!m_Objects.empty()) {
      graph->objects.clear();
      for (std::map<long,ObjDataHolder>::iterator iter = m_Objects.begin();
           iter != m_Objects.end(); iter++) {
        graph->objects.push_back(iter->second.m_data);
      }
    } else {
      graph->objects.clear();
    }

    if(m_NavGraphWMid == ""){
    	debug("Write Graph to WM");
      m_NavGraphWMid = newDataID();
      addToWorkingMemory<NavData::NavGraph>(m_NavGraphWMid,
                                            graph); // sync!
      
      log("Added composite NavGraph to working memory");
    }else{
    	debug("Write Graph to WM");
      overwriteWorkingMemory<NavData::NavGraph>(m_NavGraphWMid,
                                                graph);      
    }

    debug("Wrote composite NavGraph with %d nodes, %d edges and %d objects to WM",
          graph->fNodes.size(), 
          graph->aEdges.size(), 
          graph->objects.size());
    
  } else {
    debug("NOT writing composite graph to WM");
  }

  if (!m_DontWriteFiles) saveGraphToFile(m_cureNavGraphFile);  
}

void NavGraphProcess::addFreeNode(long nodeId, 
                                  double x, double y, double z, double theta,
                                  long areaId, 
                                  const string &areaType, long areaTypeNo,
                                  short gateway, 
                                  double maxSpeed, double width,
                                  const std::string &name,
                                  bool tryToPullAreaType) 
{  
  // Add the free node
  FNodeHolder fnode;
  fnode.m_data = new NavData::FNode();
  fnode.m_data->x = x;
  fnode.m_data->y = y;
  fnode.m_data->theta = theta;
  fnode.m_data->areaId = areaId;
  fnode.m_data->nodeId = nodeId;
  fnode.m_data->gateway = gateway;
  if (gateway) {
    fnode.m_data->width.resize(1);
    fnode.m_data->width[0] = width;
  }

  if (areaType != "") {
    fnode.m_data->type.resize(1);
    fnode.m_data->type[0].name = areaType.c_str();  
    fnode.m_data->type[0].id = areaTypeNo;
  }

  m_NewFNodes.push_back(fnode);
  
  log("Added free node %d of type %d in area %d at pose (x=%.2f y=%.2f a=%.4f) (local repr)", 
      (int)fnode.m_data->nodeId, 
      fnode.m_data->gateway, 
      (int)fnode.m_data->areaId,
      fnode.m_data->x, fnode.m_data->y, fnode.m_data->theta);
}

void NavGraphProcess::addObject(double x, double y, double z, 
                                double camX, double camY,
                                long areaId,
                                const string &category, long objectId, double probability)
{
  // Add the object

  ObjDataHolder onode;
  onode.m_data = new NavData::ObjData();

  onode.m_data->x = x;
  onode.m_data->y = y;
  onode.m_data->z = z;
  onode.m_data->camX = camX;
  onode.m_data->camY = camY;
  onode.m_data->areaId = areaId;  
  onode.m_data->category = category;
  onode.m_data->probability= probability;
  onode.m_data->objectId = objectId;


  if (m_UniqueObjects) {
    
    // Check if this object already exists, in which case we update
    // the position of it
    for (std::map<long, ObjDataHolder>::iterator i = m_Objects.begin();
         i != m_Objects.end(); i++) {
      if (i->second.m_data->category == category) {
        *(i->second.m_data) = *(onode.m_data);

        log("Adding pointer to object 0x%X with category %s and areaId %d objectId %d", 
            &(i->second), i->second.m_data->category.c_str(),
            i->second.m_data->areaId, i->second.m_data->objectId);
        m_ChangedObjects.push_back( &(i->second) );
        println("Updated object of category %s and id %d in area %d (local)", 
                category.c_str(), objectId, areaId);
        return;
      }
    }

    // Change the data of any objects that have not yet been writte to
    // working memory
    for (std::list<ObjDataHolder>::iterator i = m_NewObjData.begin();
         i != m_NewObjData.end(); i++) {
      if (i->m_data->category == category) {
        i->m_data = onode.m_data;
        println("Updated new object of category %s and id %d in area %d (local)", 
                category.c_str(), objectId, areaId);
        return;
      }
    }
  } else {
    println("WARNING: Not dealing with non-unique objects. Will add a new instance for every observation");
  }
  
  m_NewObjData.push_back(onode);
  println("added object of category %s and id %d in area %d (local)", 
        category.c_str(), objectId, areaId);
}

void NavGraphProcess::addAccessEdge(long startNodeId, long endNodeId, 
                                    double weight) 
{  
  AEdgeHolder aedge;
  aedge.m_data = new NavData::AEdge();

  aedge.m_data->startNodeId = startNodeId;
  aedge.m_data->endNodeId = endNodeId;
  aedge.m_data->cost = weight;

  m_NewAEdges.push_back(aedge);

  log("Added access edge, connecting nodes %d and %d (local repr)",
      startNodeId, endNodeId);
}

void NavGraphProcess::writeTopologicalPosToWorkingMemory(int aid)
{
  if (!m_TopRobPos) m_TopRobPos = new NavData::TopologicalRobotPos;

  m_TopRobPos->areaId = aid;
  
  if(m_TopRobPosWMid == ""){
    m_TopRobPosWMid = newDataID();
    addToWorkingMemory<NavData::TopologicalRobotPos>(m_TopRobPosWMid, 
                       m_TopRobPos); 

    log("added TopologicalRobotPos to working memory");
  }else{
    overwriteWorkingMemory<NavData::TopologicalRobotPos>(m_TopRobPosWMid,
							 m_TopRobPos);
    
    log("changed TopologicalRobotPos on working memory to area %d",
        (int)m_TopRobPos->areaId);
  }
}

void NavGraphProcess::runComponent() 
{
  setupPushScan2d(*this);
  setupPushOdometry(*this);

  // If we're going to load a map, write a MapLoadStatus struct
  if (m_WriteFirstGraph) {
    SpatialData::MapLoadStatusPtr statusStruct 
      = new SpatialData::MapLoadStatus;
    statusStruct->nodesWritten = false;
    statusStruct->placesWritten = false;
    statusStruct->roomsWritten = false;
    statusStruct->categoryDataWritten = false;
    statusStruct->obstacleMapsLoaded = false;
    statusStruct->localMapsLoaded = false;

    m_MapLoadStatusWM = newDataID();
    addToWorkingMemory<SpatialData::MapLoadStatus>(m_MapLoadStatusWM, statusStruct);
  }

  while (isRunning()) {

    m_eventQueueMutex.lock();

    while(!m_graphEventQueue.empty()) {
      GraphEventType type = m_graphEventQueue.front().first;
      cdl::WorkingMemoryAddress addr = m_graphEventQueue.front().second;
      m_graphEventQueue.pop_front();
      
      m_eventQueueMutex.unlock();

      switch(type) {
	case NEW_OBJ_OBS:
	  processNewObjObs(addr);
	  break;
	case NEW_VISUAL_OBJECT:
	  processNewVisualObject(addr);
	  break;
	case NEW_DOOR_HYPTOHESIS:
	  processNewDoorHypothesis(addr);
	  break;
      }

      m_eventQueueMutex.lock();
    }
    m_eventQueueMutex.unlock();


    m_scanQueueMutex.lock();

    while(!m_scanQueue.empty()) {
      Cure::LaserScan2d scan = m_scanQueue.front();
      m_scanQueue.pop_front();

      m_scanQueueMutex.unlock();

      processScan(scan);

      m_scanQueueMutex.lock();
    }
    m_scanQueueMutex.unlock();

    sleepComponent(50); //Wait a bit for more events
  }
}

void NavGraphProcess::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  if (cureOdom.getTime().getDouble() <= 0) {
    log("Odometry not ready yet, time still 0");
    return;
  }

  log("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());

  IceUtil::Mutex::Lock lock(m_TOPPMutex);
  m_LastOdom = cureOdom;
  m_TOPP.addOdometry(cureOdom);
}

void NavGraphProcess::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
  
  Cure::Pose3D cp;
  cp.setTime(Cure::Timestamp(oobj->getData()->time.s, 
                             oobj->getData()->time.us));
  cp.setX(oobj->getData()->x);
  cp.setY(oobj->getData()->y);
  cp.setTheta(oobj->getData()->theta);

  if (cp.getTime().getDouble() <= 0) {
    log("RobotPose not ready yet, time still 0");
    return;
  }

  debug("newRobotPose called with x=%.3f y=%.3f theta=%.3f t=%.6f",
        cp.getX(), cp.getY(), cp.getTheta(), cp.getTime().getDouble());

  m_LastRobotPose = cp;  

  IceUtil::Mutex::Lock lock(m_TOPPMutex);
  
  m_TOPP.defineTransform(cp);
}


void NavGraphProcess::newVisualObject(const cast::cdl::WorkingMemoryChange & wmChange)
{
  IceUtil::Mutex::Lock lock(m_GraphMutex);

  m_graphEventQueue.push_back(pair<GraphEventType, cast::cdl::WorkingMemoryAddress>
      (NEW_VISUAL_OBJECT, wmChange.address));
}

void NavGraphProcess::processNewVisualObject(const cast::cdl::WorkingMemoryAddress & addr)
{
	VisionData::VisualObjectPtr visualObjectPtr = getMemoryEntry<VisionData::VisualObject>(addr);

	string category = visualObjectPtr->identLabels[0];
	double probability = visualObjectPtr->identDistrib[0];
	log("Observed object of category \"%s\" with probability %f", category.c_str(), probability);
	// FIXME: this threshold is magic be Michi Zillich
	if (probability<0.08)
		return;
    Cure::Timestamp t(visualObjectPtr->time.s, visualObjectPtr->time.us);

    // As a first approximation we use the last robot pose as the robot
    // pose when the object was observed
    Cure::Pose3D rp = m_LastRobotPose;

    {
    IceUtil::Mutex::Lock lock(m_TOPPMutex);
    if (!m_TOPP.isTransformDefined())
    {
    	log("TOPP transformation not defined yet -> GUESSING pose more or less");
    } else
    {
    	m_TOPP.getPoseAtTime(t, rp);
    }
    }
    rp.setTime(t);

    // Pose of the camera in world frame at the time of the observation
    Cure::Pose3D camPw;
    camPw.add(rp, m_CamPoseR);

    // The default distance is 1m
    double d = 1.0;
//    if (!oobj->getData()->dist.empty()) d = oobj->getData()->dist[0];

    double pan = 0, tilt = 0;
//    if (oobj->getData()->angles.size() > 0)
//    {
//    	pan = oobj->getData()->angles[0];
//    	if (oobj->getData()->angles.size() > 1)
//    	{
//    		tilt = oobj->getData()->angles[1];
//    	}
//    }

    // Object pos in the camera frame
    Cure::Pose3D objPc;
    objPc.setX(d);
    objPc.setY(d * tan(pan));
    objPc.setZ(d * tan(tilt));

    // Object pos in world frame
    Cure::Pose3D objPw;
    objPw.add(camPw, objPc);

    long objId = -1;

    long areaId = 0;
    if (m_TopRobPos) areaId = m_TopRobPos->areaId;

    {
      IceUtil::Mutex::Lock lock(m_GraphMutex);

      addObject(objPw.getX(), objPw.getY(), objPw.getZ(),
	  rp.getX(), rp.getY(),
	  areaId,
	  category, objId, probability);

      writeGraphToWorkingMemory();
    }
}

void NavGraphProcess::newObjObs(const cdl::WorkingMemoryChange &objID) 
{
  IceUtil::Mutex::Lock lock(m_GraphMutex);

  m_graphEventQueue.push_back(pair<GraphEventType, const cast::cdl::WorkingMemoryAddress> 
      (NEW_OBJ_OBS, objID.address));
}

void NavGraphProcess::processNewObjObs(const cdl::WorkingMemoryAddress &addr) 
{
  shared_ptr<CASTData<NavData::ObjObs> > oobj =
    getWorkingMemoryEntry<NavData::ObjObs>(addr);

  log("Observed object of category \"%s\"", oobj->getData()->category.c_str());

  Cure::Timestamp t(oobj->getData()->time.s, oobj->getData()->time.us);

  // As a first approximation we use the last robot pose as the robot
  // pose when the object was observed
  Cure::Pose3D rp = m_LastRobotPose;

  {
  IceUtil::Mutex::Lock lock(m_TOPPMutex);

  if (!m_TOPP.isTransformDefined()) {
    log("TOPP transformation not defined yet -> GUESSING pose more or less");
  } else {
    m_TOPP.getPoseAtTime(t, rp);
  }

  }

  rp.setTime(t);
  
  // Pose of the camera in world frame at the time of the observation
  Cure::Pose3D camPw;
  camPw.add(rp, m_CamPoseR);

  // The default distance is 1m
  double d = 1.0;
  if (!oobj->getData()->dist.empty()) d = oobj->getData()->dist[0];
  
  double pan = 0, tilt = 0;
  if (oobj->getData()->angles.size() > 0) {
    pan = oobj->getData()->angles[0];
    if (oobj->getData()->angles.size() > 1) {
      tilt = oobj->getData()->angles[1];
    }
  }

  // Object pos in the camera frame
  Cure::Pose3D objPc;
  objPc.setX(d);
  objPc.setY(d * tan(pan));
  objPc.setZ(d * tan(tilt));

  // Object pos in world frame
  Cure::Pose3D objPw;
  objPw.add(camPw, objPc);
  
  long objId = -1;
  if (!oobj->getData()->id.empty()) objId = oobj->getData()->id[0];

  long areaId = 0;
  if (m_TopRobPos) areaId = m_TopRobPos->areaId;
  
  {
    IceUtil::Mutex::Lock lock(m_GraphMutex);
    addObject(objPw.getX(), objPw.getY(), objPw.getZ(), 
	rp.getX(), rp.getY(),
	areaId,
	oobj->getData()->category, objId, 1.0);

    writeGraphToWorkingMemory();
  }
}

void NavGraphProcess::receiveScan2d(const Laser::Scan2d &castScan)
{
  // Transform the scan into the cure format
  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (cureScan.getTime().getDouble() <= 0) {
    log("Scan not ready yet, time still 0");
    return;
  }
  debug("Got scan with t=%.6f", cureScan.getTime().getDouble());

  IceUtil::Mutex::Lock lock(m_scanQueueMutex);

  m_scanQueue.push_back(cureScan);
}

void NavGraphProcess::processScan(Cure::LaserScan2d &cureScan)
{
  if (m_RemoveMotionBeams) {

    if (m_MotionDetector == 0) {
      if (m_LaserFreqEstNScans==0) {
        m_LaserFreqEstStartTime = cureScan.getDoubleTime();
      } else {
        if (m_LaserFreqEstNScans >= 10) {
          double dt = ( (cureScan.getDoubleTime() - m_LaserFreqEstStartTime) /
                        m_LaserFreqEstNScans );
          int numScans = int(2.0 / dt + 0.5);
          m_MotionDetector = new Cure::ScanMotionDetector(true, numScans);
          println("Using %d scans for MotionDetection", numScans);
        } 
      }

      log("Not yet found scan frequency, cannot calc #scans in buffer");
      m_LaserFreqEstNScans++;
    }
  }

  if (!m_TOPP.isTransformDefined()) {
    log("TOPP transformation not defined yet, cannot add nodes to graph");
    return;
  }

  // Check if we have to write the graph to WM for the first time
  // which happens when it is loaded from file
  if (m_WriteFirstGraph) {
    writeGraphToWorkingMemory();
    try {
      lockEntry(m_MapLoadStatusWM, cdl::LOCKEDOD);
      SpatialData::MapLoadStatusPtr loadStatus = 
	getMemoryEntry<SpatialData::MapLoadStatus>(m_MapLoadStatusWM);
      loadStatus->nodesWritten = true;
      overwriteWorkingMemory<SpatialData::MapLoadStatus>(m_MapLoadStatusWM, loadStatus);
      unlockEntry(m_MapLoadStatusWM);
    }
    catch (DoesNotExistOnWMException)
    {
      getLogger()->warn("LoadMapStatus struct disappeared from WM!");
    }
    m_WriteFirstGraph = false;
  }

  // Get the current robot position
  Cure::Pose3D cp;
  bool status; 
  {
    IceUtil::Mutex::Lock lock(m_TOPPMutex);
    status = m_TOPP.getPoseAtTime(cureScan.getTime(), cp) != 0;
  }

  if (status) {
    debug("Failed to get robot pose at the time of the scan, cannot add nodes");
    debug("Wanted robot pose at t=%.6f (last odometry t=%.6f, robot pose t=%.6f)",
        cureScan.getDoubleTime(), m_LastOdom.getDoubleTime(),
        m_LastRobotPose.getDoubleTime());
    return;
  }

  cp = m_LastRobotPose;
  debug("Check if node should be placed at x=%.3f y=%.3f theta=%.3f (t=%.6f)",
        cp.getX(), cp.getY(), cp.getTheta(), cp.getTime().getDouble());
  
  {
  IceUtil::Mutex::Lock lock(m_GraphMutex);

  int ret=m_cureNavGraph.maybeAddNewNodeAt(cp.getX(),cp.getY(),cp.getTheta());

  debug("maybeAddNewNodeAt returned %d", ret);
  
  if (ret == 1) { 

    debug("maybeAddNewNodeAt returned %d", ret);

    if (processNewCureNavGraphNode()) {
    	checkForNodeChanges();
    }
    
  } else if (ret == 2) {

    log("maybeAddNewNodeAt returned %d", ret);

    // This happends when areas have been merged. We have to check if
    // any of the nodes have changed
    checkForNodeChanges();    

  }
  }

  if (m_MotionDetector) {
    Cure::Pose3D scanOdomPose;
    bool status;
    {
      IceUtil::Mutex::Lock lock(m_TOPPMutex);
      status = m_TOPP.getOdomAtTime(cureScan.getTime(), scanOdomPose) != 0;
    }

    if (status) {
      debug("Failed to get odom pose at time of scan");
      m_MotionDetector->m_Movements.clear();
    } else {
      m_MotionDetector->checkForMotion(cureScan.getTime().getDouble(),
                                       cureScan.getNPts(),
                                       cureScan.getRanges(),
                                       cureScan.getStartAngle(),
                                       cureScan.getAngleStep(),
                                       m_LaserPoseR.getX(), 
                                       m_LaserPoseR.getY(), 
                                       m_LaserPoseR.getTheta(), 
                                       scanOdomPose.getX(),
                                       scanOdomPose.getY(),
                                       scanOdomPose.getTheta());
    }
  }


  // Check if we have moved enough since last we check if we were
  // in a door
  const double minMoveDistToCheck = 0.01;
  if (hypot(m_LastDoorCheckPose.getX() - cp.getX(),
            m_LastDoorCheckPose.getY() - cp.getY()) > minMoveDistToCheck) {
    if (m_InDoor) {
      // Check if we have moved enough since adding the door to
      // say that we are no longer in a door
      const double minDoorDist = 1.0;
      if (hypot(m_LastDoorPose.getX() - cp.getX(),
                m_LastDoorPose.getY() - cp.getY()) > minDoorDist) {
        m_InDoor = false;
      }
    }
    
    if (!m_InDoor) {

      Cure::LaserScan2d doorScan(cureScan);

      if (m_MotionDetector &&
	  !m_MotionDetector->m_Movements.empty()) {

	// Go tthrough the moving objects and adjust any beams
	// associated with these to very far away readinsg so they
	// will not triger doors

	for (unsigned int i = 0; i<m_MotionDetector->m_Movements.size(); i++){
	  Cure::ScanMotionDetector::Movement &m = 
	    m_MotionDetector->m_Movements[i];
	  for (int j = 0; j < m.nPts; j++) {
	    doorScan(m.scanIndex[j]) = 1e10;
	  }
	}
      }

      if (m_DoorDetector.inDoorOpening(doorScan, 
	    m_MinDoorWidth, 
	    m_MaxDoorWidth)) {
	double minDistSq = DBL_MAX;
	if (m_UseDoorHypotheses) {
	  for (vector<pair<double, double> >::iterator it = m_doorHypPositions.begin();
	      it != m_doorHypPositions.end(); it++) {
	    double distSq = (cp.getX()-it->first)*(cp.getX()-it->first) + 
	      (cp.getY()-it->second)*(cp.getY()-it->second);
	    if (distSq < minDistSq) minDistSq = distSq;
	  }
	}
	if (!m_UseDoorHypotheses || minDistSq < 4.0) {

	  log("Door is detected angR=%.2fdeg angL=%.2fdeg",
	      m_DoorDetector.m_AngleR*180/M_PI,
	      m_DoorDetector.m_AngleL*180/M_PI);

	  // Get the pose of the laser at the time of the scan 
	  Cure::Pose3D lpW;
	  lpW.add(cp, m_LaserPoseR);

	  // Now calculate the position of the door posts
	  double xR = lpW.getX() + (m_DoorDetector.m_RangeR * cos(lpW.getTheta() + m_DoorDetector.m_AngleR));
	  double yR = lpW.getY() + (m_DoorDetector.m_RangeR * sin(lpW.getTheta() + m_DoorDetector.m_AngleR));
	  double xL = lpW.getX() + (m_DoorDetector.m_RangeL * cos(lpW.getTheta() + m_DoorDetector.m_AngleL));
	  double yL = lpW.getY() + (m_DoorDetector.m_RangeL * sin(lpW.getTheta() + m_DoorDetector.m_AngleL));

	  // Get the position and orientation of the door
	  double x = 0.5 * (xR + xL);
	  double y = 0.5 * (yR + yL);
	  double dir = atan2(yR - yL, xR - xL);

	  log("Estimated door pose to x=%.2f y=%.2f theta=%fdeg lpW.x=%.2f lpW.y=%.2f lpW.theta=%.2fdeg rR=%.2f rL=%.2f",
	      x,y,dir*180/M_PI,lpW.getX(),lpW.getY(),lpW.getTheta()*180.0/M_PI,m_DoorDetector.m_RangeR,m_DoorDetector.m_RangeL);


	  {
	  IceUtil::Mutex::Lock lock(m_GraphMutex);
	  //long lastid = m_cureNavGraph.m_Nodes.back()->getId();
	  unsigned long lastsize = m_cureNavGraph.m_Nodes.size();
	  m_cureNavGraph.addGatewayNode(x,y,dir,
	      m_DoorDetector.m_Width);
	  if (m_cureNavGraph.m_Nodes.size() == lastsize) {
	    checkForNodeChanges();
	  } else {
	    addFreeNode(m_cureNavGraph.m_Nodes.back()->getId(), 
		//lpW.getX(), lpW.getY(), 0.0, cp.getTheta()-M_PI_2,
		x, y, 0.0, dir,
		m_cureNavGraph.m_Nodes.back()->getAreaId(), 
		"Area", 0,
		1, 
		m_cureNavGraph.m_Nodes.back()->getMaxSpeed(), 
		m_DoorDetector.m_Width,
		"-",
		true); // ask for the area class!!
	    writeGraphToWorkingMemory();
	  } 
	  }

	  //if (processNewCureNavGraphNode(true)) checkForNodeChanges();     

	  m_LastDoorPose = cp;
	  m_InDoor = true;
	}
      }
    }       
  }
  m_LastDoorCheckPose = cp;  
  
  if(m_WriteFirstTopologicalPose){
    debug("Writing topological position: %d", 
          m_cureNavGraph.getCurrentAreaId());
    writeTopologicalPosToWorkingMemory(m_cureNavGraph.getCurrentAreaId());
    m_WriteFirstTopologicalPose = false;
  }
}

int NavGraphProcess::processNewCureNavGraphNode(bool knownToBeGateway,
                                                double w)
{
  debug("processNewCureNavGraphNode");
  Cure::NavGraphNode &lastNode = *(m_cureNavGraph.m_Nodes.back());
 
  if (!m_FreeNodes.empty() &&
      m_FreeNodes.find(lastNode.getId()) != m_FreeNodes.end()) {
    debug("No need to add node with id %d, already added", lastNode.getId());
    return 1;
  }

  // new node added	
  short gateway = 0;
  double width = 1.0;
  if (knownToBeGateway ||
      (lastNode.getType() == Cure::NavGraphNode::NODETYPE_GATEWAY)) {
    width = w;
    gateway = 1;
  }

  debug("Cure added new node id=%d (aid=%d, g=%d, w=%.1f) at x=%.2f y=%.2f a=%.4f", lastNode.getId(), lastNode.getAreaId(), gateway, width, lastNode.getX(), lastNode.getY(), lastNode.getTheta());
  
  addFreeNode(lastNode.getId(),
              lastNode.getX(), 
              lastNode.getY(), 
              0.0, 
              lastNode.getTheta(),
              lastNode.getAreaId(), 
              "Area", 0,
              gateway,
              lastNode.getMaxSpeed(), width,
              lastNode.getName(),
              true);

  writeGraphToWorkingMemory();
  
  return 0;
}

void NavGraphProcess::checkForNodeChanges()
{
  debug("checkForNodeChanges");

  // Got through the nodes in the Cure::NavGraph and check that they
  // have the same type as in the list of FreeNodes

  bool changesMade = false;

  std::list<Cure::NavGraphNode*>::iterator ni;
  for (ni = m_cureNavGraph.m_Nodes.begin();
       ni != m_cureNavGraph.m_Nodes.end(); ni++) {
    
    bool foundMatch = false;
    
    std::map<long, FNodeHolder>::iterator fni;
    for (fni = m_FreeNodes.begin(); fni != m_FreeNodes.end(); fni++) {
      
      if ((*ni)->getId() == fni->second.m_data->nodeId) {
        foundMatch = true;
        
        if ((*ni)->getType() != fni->second.m_data->gateway) {

          log("Node %d changed type from %d to %d",
              (*ni)->getId(), fni->second.m_data->gateway, (*ni)->getType());

          fni->second.m_data->gateway = (*ni)->getType();
          fni->second.m_data->x = (*ni)->getX();
          fni->second.m_data->y = (*ni)->getY();
          fni->second.m_data->theta = (*ni)->getTheta();
          fni->second.m_data->areaId = (*ni)->getAreaId();
          
          if ((*ni)->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) {
            fni->second.m_data->width.resize(1);
            fni->second.m_data->width[0] = 
              ((Cure::NavGraphGateway*)&(**ni))->getWidth();
          }

          m_ChangedNodes.push_back( &(fni->second) );
	  changesMade = true;
        }
        
        break;
      }
    }
    
    if (!foundMatch) {
      debug("WARNING: Found a node (id=%d) that was not added before",
              (*ni)->getId());
    }
  }

  if (changesMade) writeGraphToWorkingMemory(true);
}

void 
NavGraphProcess::newDoorHypothesis(const cast::cdl::WorkingMemoryChange &objID)
{
  IceUtil::Mutex::Lock lock(m_eventQueueMutex);

  m_graphEventQueue.push_back(pair<GraphEventType, const cdl::WorkingMemoryAddress> 
      (NEW_DOOR_HYPTOHESIS, objID.address));
}

void 
NavGraphProcess::processNewDoorHypothesis(const cast::cdl::WorkingMemoryAddress &addr)
{
  try {
    FrontierInterface::DoorHypothesisPtr doorHyp =
      getMemoryEntry<FrontierInterface::DoorHypothesis>(addr);

    if (doorHyp != 0) {
      double doorX = doorHyp->x;
      double doorY = doorHyp->y;
      log("Adding door hypothesis at (%f, %f)", doorX, doorY);

      IceUtil::Mutex::Lock lock(m_GraphMutex);

      m_doorHypPositions.push_back(pair<double, double>(doorX, doorY));
    }
  }
  catch (DoesNotExistOnWMException) {
  }
}
