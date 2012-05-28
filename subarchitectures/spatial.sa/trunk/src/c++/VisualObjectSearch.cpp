#include "VisualObjectSearch.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <stdlib.h>
#include "CureMapConversion.hpp"
#include <CureHWUtils.hpp>
#include <AddressBank/ConfigFileReader.hh>    
#include <NavData.hpp>
#include <SpatialData.hpp>
#include <VisionData.hpp>
#include <iostream>
#include <fstream>
#include "XVector3D.h"
#include "GridDataFunctors.hh"
#include <cast/core/CASTUtils.hpp>
#include <cogxmath.h>
#include "Matrix33.h"
#include <Math.hpp>
#include <Utils/HelpFunctions.hh>

namespace spatial {
using namespace cast;
using namespace spatial;
using namespace std;
using namespace boost;
using namespace SpatialGridMap;
using namespace cogx;
using namespace Math;
extern "C" {
cast::CASTComponentPtr newComponent() {
  return new VisualObjectSearch();
}
}
VisualObjectSearch::VisualObjectSearch() :
  m_sampler(0) {
  // TODO Auto-generated constructor stub
  // If we're not building the map it means we're using an already built one. Hence, read it.		         
  m_bEvaluation = false;
}
VisualObjectSearch::~VisualObjectSearch() {
  // TODO Auto-generated destructor stub
  log("Destructor called.");
}

spatial::VisualObjectSearch* AVSComponentPtr;
void VisualObjectSearch::configure(
    const std::map<std::string, std::string>& _config) {

  m_generateviewcones = false;
  m_threshold = 0.7;
  m_currentRoom = 1;
  m_totalprob = 0;
  maxnumberofcones = 10;
  m_policyManager.m_CurrentRoom = 0;
  isRunComponent = false;
  m_totalViewPoints = 0;
  isSearchFinished = false;
  AVSComponentPtr = this;
  m_policyManager.m_evalpol = this;
  m_ignoreTilt = false;
  failedPolicyStep = "Unknown";
  map<string, string>::const_iterator it = _config.find("-c");
  if (it == _config.end()) {
    println("configure(...) Need config file (use -c option)\n");
    std::abort();
  }
  std::string configfile = it->second;

  Cure::ConfigFileReader *cfg;
  if (it != _config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(it->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }
  cfg->getRoboLookHost(m_PbHost);
  std::string usedCfgFile, tmp;
  if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
    m_PbHost = tmp;
  }

  m_ignoreTilt = false;
  if (_config.find("--ignore-tilt") != _config.end())
    m_ignoreTilt = true;

  m_bSimulation = false;
  if (_config.find("--simulate") != _config.end())
    m_bSimulation = true;

  m_nobaseobject = false;
  if (_config.find("--no-support-object") != _config.end())
    m_nobaseobject = true;

  m_usePeekabot = false;
  if (_config.find("--usepeekabot") != _config.end())
    m_usePeekabot = true;

  m_publishSimCones = false;
  if (_config.find("--publishsimcones") != _config.end())
    m_publishSimCones = true;

  if (cfg->getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose for laser");
    std::abort();
  }

  m_mapceiling = 2.0;
  it = _config.find("--mapceiling");
  if (it != _config.end()) {
    m_mapceiling = (atof(it->second.c_str()));
    log("Map ceiling set to: %d", m_mapceiling);
  }

  m_threshold = 0.7;
  it = _config.find("--cov-threshold");
  if (it != _config.end()) {
    m_threshold = (atof(it->second.c_str()));
    log("Map coverage set to: %d", m_threshold);
  }

  m_sampleawayfromobs = 0.2;
  it = _config.find("--sampleawayfromobs");
  if (it != _config.end()) {
    m_sampleawayfromobs = (atof(it->second.c_str()));
    log("Distance to nearest obs for samples set to: %f", m_sampleawayfromobs);
  }

  m_best3DConeRatio = 0.1;
  it = _config.find("--3Dconeratio ");
  if (it != _config.end()) {
    m_best3DConeRatio = (atof(it->second.c_str()));
    log("Best 3D cone ratio set to: %d", m_best3DConeRatio);
  }

  m_tiltinterval = 10 * M_PI / 180;
  it = _config.find("--tiltinterval");
  if (it != _config.end()) {
    m_tiltinterval = (atof(it->second.c_str())) * M_PI / 180;
    log("Tilt Interval set to: %f", m_tiltinterval);
  }
  log("Tilt Interval set to: %f", m_tiltinterval);

  m_samplesize = 100;
  it = _config.find("--samplesize");
  if (it != _config.end()) {
    m_samplesize = (atof(it->second.c_str()));
    log("Samplesize set to: %d", m_samplesize);
  }

  if ((it = _config.find("--tpose")) != _config.end()) {
    istringstream istr(it->second);
    string label;

    istr >> label;
    m_tablelabel = label;
    istr >> label;
    tablepose.setX(atof(label.c_str()));
    istr >> label;
    tablepose.setY(atof(label.c_str()));
    istr >> label;
    tablepose.setTheta(atof(label.c_str()));
  }
  if ((it = _config.find("--objects")) != _config.end()) {
    istringstream istr(it->second);
    string label;
    while (istr >> label) {
      m_objectlist.push_back(label);
    }
  }
  log("Loaded objects.");
  for (unsigned int i = 0; i < m_objectlist.size(); i++)
    cout << m_objectlist[i] << endl;

  m_gridsize = 200;
  m_cellsize = 0.05;
  it = _config.find("--gridsize");
  if (it != _config.end()) {

    m_gridsize = (atoi(it->second.c_str()));
    log("Gridsize set to: %d", m_gridsize);
  }
  it = _config.find("--cellsize");
  if (it != _config.end()) {
    m_cellsize = (atof(it->second.c_str()));
    log("Cellsize set to: %f", m_cellsize);
  }

  m_minbloxel = 0.05;
  it = _config.find("--minbloxel");
  if (it != _config.end()) {
    m_minbloxel = (atof(it->second.c_str()));
    log("Min bloxel height set to: %f", m_minbloxel);
  }

  it = _config.find("--orientations");
  if (it != _config.end()) {
    m_sampler.setOrientationQuantization(atoi(it->second.c_str()));
  }

  it = _config.find("--samples");
  if (it != _config.end()) {
    m_sampler.setSampleNumberTarget(atoi(it->second.c_str()));
  }

  it = _config.find("--kernel-width-factor");
  if (it != _config.end()) {
    m_sampler.setKernelWidthFactor(atoi(it->second.c_str()));
  }

  m_horizangle = M_PI / 4;
  it = _config.find("--cam-horizangle");
  if (it != _config.end()) {
    m_horizangle = (atof(it->second.c_str())) * M_PI / 180.0;
    log("Camera FoV horizontal angle set to: %f", m_horizangle);
  }

  m_vertangle = M_PI / 4;
  it = _config.find("--cam-vertangle");
  if (it != _config.end()) {
    m_vertangle = (atof(it->second.c_str())) * M_PI / 180.0;
    log("Camera FoV vertical angle set to: %f", m_vertangle);
  }

  m_conedepth = 2.0;
  it = _config.find("--cam-conedepth");
  if (it != _config.end()) {
    m_conedepth = (atof(it->second.c_str()));
    log("Camera view cone depth set to: %f", m_conedepth);
  }
  m_minDistance = m_conedepth / 4.0;
  if (m_minDistance < 0.5)
    m_minDistance = 0.5;

  m_showgui = false;
  it = _config.find("--showgui");
  if (it != _config.end()) {
    m_showgui = true;
    if (m_showgui)
      log("Save map mode : on");
    else
      log("Save map mode : off");
  }

  m_showconemap = false;
  it = _config.find("--showconemap");
  if (it != _config.end()) {
    m_showconemap = true;
  }

  m_usePTZ = false;
  if (_config.find("--ctrl-ptu") != _config.end()) {
    m_usePTZ = true;
    log("will use ptu");
  }

  m_curemapfile = "";
  if ((it = _config.find("--curemap-file")) != _config.end()) {
    istringstream istr(it->second);
    istr >> m_curemapfile;
  }

  m_pout = 0.001;
  log("Cure map file: %s", m_curemapfile.c_str());
  GridMapData def;
  def.occupancy = UNKNOWN;
  //std::vector< pair<std::string,double> > objectprobability;
  //objectprobability.push_back(make_pair("ricebox",0));
  //def.objprob = objectprobability;
  def.pdf = 0;
  m_tempmap = new SpatialGridMap::GridMap<GridMapData>(m_gridsize, m_gridsize,
      m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
  m_map = new SpatialGridMap::GridMap<GridMapData>(m_gridsize, m_gridsize,
      m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
  //m_tracer = new LaserRayTracer<GridMapData>(m_map,1.0);

  m_lgm = new CureObstMap(m_gridsize / 2, m_cellsize, '2', CureObstMap::MAP1);

  // m_Glrt  = new Cure::ObjGridLineRayTracer<unsigned char>(*m_lgm);
  if (m_usePeekabot) {
    pbVis = new VisualPB_Bloxel(m_PbHost, 5050, m_gridsize, m_gridsize,
        m_cellsize, 1, true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time
    pbVis->connectPeekabot();
  } else {
    pbVis = 0;
  }
  if (m_usePTZ) {
    log("connecting to PTU");
    Ice::CommunicatorPtr ic = getCommunicator();

    Ice::Identity id;
    id.name = "PTZServer";
    id.category = "PTZServer";

    std::ostringstream str;
    str << ic->identityToString(id) << ":default" << " -h localhost" << " -p "
        << cast::cdl::CPPSERVERPORT;

    Ice::ObjectPrx base = ic->stringToProxy(str.str());
    m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
  }

  gotPC = false;

}

double VisualObjectSearch::GetGraphPathLength(double xS, double yS, double aS,
    double xG, double yG, double aG) {

  NavData::NavGraphInterfacePrx agg(getIceServer<NavData::NavGraphInterface> (
      "navgraph.process"));
  double d = agg->getPathLength(xS, yS, aS, xG, yG, aG);
  return d;
}

double VisualObjectSearch::GetPlaceIdFromNodeId(int nodeId) {
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  SpatialData::PlacePtr p = new SpatialData::Place;
  p = agg->getPlaceFromNodeID(nodeId);
  if (p != NULL)
    return p->id;
  else
    return -1;
}
int VisualObjectSearch::GetClosestNodeId(double x, double y, double a) {

  NavData::NavGraphInterfacePrx agg(getIceServer<NavData::NavGraphInterface> (
      "navgraph.process"));
  int d = agg->getClosestNodeId(x, y, a, 3);
  return d;
}

int VisualObjectSearch::GetAreaId(double x, double y, double a) {

  NavData::NavGraphInterfacePrx agg(getIceServer<NavData::NavGraphInterface> (
      "navgraph.process"));
  int d = agg->getAreaId(x, y, a, 3);
  return d;
}

void VisualObjectSearch::IcetoCureLGM(SpatialData::LocalGridMap icemap,
    CureObstMap* lgm) {
  log(
      "icemap.size: %d, icemap.data.size %d, icemap.cellSize: %f, centerx,centery: %f,%f",
      icemap.size, icemap.data.size(), icemap.cellSize, icemap.xCenter,
      icemap.yCenter);
  int lp = 0;
  for (int x = -icemap.size; x <= icemap.size; x++) {
    for (int y = -icemap.size; y <= icemap.size; y++) {
      (*lgm)(x, y) = (icemap.data[lp]);
      lp++;
    }
  }
  log("converted icemap to Cure::LocalGridMap");
}
void VisualObjectSearch::InitializeMaps(SpatialData::PlaceIDSeq placestosearch) {
  if (placestosearch.size() == 0) {
    // Read in the full map
    // Then read local room maps

    // Ideally we would get the number of rooms and belonging nodes from somewhere but here we hardcode it. 
    // Get the combined map of room1

    for (int i = 0; i < 11; i++)
      placestosearch.push_back(i);
    log("creating placeinterface proxy");
    SpatialData::LocalGridMap combined_lgm;
    log("getting combined lgm");
    FrontierInterface::LocalMapInterfacePrx agg2(getIceServer<
        FrontierInterface::LocalMapInterface> ("map.manager"));
    for (unsigned int g = 0; g < placestosearch.size(); g++)
      log("%d", placestosearch[g]);
    combined_lgm = agg2->getCombinedGridMap(placestosearch);
    log("have combined lgm");
    m_lgms[1] = new CureObstMap(combined_lgm.size, m_cellsize, '2',
        CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);

    IcetoCureLGM(combined_lgm, m_lgms[1]);

    // Fill in Bloxel Map 
    GridMapData def;
    def.occupancy = UNKNOWN;
    def.pdf = 0;
    SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* tmpmap;
    tmpmap = new SpatialGridMap::GridMap<GridMapData>(
        combined_lgm.size * 2 + 1, combined_lgm.size * 2 + 1, m_cellsize,
        m_minbloxel, 0, m_mapceiling, combined_lgm.xCenter,
        combined_lgm.yCenter, 0, def);
    m_maps[1] = tmpmap;
    GDMakeObstacle makeobstacle;
    for (int x = -combined_lgm.size; x < combined_lgm.size; x++) {
      for (int y = -combined_lgm.size; y < combined_lgm.size; y++) {
        if ((*m_lgms[1])(x, y) == '1') {
          m_maps[1]->boxSubColumnModifier(x + combined_lgm.size, y
              + combined_lgm.size, m_LaserPoseR.getZ(), m_minbloxel * 2,
              makeobstacle);
        }
      }
    }

    // Get the combined map of room2
    placestosearch.clear();
    for (int i = 17; i < 24; i++)
      placestosearch.push_back(i);
    combined_lgm = agg2->getCombinedGridMap(placestosearch);
    log("have combined lgm");
    m_lgms[2] = new CureObstMap(combined_lgm.size, m_cellsize, '2',
        CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);
    IcetoCureLGM(combined_lgm, m_lgms[2]);

    tmpmap = new SpatialGridMap::GridMap<GridMapData>(
        combined_lgm.size * 2 + 1, combined_lgm.size * 2 + 1, m_cellsize,
        m_minbloxel, 0, m_mapceiling, combined_lgm.xCenter,
        combined_lgm.yCenter, 0, def);
    m_maps[2] = tmpmap;
    for (int x = -combined_lgm.size; x < combined_lgm.size; x++) {
      for (int y = -combined_lgm.size; y < combined_lgm.size; y++) {
        if ((*m_lgms[2])(x, y) == '1') {
          m_maps[2]->boxSubColumnModifier(x + combined_lgm.size, y
              + combined_lgm.size, m_LaserPoseR.getZ(), m_minbloxel * 2,
              makeobstacle);
        }
      }
    }
    if (m_usePeekabot) {
      pbVis->Display2DCureMap(m_lgms[1], "room1");
      pbVis->Display2DCureMap(m_lgms[2], "room2");
    }
  } else {
    log("creating placeinterface proxy for specific areas");
    SpatialData::LocalGridMap combined_lgm;
    log("getting combined lgm");
    FrontierInterface::LocalMapInterfacePrx agg2(getIceServer<
        FrontierInterface::LocalMapInterface> ("map.manager"));
    for (unsigned int g = 0; g < placestosearch.size(); g++)
      log("%d", placestosearch[g]);
    combined_lgm = agg2->getCombinedGridMap(placestosearch);
    log("have combined lgm");
    m_lgms[1] = new CureObstMap(combined_lgm.size, m_cellsize, '2',
        CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);

    IcetoCureLGM(combined_lgm, m_lgms[1]);

    // Fill in Bloxel Map 
    log("filling in Bloxel map");
    GridMapData def;
    def.occupancy = UNKNOWN;
    def.pdf = 1;
    SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* tmpmap;
    tmpmap = new SpatialGridMap::GridMap<GridMapData>(
        combined_lgm.size * 2 + 1, combined_lgm.size * 2 + 1, m_cellsize,
        m_minbloxel, 0, m_mapceiling, combined_lgm.xCenter,
        combined_lgm.yCenter, 0, def);
    normalizePDF(*tmpmap, 1.0);
    m_maps[1] = tmpmap;
    GDMakeObstacle makeobstacle;
    for (int x = -combined_lgm.size; x < combined_lgm.size; x++) {
      for (int y = -combined_lgm.size; y < combined_lgm.size; y++) {
        if ((*m_lgms[1])(x, y) == '1') {
          m_maps[1]->boxSubColumnModifier(x + combined_lgm.size, y
              + combined_lgm.size, m_LaserPoseR.getZ(), m_minbloxel * 2,
              makeobstacle);
        }
      }
    }

    if (m_usePeekabot)
      pbVis->Display2DCureMap(m_lgms[1], "room1");
  }
  log("filled");
}

void VisualObjectSearch::start() {
  log("I have started");
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newRobotPose));

  addChangeFilter(createLocalTypeFilter<SpatialData::SpatialObject> (cdl::ADD),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newSpatialObject));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newRobotPose));

  /*addChangeFilter(createLocalTypeFilter<
   SpatialData::NavCommand> (cdl::OVERWRITE),
   new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
   &VisualObjectSearch::owtNavCommand));*/

  /* addChangeFilter(createLocalTypeFilter<
   FrontierInterface::ObjectTiltAngleRequest> (cdl::OVERWRITE),
   new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
   &VisualObjectSearch::owtTiltAngleRequest));
   */
  addChangeFilter(createGlobalTypeFilter<VisionData::Recognizer3DCommand> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<VisualObjectSearch> (
      this, &VisualObjectSearch::owtRecognizer3DCommand));

  /*  addChangeFilter(createGlobalTypeFilter<
   VisionData::ARTagCommand> (cdl::OVERWRITE),
   new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
   &VisualObjectSearch::owtARTagCommand));*/

  addChangeFilter(
      createGlobalTypeFilter<FrontierInterface::ObjectPriorRequest> (
          cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::owtWeightedPointCloud));

  /*addChangeFilter(createGlobalTypeFilter<
   VisionData::VisualObject> (cdl::OVERWRITE),
   new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
   &VisualObjectSearch::newVisualObject));*/

  addChangeFilter(createGlobalTypeFilter<VisionData::VisualObject> (cdl::ADD),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newVisualObject));

  addChangeFilter(createGlobalTypeFilter<
      SpatialData::ViewPointGenerationCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newViewPointGenerationCommand));

  addChangeFilter(
      createGlobalTypeFilter<SpatialData::ProcessViewPointCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
          &VisualObjectSearch::newProcessViewPointCommand));
}

void VisualObjectSearch::newProcessViewPointCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("got new ProcessViewPointCommand");

  SpatialData::ProcessViewPointCommandPtr newProcessVP = getMemoryEntry<
      SpatialData::ProcessViewPointCommand> (objID.address);
  m_ProcessVPID = objID.address.id;
  m_objectlist = newProcessVP->objectModels;
  m_tilt = newProcessVP->vp->tilt;
  Cure::Pose3D pos;
  pos.setX(newProcessVP->vp->pose.x);
  pos.setY(newProcessVP->vp->pose.y);
  pos.setTheta(newProcessVP->vp->pose.z);
  log("posting nav command");
  PostNavCommand(pos, SpatialData::GOTOPOSITION);

}
void VisualObjectSearch::newViewPointGenerationCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    log("got new ViewPointGenerationCommand");
    SpatialData::ViewPointGenerationCommandPtr newVPCommand = getMemoryEntry<
        SpatialData::ViewPointGenerationCommand> (objID.address);

    targetObject = newVPCommand->label;
    m_viewpointgen_id = objID.address.id;
    m_placestosearch = newVPCommand->placestosearch;
    m_generateviewcones = true;
  } catch (DoesNotExistOnWMException e) {
    log("Error! SpatialObject disappeared from WM!");
  }
}

void split(const string& str, const string& delim, vector<string>& parts) {
  size_t start, end = 0;
  while (end < str.size()) {
    start = end;
    while (start < str.size() && (delim.find(str[start]) != string::npos)) {
      start++; // skip initial whitespace
    }
    end = start;
    while (end < str.size() && (delim.find(str[end]) == string::npos)) {
      end++; // skip to end of word
    }
    if (end - start != 0) { // just ignore zero-length strings.
      parts.push_back(string(str, start, end - start));
    }
  }
}

void VisualObjectSearch::getStructuredStrategy(std::string strategy,
    std::vector<ObjectPairRelation> &singleStrategy) {
  ObjectPairRelation rel;
  std::vector<std::string> parts;
  split(strategy, " ", parts);
  log("parsing %s", strategy.c_str());
  if (parts.size() < 3) {
    if (parts.size() == 1) {
      log("room string");
      rel.primaryobject = parts[0];
      rel.secobject = "";
      singleStrategy.push_back(rel);
    } else {
      log(
          "there is something wrong with the policy string, returning with doing nothing");
      return;
    }
  } else {
    for (unsigned int i = parts.size() - 1; i >= 2; i = i - 2) {
      log("got relation: %s %s %s", parts[i - 2].c_str(), parts[i - 1].c_str(),
          parts[i].c_str());
      rel.primaryobject = parts[i - 2];
      rel.relation = parts[i - 1] == "ON" ? FrontierInterface::ON
          : FrontierInterface::IN;
      rel.secobject = parts[i];
      singleStrategy.push_back(rel);
    }
  }
}

double VisualObjectSearch::GetCostForSingleStrategy(
    GridMap<GridMapData>* tmpMap, std::string targetObject, double threshold,
    bool ishypo) {
  SetCurrentTarget(targetObject);
  m_pout = 0.001;
  double totalprob = 0;
  double cost = 0;
  int count = 1;
  vector<SensingAction> plan;
  Cure::Pose3D start, dest;
  SpatialData::ViewPointSeq viewpoints;
  while (totalprob <= threshold) {
    log("total prob mass of cones for strategy step so far: %f", totalprob);
    SensingAction nbv = SampleAndSelect(tmpMap);
    if (nbv.totalprob < 0) {
      log(
          "Probably you have a map with no obstacles, returning without doing anything now.");
      break;
    }
    plan.push_back(nbv);
    totalprob += nbv.totalprob;
    UnsuccessfulDetection(nbv, tmpMap);
    dest.setX(nbv.pos[0]);
    dest.setY(nbv.pos[1]);
    dest.setTheta(nbv.pan);
    if (count != 0) {
      if (!ishypo) {
        cost += GetGraphPathLength(start.getX(), start.getY(),
            start.getTheta(), dest.getX(), dest.getY(), dest.getTheta());
      } else {
        log("hypothetical fully empty map, calling BM");
        cost += GetPathLength(start, dest);
      }
    } else {
      log("first nbv");
    }
    log("total cost at %dth cone so far %3.2f", count, cost);
    start = dest;
    if (m_publishSimCones) {
      try {
        log("Adding View cone to WM for planner");
        SpatialData::ViewPointPtr vp = new SpatialData::ViewPoint;
        vp->pose.x = nbv.pos[0];
        vp->pose.y = nbv.pos[1];
        vp->pose.z = nbv.pan;
        vp->tilt = nbv.tilt;
        vp->label = targetObject;
        vp->probability = nbv.totalprob;

        //vp->closestPlaceId = GetPlaceIdFromNodeId(GetClosestNodeId(vp->pose.x, vp->pose.y, vp->pose.z)); 

        vp->closestPlaceId = FindClosestPlaceID(vp->pose.x, vp->pose.y);
        vp->areaId = GetAreaId(vp->pose.x, vp->pose.y, vp->pose.z);
        addToWorkingMemory(newDataID(), vp);

        PostViewCone(nbv);
      } catch (DoesNotExistOnWMException e) {
        log("WARNING: View cone could not be added: %s", e.what());
      }
    }
    count++;
    if (count > maxnumberofcones) {
      double averageprob = totalprob / count;
      double averagecost = cost / count;
      log("# of cones maxed out avprob: %3.2f avcost: %3.2f", averageprob,
          averagecost);
      double addcost = 0;
      for (double f = totalprob; f < threshold; f = f + averageprob) {
        addcost += averagecost;
      }
      log("addcost: %3.2f starting from totalprob: %3.2f", addcost, totalprob);
      cost += addcost;
      break;
    }

  }
  log("Returning cost: %3.2f with totalprob: %3.2f at %dth cone", cost,
      totalprob, count);
  return cost;
}

int VisualObjectSearch::FindClosestPlaceID(double x, double y) {
  // first get place positions
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  vector<pair<double, double> > place_positions;
  for (unsigned int i = 0; i < m_placestosearch.size(); i++) {
    //Get Node and get
    NavData::FNodePtr node = agg->getNodeFromPlaceID(m_placestosearch[i]);
    place_positions.push_back(make_pair(node->x, node->y));
  }
  double minlen = 100000;
  int closestplace = 0;
  for (unsigned int i = 0; i < place_positions.size(); i++) {
    double xp = place_positions[i].first;
    double yp = place_positions[i].second;
    double len = sqrt((x - xp) * (x - xp) + (y - yp) * (y - yp));
    if (len < minlen) {
      minlen = len;
      closestplace = i;
    }
  }
  log("closest placeid to this cone: %d", m_placestosearch[closestplace]);
  return m_placestosearch[closestplace];
}
void VisualObjectSearch::ChangeMaps(std::string roomid) {
  const char* id = &roomid[roomid.size() - 1];
  // set our maps for it
  m_map = m_maps[atoi(id)];
  m_lgm = m_lgms[atoi(id)];

  log("Changed current room id to: %d, pointing maps to this.", atoi(id));
  if (m_usePeekabot) {
    pbVis->Display2DCureMap(m_lgm);
    pbVis->DisplayMap(*m_map);
  }
}

double VisualObjectSearch::tryLoadStepCost(
    const std::vector<ObjectPairRelation> &step) {
  string filename("cached_cost_");
  filename += step.back().primaryobject;
  for (int i = step.size() - 1; i >= 0; i--) {
    //  for (std::vector<ObjectPairRelation>::const_iterator it = step.back(); 
    //	it != step.begin(); it--) {
    filename += (step[i].relation == FrontierInterface::ON ? "_ON_" : "_IN_");
    filename += step[i].secobject;
  }
  filename += ".cst";

  log("looking for cached file %s", filename.c_str());
  ifstream infile(filename.c_str(), ios::in);

  if (!infile.good()) {
    log("cost does not exists in cache calculating");
    return 0.0;
  }

  double ret;

  infile >> ret;

  log("read cost from file: %3.2f", ret);
  infile.close();
  return ret;
}

void VisualObjectSearch::cacheStepCost(
    const std::vector<ObjectPairRelation> &step, double cost, std::string end) {
  string filename("cached_cost_");
  filename += step.back().primaryobject;
  for (int i = step.size() - 1; i >= 0; i--) {
    //  for (std::vector<ObjectPairRelation>::const_iterator it = step.back(); 
    //	it != step.begin(); it--) {
    filename += (step[i].relation == FrontierInterface::ON ? "_ON_" : "_IN_");
    filename += step[i].secobject;
  }
  filename += end;
  filename += ".cst";
  log("caching step cost: %s", filename.c_str());
  ofstream outfile(filename.c_str(), ios::out);

  if (!outfile.good()) {
    log("Error! Could not write to file \"%s\"s", filename.c_str());
    return;
  }

  outfile << cost;
}

vector<VisualObjectSearch::ObjectPairRelation> VisualObjectSearch::getStrategyStep(
    vector<string> &policy, int step) {
  vector<ObjectPairRelation> strategyStep;

  std::vector<ObjectPairRelation> singleStrategy, prevStrategy;
  singleStrategy.clear();
  prevStrategy.clear();
  getStructuredStrategy(policy[step], singleStrategy);
  getStructuredStrategy(policy[step - 1], prevStrategy);
  unsigned int j = 0;
  for (j = 0; j < prevStrategy.size(); j++) {
    if (prevStrategy[j].relation != singleStrategy[j].relation
        || prevStrategy[j].primaryobject != singleStrategy[j].primaryobject
        || prevStrategy[j].secobject != singleStrategy[j].secobject)
      break;
  }
  for (; j < singleStrategy.size(); j++) {
    strategyStep.push_back(singleStrategy[j]);
  }
  return strategyStep;
}

double VisualObjectSearch::GetStrategyCost(std::list<std::string> policy) {
  std::vector<std::string> pol(policy.begin(), policy.end());
  return GetStrategyCost(pol);
}
double VisualObjectSearch::GetStrategyCost(std::vector<std::string> policy) {
  double StrategyCost = 0;
  std::string room = policy[0];
  log("got room: %s", room.c_str());

  // we always execute the first policy
  for (unsigned int i = 1; i < policy.size(); i++) {
    vector<ObjectPairRelation> strategyStep = getStrategyStep(policy, i);

    double costThisStep = tryLoadStepCost(strategyStep);
    if (costThisStep == 0) {
      log("novel # of relations: %d", strategyStep.size());
      ChangeMaps(room);

      // Set up query
      std::vector<std::string> labels;
      std::vector<FrontierInterface::ObjectRelation> relations;

      std::string topObject = strategyStep.back().primaryobject;

      SetCurrentTarget(topObject);

      labels.push_back(topObject);
      string logstring = "Asking for: ";
      logstring = logstring + labels.back();
      for (int i = strategyStep.size() - 1; i >= 0; i--) {
        labels.push_back(strategyStep[i].secobject);
        relations.push_back(strategyStep[i].relation);
        logstring = logstring + (strategyStep[i].relation
            == FrontierInterface::ON ? string(" ON ") : string(" IN "));
        logstring = logstring + strategyStep[i].secobject;
      }
      log(logstring.c_str());

      FrontierInterface::WeightedPointCloudPtr queryCloud =
          new FrontierInterface::WeightedPointCloud;
      FrontierInterface::ObjectPriorRequestPtr objreq =
          new FrontierInterface::ObjectPriorRequest;
      objreq->relationTypes = relations; // ON or IN or whatnot
      objreq->objects = labels; // Names of objects, starting with the query object
      objreq->cellSize = m_cellsize; // Cell size of map (affects spacing of samples)
      objreq->outCloud = queryCloud; // Data struct to receive output
      objreq->totalMass = 1.0;

      string baseObject = strategyStep.front().secobject;

      if (baseObject.find("room") != string::npos) {
        ChangeMaps(strategyStep.front().secobject);
        SpatialGridMap::GridMap<GridMapData> tmpMap = *m_map;

        // Search is uniform in the room (possibly direct informed)
        // Now, check if there's just one object left

        if (strategyStep.size() == 1) {
          // If so, do uniform search without asking for a point cloud
          log("strategy contains room and has 1 step: %3.2f", m_threshold);
          InitializePDFForObject(1.0, strategyStep[0].primaryobject, &tmpMap);
          if (m_usePeekabot) {
            pbVis->AddPDF(tmpMap);
          }
          double cost = 0.0;
          cost = GetCostForSingleStrategy(&tmpMap, topObject, m_threshold,
              false);
          costThisStep = cost;
          if (!m_publishSimCones)
            cacheStepCost(strategyStep, costThisStep);
        } else {
          log("strategy contains room and has more than 1 step");
          // There's more than one relation. 
          // Strip off the "in room" relation from the policy for purposes
          // of relation query
          strategyStep.erase(strategyStep.begin());
          // Get the uninformed sample cloud from the ORM

          // remove the room relation from the request 
          objreq->objects.erase(objreq->objects.end() - 1);
          objreq->relationTypes.erase(objreq->relationTypes.end() - 1);
          // Send request and wait for reply
          m_bEvaluation = true;
          {
            unlockComponent();
            addToWorkingMemory(newDataID(), objreq);
            gotPC = false;
            while (!gotPC)
              usleep(2500);
            log("got PC for direct search");
          }

          lockComponent();
          FrontierInterface::WeightedPointCloudPtr cloud = m_priorreq->outCloud;
          log("got point cloud with height %f", cloud->center.z);
          vector<pair<int, int> > obstacleCells;
          const unsigned int nTargetKernelSamples = 100;
          const int step1 = 20;
          const int step2 = 1;

          // Form a diagonal striped pattern of samples across the map
          // This is a dirty trick, as it assumes the spatial frequency is low along the diagonal
          while (obstacleCells.size() < nTargetKernelSamples) {
            int offset1 = rand() % step1;
            for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
              for (int y = -m_lgm->getSize() + (offset1
                  + (x + m_lgm->getSize()) * step2) % step1; y
                  <= m_lgm->getSize(); y += step1) {
                int bloxelX = x + m_lgm->getSize();
                int bloxelY = y + m_lgm->getSize();
                for (int x2 = x - 1; x2 <= x + 1; x2++) {
                  if (x2 >= -m_lgm->getSize() && x2 <= m_lgm->getSize()) {
                    for (int y2 = y - 1; y2 <= y + 1; y2++) {
                      if (y2 >= -m_lgm->getSize() && y2 <= m_lgm->getSize()) {
                        if ((*m_lgm)(x2, y2) == '1' || (*m_lgm)(x2, y2) == '3') {
                          obstacleCells.push_back(make_pair<int> (bloxelX,
                              bloxelY));
                        }
                      }
                    }
                  }
                }
              }
            }
          }

          vector<Vector3> centers;
          for (unsigned int i = 0; i < obstacleCells.size(); i++) {
            // place a KDE cloud around it, with the center given
            // by the relation manager (which includes a z-coordinate
            // for tables, desks etc)
            int bloxelX = obstacleCells[i].first;
            int bloxelY = obstacleCells[i].second;
            pair<double, double> kernelCoords = tmpMap.gridToWorldCoords(
                bloxelX, bloxelY);
            cloud->center.x = kernelCoords.first;
            cloud->center.y = kernelCoords.second;
            centers.push_back(cloud->center);
          }

          m_sampler.kernelDensityEstimation3D(tmpMap, centers, cloud->interval,
              cloud->xExtent, cloud->yExtent, cloud->zExtent, cloud->values,
              1.0 / (m_lgm->getSize()), //Just an ad-hoc guess
              1.0, m_lgm);
          normalizePDF(tmpMap, 1.0);
          if (m_usePeekabot)
            pbVis->AddPDF(tmpMap);

          double cost = 0.0;
          //FIXME: get lgmpdf as an arg
          cost = GetCostForSingleStrategy(&tmpMap, topObject, m_threshold,
              false);
          costThisStep = cost;
          cacheStepCost(strategyStep, costThisStep, "_IN_" + policy[0]);
        }

      } else {
        log("strategy does not contain room");
        GridMapData def;
        def.occupancy = FREE;

        // This step doesn't begin with the room; this means it's
        // an indirect search. 

        // Assume a number of poses for the base object, and
        // create and query a sample cloud for each
        // KDE and compute cost for each, then average the costs

        // Select hypothesical poses for base object
        vector<Pose3> baseObjectPoses;
        Pose3 tmpPose;
        tmpPose.pos = vector3(0, 0, 0);
        ;

        // Randomize orientations for involved objects, unless we already have such
        vector<Matrix33> orientations;
        if (//FIXME
        baseObject == "table1" || baseObject == "table2" || baseObject
            == "bookcase_sm" || baseObject == "bookcase_lg" || baseObject
            == "shelves" || baseObject == "desk") {
          getRandomSampleCircle(orientations, 1);
        } else {
          vector<Matrix33> orientations;
          getRandomSampleSphere(orientations, 1);
        }
        for (vector<Matrix33>::iterator it = orientations.begin(); it
            != orientations.end(); it++) {
          tmpPose.rot = *it;
          baseObjectPoses.push_back(tmpPose);
        }

        for (vector<Pose3>::iterator it = baseObjectPoses.begin(); it
            != baseObjectPoses.end(); it++) {

          log(
              "looping over hypothetical case (once again?) for hypothetical case");
          // Fill in the hypothetical pose in question
          objreq->baseObjectPose.clear();
          objreq->baseObjectPose.push_back(*it);

          // Send request and wait for reply
          m_bEvaluation = true;
          {
            unlockComponent();
            addToWorkingMemory(newDataID(), objreq);
            gotPC = false;
            while (!gotPC)
              usleep(2500);
            log("got PC for a hypothetical pose");
          }
          lockComponent();
          FrontierInterface::WeightedPointCloudPtr cloud = m_priorreq->outCloud;

          double interval = cloud->interval;
          int xExtent = cloud->xExtent;
          int yExtent = cloud->yExtent;

          //	  log("Cloud: (%d x %d) at interval %f", xExtent, yExtent, interval);

          double tmp = interval * (xExtent > yExtent ? xExtent : yExtent);
          tmp += m_conedepth;
          tmp /= m_cellsize;
          int lgmSize = (int) tmp;
          if (lgmSize > m_gridsize)
            lgmSize = m_gridsize;
          log("lgmSize = %i", lgmSize);

          SpatialGridMap::GridMap<GridMapData> tmpMap(lgmSize * 2 + 1, lgmSize
              * 2 + 1, m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
          GDProbSet resetter(0.0);
          tmpMap.universalQuery(resetter, true);

          CureObstMap
              tempLGM(lgmSize, m_cellsize, '0', CureObstMap::MAP1, 0, 0);
          CureObstMap *backupRoomLGM = m_lgm;
          m_lgm = &tempLGM;

          // Do KDE for this object
          vector<Vector3> centers;
          centers.push_back(cloud->center);
          m_sampler.kernelDensityEstimation3D(tmpMap, centers, cloud->interval,
              cloud->xExtent, cloud->yExtent, cloud->zExtent, cloud->values,
              1.0, 1.0, m_lgm);
          normalizePDF(tmpMap, 1.0);
          //pbVis->AddPDF(tmpMap);
          // Compute cost for this map and object

          // For the purposes of cone generation/evaluation, 
          // set m_lgm temporarily to an empty grid
          // centered at the origin
          // Set its extent to be a bounding box based on
          // the spread of the point cloud and the maximum cone length
          // FIXME get lgmpdf as arg
          double cost = 0.0;
          cost
              = GetCostForSingleStrategy(&tmpMap, topObject, m_threshold, true);
          costThisStep += cost / baseObjectPoses.size();
          m_lgm = backupRoomLGM;
        }
        cacheStepCost(strategyStep, costThisStep);

      }
    }
    StrategyCost += costThisStep;
  }
  return StrategyCost;
}

void VisualObjectSearch::newSpatialObject(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    SpatialData::SpatialObjectPtr newObj = getMemoryEntry<
        SpatialData::SpatialObject> (objID.address);

    //	if(m_publishSimCones)
    //			return;

    spatial::Object *model = generateNewObjectModel(newObj->label);
    log("Got Spatial Object: %s", newObj->label.c_str());
    m_recognizedobjects.push_back(newObj->label.c_str());
    char buffer[256];
    sprintf(buffer, "Object %s detected.", newObj->label.c_str());
    writeStringToFile((string) buffer);
    model->pose = newObj->pose;

    putObjectInMap(*m_map, model);
    if (m_usePeekabot)
      pbVis->DisplayMap(*m_map);
    m_map->clearDirty();

    // just place it in the map and move on
    if (m_publishSimCones)
      return;

    delete model;
    if (newObj->label == targetObject) {
      log("target object detected, stopping search");
      char buffer[1024];
      sprintf(
          buffer,
          "Success, target Object %s detected. Policy Step completed: %s, total # of VCones for this step: %d",
          targetObject.c_str(), currentSearchPolicy[currentPolicyStep].c_str(),
          m_totalViewPoints);
      writeStringToFile((string) buffer);

      DetectionComplete(true, newObj->label);
      return;
    }

    if (!waitingForDetection.empty() || !waitingForObjects.empty()) {

      if (newObj->label == currentTarget) {
        log("Current Target detected.");
        waitingForDetection.clear();
        waitingForObjects.clear();
        DetectionComplete(true, newObj->label);
      } else {
        waitingForObjects.erase(newObj->label);
        waitingForDetection.erase(newObj->label);

        string logString(" Spatial Object Waiting list: ");
        for (std::set<string>::iterator it = waitingForObjects.begin(); it
            != waitingForObjects.end(); it++) {
          logString += *it + " ";
        }
        logString += " / ";
        for (std::set<string>::iterator it = waitingForDetection.begin(); it
            != waitingForDetection.end(); it++) {
          logString += *it + " ";
        }
        log(logString.c_str());

        if (waitingForDetection.empty() && waitingForObjects.empty()) {
          DetectionComplete(false);
        }
      }
    }
  } catch (DoesNotExistOnWMException e) {
    log("Error! SpatialObject disappeared from WM!");
  }
}

void VisualObjectSearch::putObjectInMap(GridMap<GridMapData> &map,
    spatial::Object *object) {
  cogx::Math::Pose3 &pose = object->pose;
  switch (object->type) {
  case OBJECT_BOX: {
    BoxObject &box = *(BoxObject*) object;
    double radius1, radius2, radius3;
    //Flatten pose to xy-orientation

    double maxAxis = pose.rot.m20;
    Vector3 fakeXDirection = vector3(pose.rot.m01, pose.rot.m11, pose.rot.m21);
    radius1 = box.radius2;
    radius2 = box.radius3;
    radius3 = box.radius1;

    if (-pose.rot.m20 > maxAxis) {
      maxAxis = -pose.rot.m20;
      fakeXDirection = vector3(pose.rot.m01, pose.rot.m11, pose.rot.m21);
      radius1 = box.radius2;
      radius2 = box.radius3;
      radius3 = box.radius1;
    }
    if (pose.rot.m21 > maxAxis) {
      maxAxis = pose.rot.m21;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      radius1 = box.radius1;
      radius2 = box.radius3;
      radius3 = box.radius2;
    }
    if (-pose.rot.m21 > maxAxis) {
      maxAxis = -pose.rot.m21;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      radius1 = box.radius1;
      radius2 = box.radius3;
      radius3 = box.radius2;
    }
    if (pose.rot.m22 > maxAxis) {
      maxAxis = pose.rot.m22;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      radius1 = box.radius1;
      radius2 = box.radius2;
      radius3 = box.radius3;
    }
    if (-pose.rot.m22 > maxAxis) {
      maxAxis = -pose.rot.m22;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      radius1 = box.radius1;
      radius2 = box.radius2;
      radius3 = box.radius3;
    }

    double direction = atan2(fakeXDirection.y, fakeXDirection.x);
    if (direction < 0)
      direction += 2 * M_PI;
    GDMakeObstacle makeObstacle;
    map.boxModifier(box.pose.pos.x, box.pose.pos.y, box.pose.pos.z,
        2 * radius1, 2 * radius2, 2 * radius3, direction, makeObstacle);
  }
    break;
  case OBJECT_HOLLOW_BOX: {
    log("drawing hollow box");
    HollowBoxObject &box = *(HollowBoxObject*) object;
    double radius1, radius2, radius3;
    //Flatten pose to xy-orientation

    double maxAxis = pose.rot.m20;
    Vector3 fakeXDirection = vector3(pose.rot.m01, pose.rot.m11, pose.rot.m21);
    int openSide = 0;
    radius1 = box.radius2;
    radius2 = box.radius3;
    radius3 = box.radius1;

    if (-pose.rot.m20 > maxAxis) {
      maxAxis = -pose.rot.m20;
      fakeXDirection = vector3(pose.rot.m01, pose.rot.m11, pose.rot.m21);
      openSide = 5;
      radius1 = box.radius2;
      radius2 = box.radius3;
      radius3 = box.radius1;
    }
    if (pose.rot.m21 > maxAxis) {
      maxAxis = pose.rot.m21;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      openSide = 1;
      radius1 = box.radius1;
      radius2 = box.radius3;
      radius3 = box.radius2;
    }
    if (-pose.rot.m21 > maxAxis) {
      maxAxis = -pose.rot.m21;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      openSide = 1;
      radius1 = box.radius1;
      radius2 = box.radius3;
      radius3 = box.radius2;
    }
    if (pose.rot.m22 > maxAxis) {
      maxAxis = pose.rot.m22;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      openSide = 1;
      radius1 = box.radius1;
      radius2 = box.radius2;
      radius3 = box.radius3;
    }
    if (-pose.rot.m22 > maxAxis) {
      maxAxis = -pose.rot.m22;
      fakeXDirection = vector3(pose.rot.m00, pose.rot.m10, pose.rot.m20);
      openSide = 1;
      radius1 = box.radius1;
      radius2 = box.radius2;
      radius3 = box.radius3;
    }

    double direction = atan2(fakeXDirection.y, fakeXDirection.x);
    if (direction < 0)
      direction += 2 * M_PI;
    GDMakeObstacle makeObstacle;
    double h = box.thickness / 2;
    double cd = cos(direction);
    double sd = sin(direction);

    if (openSide != 0)
      map.boxModifier(box.pose.pos.x, box.pose.pos.y, box.pose.pos.z + radius3
          - h, radius1 * 2, radius2 * 2, 2 * h, direction, makeObstacle);

    if (openSide != 1)
      map.boxModifier(box.pose.pos.x + cd * (radius1 - h), box.pose.pos.y + sd
          * (radius1 - h), box.pose.pos.z, 2 * h, radius2 * 2, radius3 * 2,
          direction, makeObstacle);

    map.boxModifier(box.pose.pos.x - cd * (radius1 - h), box.pose.pos.y - sd
        * (radius1 - h), box.pose.pos.z, 2 * h, radius2 * 2, radius3 * 2,
        direction, makeObstacle);

    map.boxModifier(box.pose.pos.x - sd * (radius2 - h), box.pose.pos.y + cd
        * (radius2 - h), box.pose.pos.z, radius1 * 2, 2 * h, radius3 * 2,
        direction, makeObstacle);

    map.boxModifier(box.pose.pos.x + sd * (radius2 - h), box.pose.pos.y - cd
        * (radius2 - h), box.pose.pos.z, radius1 * 2, 2 * h, radius3 * 2,
        direction, makeObstacle);

    if (openSide != 5)
      map.boxModifier(box.pose.pos.x, box.pose.pos.y, box.pose.pos.z - radius3
          + h, radius1 * 2, radius2 * 2, 2 * h, direction, makeObstacle);
  }
    break;
  default:
    log("Error! Unsupported object type in puObjectInMap!");
    return;
  }
  log("returning");
}

void VisualObjectSearch::SaveCureMapToFile() {
  log("Writing cure map");

  ofstream fout("curemap.txt");
  for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
    for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
      fout << (*m_lgm)(x, y);
    }
    //fout << endl;
  }
  fout.close();
}

void VisualObjectSearch::InitializePDF(double initprob) {
  GDProbInit initfunctor(initprob / (m_map->getZBounds().second
      - m_map->getZBounds().first));

  for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
    for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
      int bloxelX = x + m_lgm->getSize();
      int bloxelY = y + m_lgm->getSize();
      if ((*m_lgm)(x, y) == '3' || (*m_lgm)(x, y) == '1') {
        // For each "high" obstacle cell, assign a uniform probability density to 
        // its immediate neighbors
        // (Only neighbors which are not unknown in the Cure map. Unknown 3D
        // space is still assigned, though)
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if ((*m_lgm)(x + i, y + j) == '0' && (bloxelX + i <= m_gridsize
                && bloxelX + i > 0) && (bloxelY + i <= m_gridsize && bloxelY
                + i > 0))
              m_map->boxSubColumnModifier(bloxelX + i, bloxelY + j,
                  m_mapceiling / 2, m_mapceiling, initfunctor);
          }
        }
      }
    }
  }
  double massAfterInit = initfunctor.getTotal();
  //    double normalizeTo = (initfunctor.getTotal()*m_pout)/(1 - m_pout);
  normalizePDF(*m_map, initprob, massAfterInit);
}

void VisualObjectSearch::InitializePDFForObject(double initprob,
    const string &label,
    SpatialGridMap::GridMap<SpatialGridMap::GridMapData>* map) {

  if (map == 0)
    map = m_map;
  double height;
  bool isspecial = false;
  if (label == "table1") {
    height = 0.45 - 0.225;
    isspecial = true;
  } else if (label == "table2" || label == "table") {
    isspecial = true;
    height = 0.72 - 0.36;
  } else if (label == "bookcase_sm") {
    isspecial = true;
    height = 0.75 + 0.08;
  } else if (label == "bookcase_lg") {
    isspecial = true;
    height = 0.965 + 0.08;
  } else if (label == "shelves") {
    isspecial = true;
    height = 1.075 + 0.00;
  } else if (label == "desk") {
    isspecial = true;
    height = 0.75 - 0.05;
  } else {
    log("initializing PDF for object %s, as a uniform distribution.",
        label.c_str());
  }

  GDProbSet resetter(0.0);
  map->universalQuery(resetter, true);

  GDProbInit initfunctor(initprob / (map->getZBounds().second
      - map->getZBounds().first));

  for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
    for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
      int bloxelX = x + m_lgm->getSize();
      int bloxelY = y + m_lgm->getSize();
      if ((*m_lgm)(x, y) == '3' || (*m_lgm)(x, y) == '1') {
        // For each "high" obstacle cell, assign a uniform probability density to its immediate neighbors
        // (Only neighbors which are not unknown in the Cure map. Unknown 3D
        // space is still assigned, though)

        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if ((*m_lgm)(x + i, y + j) == '0' && (bloxelX + i
                <= map->getMapSize().first && bloxelX + i > 0) && (bloxelY + i
                <= map->getMapSize().second && bloxelY + i > 0)) {
              if (isspecial)
                map->boxSubColumnModifier(bloxelX + i, bloxelY + j, height, 2
                    * m_minbloxel, initfunctor);
              else
                map->boxSubColumnModifier(bloxelX + i, bloxelY + j,
                    m_mapceiling / 2, m_mapceiling, initfunctor);
            }
          }
        }
      }
    }
  }
  double massAfterInit = initfunctor.getTotal();
  //    double normalizeTo = (initfunctor.getTotal()*m_pout)/(1 - m_pout);
  normalizePDF(*map, initprob, massAfterInit);
}

void VisualObjectSearch::savemap(GtkWidget *widget, gpointer data) {
  AVSComponentPtr->addProcessViewPointCommand();
  //AVSComponentPtr->MovePanTilt(0, -10*M_PI/180,0.08);
  //	while(true){
  //	  AVSComponentPtr->addARTagCommand("metalbox");
  //	  AVSComponentPtr->addARTagCommand("table1");
  //	  AVSComponentPtr->addARTagCommand("table2");
  //	  AVSComponentPtr->addARTagCommand("shelves");
  //	  sleep(1);
  //	}
}
void VisualObjectSearch::readmap(GtkWidget *widget, gpointer data) {
  AVSComponentPtr->isRunComponent = true;
}
void VisualObjectSearch::selectdu(GtkWidget *widget, gpointer data) {
  AVSComponentPtr->writeStringToFile("Search started.");
  AVSComponentPtr->LookforObjectWithStrategy();
}
void VisualObjectSearch::selectdi(GtkWidget *widget, gpointer data) {
  AVSComponentPtr->m_totalprob = 0.93;
  // AVSComponentPtr->targetObject = "book";
  //  AVSComponentPtr->LookforObjectWithStrategy();
}
void VisualObjectSearch::selectind(GtkWidget *widget, gpointer data) {
  AVSComponentPtr->addViewPointGenerationCommand();
}

void VisualObjectSearch::setRot(double p11, double p12, double p13, double p21,
    double p22, double p23, double p31, double p32, double p33, Pose3 &p) {

  p.rot.m00 = p11;
  p.rot.m01 = p12;
  p.rot.m02 = p13;

  p.rot.m10 = p21;
  p.rot.m11 = p22;
  p.rot.m12 = p23;

  p.rot.m20 = p31;
  p.rot.m21 = p32;
  p.rot.m22 = p33;
}

void VisualObjectSearch::addProcessViewPointCommand() {
  SpatialData::ProcessViewPointCommandPtr cmd =
      new SpatialData::ProcessViewPointCommand;
  SpatialData::ViewPointPtr vp = new SpatialData::ViewPoint;
  cmd->vp = vp;
  cmd->vp->pose.x = 0;
  cmd->vp->pose.y = 0.5;
  cmd->vp->pose.z = 45 * M_PI / 180;
  cmd->vp->tilt = -25 * M_PI / 180;
  cmd->objectModels.push_back("bookcase_sm");
  cmd->objectModels.push_back("book");
  addToWorkingMemory(newDataID(), cmd);
}
void VisualObjectSearch::addViewPointGenerationCommand() {
  SpatialData::ViewPointGenerationCommandPtr cmd =
      new SpatialData::ViewPointGenerationCommand;
  cmd->label = "book";
  cmd->placestosearch.push_back(1);
  addToWorkingMemory(newDataID(), cmd);
}
void VisualObjectSearch::writeStringToFile(std::string str) {
  time_t rawtime;
  struct tm * timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  std::string timenow(asctime(timeinfo));
  ofstream out("SearchLog.txt", ios::app);
  out << "> " << str << " -- " << timenow;
  out.close();
  //   out << currentSearchMode << " took " << m_totalViewPoints<< " view points" << ", with result:  " << result << " m_pout: " << m_pout << " " << " time: " << buffer << endl;
  //   out.close();
}

void VisualObjectSearch::GenerateViewPoints() {
  // We are being asked to search 
  InitializeMaps( m_placestosearch);
  m_map = m_maps[1];
  m_lgm = m_lgms[1];
  int tableid = GetPlaceIdFromNodeId(GetClosestNodeId(tablepose.getX(),
      tablepose.getY(), 0));

  //if this id exists in the list
  bool exists = false;
  for (unsigned int i = 0; i < m_placestosearch.size(); i++) {
    if (m_placestosearch[i] == tableid) {
      exists = true;
      break;
    }
  }
  SpatialGridMap::GridMap<GridMapData> tmpMap = *m_map;

  FrontierInterface::ObjectPriorRequestPtr objreq =
      new FrontierInterface::ObjectPriorRequest;
  if (exists && !m_nobaseobject) {
    log("table exists in the places  what we are asked to search");
    // we always execute the first policy
    string filename = "table+with+object.txt";
    ifstream tableCloudFile(filename.c_str(), ios::in);
    if (tableCloudFile.is_open()) {

      log("Reading cloud from file");
      FrontierInterface::WeightedPointCloudPtr cloud =
          new FrontierInterface::WeightedPointCloud;
      tableCloudFile >> cloud->center.x;
      tableCloudFile >> cloud->center.y;
      tableCloudFile >> cloud->center.z;
      tableCloudFile >> cloud->interval;
      tableCloudFile >> cloud->xExtent;
      tableCloudFile >> cloud->yExtent;
      tableCloudFile >> cloud->zExtent;

      long int valn;
      tableCloudFile >> valn;

      log("xExtend: %d , yExtend: %d, zExtend: %d", cloud->xExtent,
          cloud->yExtent, cloud->zExtent);
      cloud->values.resize(valn);

      for (unsigned long i = 0; i < valn; i++) {
        tableCloudFile >> cloud->values[i];
      }

      int tmp;
      tableCloudFile >> tmp;
      cloud->isBaseObjectKnown = tmp;

      double totalMass;
      tableCloudFile >> totalMass;
      log("totalMass read from file: %3.2f", totalMass);
      receivePointCloud(cloud, totalMass);
    } else {
      FrontierInterface::WeightedPointCloudPtr queryCloud =
          new FrontierInterface::WeightedPointCloud;
      vector<FrontierInterface::ObjectRelation> relations;
      vector<string> labels;
      relations.push_back(FrontierInterface::ON);
      labels.push_back(targetObject);
      labels.push_back(m_tablelabel);
      objreq->relationTypes = relations; // ON or IN or whatnot
      objreq->objects = labels; // Names of objects, starting with the query object
      objreq->cellSize = m_cellsize; // Cell size of map (affects spacing of samples)
      objreq->outCloud = queryCloud; // Data struct to receive output
      objreq->totalMass = 1.0;

      GDProbScale reset(0.0);
      m_map->universalQuery(reset, true);
      unlockComponent();
      addToWorkingMemory(newDataID(), objreq);
      gotPC = false;
      while (!gotPC)
        usleep(2500);
      log("got PC for direct search");
      lockComponent();
    }
    GetCostForSingleStrategy(m_map, targetObject, m_threshold, false);
    // overwrite the command
  } else {
    log("table does not exists in the places  what we are asked to search");
    InitializePDFForObject(1.0, targetObject, m_map);
    if (m_usePeekabot) {
      pbVis->AddPDF(*m_map);
    }
    GetCostForSingleStrategy(m_map, targetObject, m_threshold, false);
  }
  SpatialData::ViewPointGenerationCommandPtr newVPCommand =
      new SpatialData::ViewPointGenerationCommand;
  newVPCommand->status = SpatialData::SUCCESS;
  overwriteWorkingMemory<SpatialData::ViewPointGenerationCommand> (
      m_viewpointgen_id, newVPCommand);
  /*	list<string> listpolicy;
   AVSPolicyManager policyManager;	
   policyManager.m_CurrentRoom=0;
   policyManager.m_evalpol = this; 
   policyManager.m_CostCredit = 9;
   policyManager.m_ExpectedCost = 30;
   policyManager.m_Type = 2;
   string filename = "configs_"+ targetObject + ".txt";	
   ifstream ifile(filename.c_str());
   if (!ifile) {
   // The file exists, and is open for input

   ofstream out(filename.c_str());
   out << targetObject << endl;
   out << 0.8 << " " << targetObject << " ON " << "table1 IN room1" << endl;
   out << 0.95 << " " << "table1" << " IN " << "room1" << endl;	
   out << 0.95 << " " <<  targetObject << " IN " << "room1";	
   out.close();
   }
   policyManager.m_ConfigurationsFilename = filename;

   policyManager.preCompute(targetObject);
   // overwrite the command
   SpatialData::ViewPointGenerationCommandPtr newVPCommand = new SpatialData::ViewPointGenerationCommand;
   newVPCommand->status = SpatialData::SUCCESS;
   overwriteWorkingMemory<SpatialData::ViewPointGenerationCommand>(m_viewpointgen_id , newVPCommand);*/
}
void VisualObjectSearch::runComponent() {
  m_command = IDLE;
  m_ProcessVPID = "";

  //				if (m_posttable){
  //post a fake table object
  //	}


  log("I am running");

  sleep(10);
  log("Posting a fake table.");
  VisionData::Post3DObjectPtr obj3D = new VisionData::Post3DObject;
  obj3D->isDetected = true;
  obj3D->label = m_tablelabel;
  Pose3 p;
  p.pos.x = tablepose.getX();
  p.pos.y = tablepose.getY();
  p.pos.z = 0.26;
  setRot(cos(tablepose.getTheta()), -sin(tablepose.getTheta()), 0, sin(
      tablepose.getTheta()), cos(tablepose.getTheta()), 0, 0, 0, 1, p);
  obj3D->pose = p;
  addToWorkingMemory(newDataID(), obj3D);
  sleep(1);

  if (m_showgui) {

    log("showing GUI");

    int argc = 0;
    char** argv = NULL;

    gtk_init(&argc, &argv);

    // add a window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "map controls");

    //window cannot hold more than 1 button, thus ad a box
    hbox = gtk_hbox_new(FALSE, 5);
    savebutton = gtk_button_new_with_label("Ask to Execute VC");
    readbutton = gtk_button_new_with_label("Load Map");
    direct_uninformed = gtk_button_new_with_label("Start");
    direct_informed = gtk_button_new_with_label("Fail Policy");
    indirect = gtk_button_new_with_label("Ask for View Cones");

    //parent: window, child: hbox
    gtk_container_add(GTK_CONTAINER(window), hbox);
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);
    gtk_widget_show ( hbox);

    //callbacks

    g_signal_connect(G_OBJECT(savebutton), "clicked", G_CALLBACK(
        &spatial::VisualObjectSearch::savemap), NULL);
    g_signal_connect(G_OBJECT(readbutton), "clicked", G_CALLBACK(
        &spatial::VisualObjectSearch::readmap), NULL);

    g_signal_connect(G_OBJECT(direct_uninformed), "clicked", G_CALLBACK(
        &spatial::VisualObjectSearch::selectdu), NULL);
    g_signal_connect(G_OBJECT(direct_informed), "clicked", G_CALLBACK(
        &spatial::VisualObjectSearch::selectdi), NULL);
    g_signal_connect(G_OBJECT(indirect), "clicked", G_CALLBACK(
        &spatial::VisualObjectSearch::selectind), NULL);

    //add buttons to box
    gtk_box_pack_start(GTK_BOX(hbox), readbutton, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(hbox), direct_uninformed, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), direct_informed, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), indirect, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), savebutton, TRUE, TRUE, 0);

    gtk_widget_show ( direct_uninformed);
    gtk_widget_show ( direct_informed);
    gtk_widget_show ( readbutton);
    gtk_widget_show ( indirect);
    gtk_widget_show ( savebutton);
    gtk_widget_show ( window);
  }

  //    ReadCureMapFromFile();

  viewCount = 0; //Init viewcount

  while (isRunning()) {
    lockComponent();
    if (m_showgui) {
      while (gtk_events_pending()) {
        gtk_main_iteration();
      }
    }
    if (m_generateviewcones) {
      log("Generate ViewCone request.");
      GenerateViewPoints();
      m_generateviewcones = false;
    }

    /* if(isRunComponent){
     lockComponent();
     //      pbVis->DisplayMap(*m_map);
     pbVis->Display2DCureMap(m_lgm,"current_m_lgm");
     if(m_lgms.size() == 0 ){
     //InitializeMaps();
     m_policyManager.preCompute("book");
     }
     InterpretCommand();
     unlockComponent();
     }*/

    unlockComponent();
    sleep(1);
  }
}

VisualObjectSearch::ObjectPairRelation VisualObjectSearch::GetSecondaryObject(
    string name) {
  ObjectPairRelation obj;
  obj.primaryobject = "";
  //NOTE: WE ASSUME THAT EACH OBJECT HAS EXACTLY ONE SPATIAL RELATION WITH ANOTHER OBJECT!
  // I.E. NO SUCH: RICE ON TABLE, RICE IN ROVIO AT THE SAME TIME
  for (unsigned int i = 0; i < objectData.size(); i++) {
    if (objectData[i].object == name && objectData[i].relations.size() != 0) {
      return objectData[i].relations[0];
    }
  }
  return obj;
}

std::string VisualObjectSearch::printPolicy(vector<string> policy) {
  string pol = "";
  for (vector<string>::iterator it = policy.begin(); it != policy.end(); it++) {
    pol += *it;
    pol += "->";
    cout << it->c_str() << "->";
  }
  cout << endl;
  return pol;
}
void VisualObjectSearch::InterpretCommand() {
  switch (m_command) {
  case STOP: {
    log("Stopped.");
    m_command = IDLE;

    if (m_bSimulation)
      break;

    log("Command: STOP");
    Cure::Pose3D pos;
    PostNavCommand(pos, SpatialData::STOP);
    break;
  }
  case EVALUATE_POLICIES: {
    log("Command: EVALUATE_POLICIES");
    // target object is given hence we hard code here
    targetObject = "book";
    vector<vector<string> > policies;
    list<string> listpolicy;
    vector<string> policy;
    m_policyManager.m_CostCredit = 9;
    m_policyManager.m_ExpectedCost = 30;
    m_policyManager.m_Type = 2;
    char buffer[1024];
    sprintf(
        buffer,
        "Selecting best policy with parameters, failedPolicyStep: %s, currentRoom: %ld, CostCredit: %f, ExpectedCost: %f, m_Type: %d",
        failedPolicyStep.c_str(), m_policyManager.m_CurrentRoom,
        m_policyManager.m_CostCredit, m_policyManager.m_ExpectedCost,
        m_policyManager.m_Type);
    writeStringToFile((string) buffer);
    if (m_policyManager.nextNearlyBestPolicy(listpolicy, failedPolicyStep, 0.1,
        0.9)) {
      log("AAAAA best policy found");
      vector<string> tmp(listpolicy.begin(), listpolicy.end());
      policy = tmp;
      writeStringToFile("Best policy " + printPolicy(policy));
    } else {
      m_command = STOP;
      log("No other best policy found. Stopping");
      writeStringToFile("No other best policy found. Stopping and exiting");
      exit(0);
    }
    //policy.push_back("room1");
    //policy.push_back("bookcase_sm IN room1");
    //policy.push_back("dell ON bookcase_sm IN room1");
    policies.push_back(policy);
    currentSearchPolicy = policies[0];
    currentPolicyStep = 0;
    const char* id = &policy[0][policy[0].size() - 1];
    m_policyRoom = atoi(id);

    m_command = GOTOROOM;
    break;
  }
  case GOTOROOM: {
    log("command: GOTOROOM");

    if (m_policyRoom != m_currentRoom) {
      //TODO: Check which room we are
      SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand;

      // hard coded room nodes
      cmd->destId.resize(1);
      if (m_currentRoom == 1) {
        m_policyManager.m_CurrentRoom = 0;
        cmd->destId[0] = 1;
      } else {
        m_policyManager.m_CurrentRoom = 1;
        cmd->destId[0] = 17;
      }
      cmd->cmd = SpatialData::GOTOPLACE;
      cmd->status = SpatialData::NONE;
      cmd->comp = SpatialData::COMMANDPENDING;
      cmd->prio = SpatialData::NORMAL;
      addToWorkingMemory(newDataID(), cmd);
      m_command = IDLE;
    } else {
      char buffer[256];
      sprintf(buffer, "We are already at room %d", m_currentRoom);
      writeStringToFile((string) buffer);
      log("Already at room %d", m_currentRoom);
      ChangeMaps( currentSearchPolicy[currentPolicyStep]);
      currentPolicyStep++;
      m_command = ASK_FOR_DISTRIBUTION;
    }
    break;
  }
  case ASK_FOR_DISTRIBUTION: {

    if (m_usePeekabot) {
      pbVis->DisplayMap(*m_map);
      pbVis->Display2DCureMap(m_lgm);
    }
    log("Command: ASK_FOR_DIST");
    m_command = IDLE;
    AskForDistribution();
    break;
  }
  case NEXT_NBV: {
    if (m_bSimulation) {
      if (m_pout > 0.7 || m_totalViewPoints > 2) {

        m_totalViewPoints = 0;
        m_pout = 0.3;
        isSearchFinished = true;
        m_command = STOP;
        log("Search stopped m_pout: %f", m_pout);
        SaveSearchPerformance("NOT FOUND");
        break;
      }
      m_command = RECOGNIZE;

      PopulateLGMap();
      m_nbv = SampleAndSelect();
      PostViewCone( m_nbv);
    }

    else {
      if (m_totalprob > 0.9 || m_totalViewPoints > maxnumberofcones) {
        log("policy %s failed, asking for next one",
            currentSearchPolicy[currentPolicyStep].c_str());
        failedPolicyStep = currentSearchPolicy[currentPolicyStep];
        if (m_totalprob > 0.7) {
          char buffer[1024];
          sprintf(
              buffer,
              "Policy Step failed, cause: max total covered prob. reached: %f, policy was: %s , total view cones in this step: %d",
              m_totalprob, failedPolicyStep.c_str(), m_totalViewPoints);
          writeStringToFile((string) buffer);
        } else if (m_totalViewPoints > maxnumberofcones) {
          char buffer[1024];
          sprintf(
              buffer,
              "Policy Step failed, cause: maxed out # of VCones, policy step was: %s , total view cones in this step: %d",
              failedPolicyStep.c_str(), m_totalViewPoints);
          writeStringToFile((string) buffer);
        }

        m_totalprob = 0;
        m_totalViewPoints = 0;
        m_pout = 0.3;
        m_command = EVALUATE_POLICIES;
        break;
      }
      log("Command: NEXT_NBV");
      m_command = WAITING; //Wait to get in position
      m_waitingCount = 0;

      PopulateLGMap();
      m_nbv = SampleAndSelect();
      PostViewCone( m_nbv);

      GoToNBV();

    }
    break;
  }
  case RECOGNIZE: {
    log("Command: Recognize");
    log("At %dth viewpoint", m_totalViewPoints);
    if (m_bSimulation) {
      m_command = NEXT_NBV; //Default, but may change in DetectionComplete
      // TODO more code logic could go here
      DetectionComplete(false);
      viewCount++;
    } else {
      m_command = WAITING; //Wait for recognition
      m_waitingCount = 0;
      Recognize();
    }
    break;
  }
  case WAITING: {
    log("Waiting... %i", m_waitingCount);
    m_waitingCount++;
    if (m_waitingCount == 50) {
      for (unsigned int i = 0; i < m_objectlist.size(); i++)
        addRecognizer3DCommand(VisionData::RECSTOP, m_objectlist[i], "");
      waitingForDetection.clear();

      m_command = NEXT_NBV;
      m_waitingCount = 0;
    }
    if (!waitingForDetection.empty()) {
      std::string logString;
      for (std::set<string>::iterator it = waitingForDetection.begin(); it
          != waitingForDetection.end(); it++) {
        logString += *it + " ";
      }
      log("Waiting for detection: %s", logString.c_str());
    }
    if (!waitingForObjects.empty()) {
      std::string logString;
      for (std::set<string>::iterator it = waitingForObjects.begin(); it
          != waitingForObjects.end(); it++) {
        logString += *it + " ";
      }
      log("Waiting for spatial objects: %s", logString.c_str());
    }
    break;
  }
  case IDLE: {
    log("Idling..");
    break;
  }
  }
}

void VisualObjectSearch::PostViewCone(const SensingAction &nbv) {
  /* Add plan to PB BEGIN */
  NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
  NavData::ViewPoint viewpoint;
  viewpoint.pos.x = nbv.pos[0];
  viewpoint.pos.y = nbv.pos[1];
  viewpoint.pos.z = nbv.pos[2];
  viewpoint.length = m_conedepth;
  viewpoint.pan = nbv.pan;
  viewpoint.tilt = nbv.tilt;
  obs->planlist.push_back(viewpoint);
  addToWorkingMemory(newDataID(), obs);
  /* Add plan to PB END */

  //cout << "selected cone " << viewpoint.pos.x << " " << viewpoint.pos.y << " " <<viewpoint.pos.z << " " << viewpoint.pan << " " << viewpoint.tilt << endl;
}

CurePDFMap* VisualObjectSearch::PopulateLGMap(const GridMap<GridMapData>* map) {
  log("populate lg map.");
  if (map == 0)
    map = m_map;
  CurePDFMap* lgmpdf = new CurePDFMap(m_lgm->getSize(), m_lgm->getCellSize(),
      0, CureObstMap::MAP1, m_lgm->getCentXW(), m_lgm->getCentYW());

  for (int x = -lgmpdf->getSize(); x <= lgmpdf->getSize(); x++) {
    for (int y = -lgmpdf->getSize(); y <= lgmpdf->getSize(); y++) {
      int bloxelX = x + lgmpdf->getSize();
      int bloxelY = y + lgmpdf->getSize();
      if ((*m_lgm)(x, y) != '2') {
        double bloxel_floor = 0;
        (*lgmpdf)(x, y) = 0;
        for (unsigned int i = 0; i < (*map)(bloxelX, bloxelY).size(); i++) {
          (*lgmpdf)(x, y) += (*map)(bloxelX, bloxelY)[i].data.pdf * ((*map)(
              bloxelX, bloxelY)[i].celing - bloxel_floor);
          bloxel_floor = (*map)(bloxelX, bloxelY)[i].celing;
        }
      }
    }
  }
  return lgmpdf;
}

void VisualObjectSearch::GoToNBV() {
  /* Show view cone on a temporary map, display the map and wipe it*/
  GDMakeObstacle makeobstacle;
  GDIsObstacle isobstacle;
  if (m_showconemap) {
    m_tempmap->coneModifier(m_nbv.pos[0], m_nbv.pos[1], m_nbv.pos[2],
        m_nbv.pan, m_nbv.tilt, m_horizangle, m_vertangle, m_conedepth, 5, 5,
        isobstacle, makeobstacle, makeobstacle);
    if (m_usePeekabot)
      pbVis->DisplayMap(*m_tempmap, "lastconemap");

    GridMapData def;
    def.occupancy = UNKNOWN;
    GDSet wipemap(def);
    BloxelFalse<GridMapData> dummy;
    m_tempmap->coneModifier(m_nbv.pos[0], m_nbv.pos[1], m_nbv.pos[2],
        m_nbv.pan, m_nbv.tilt, m_horizangle, m_vertangle, m_conedepth, 5, 5,
        dummy, wipemap, wipemap);
  }

  // Send move command
  Cure::Pose3D pos;
  pos.setX(m_nbv.pos[0]);
  pos.setY(m_nbv.pos[1]);
  pos.setTheta(m_nbv.pan);
  log("posting nav command");
  PostNavCommand(pos, SpatialData::GOTOPOSITION);
}
void VisualObjectSearch::LookforObjectWithStrategy() {
  //ask   
  m_command = EVALUATE_POLICIES;

  //    if (mode == DIRECT_UNINFORMED){
  //      currentSearchMode = DIRECT_UNINFORMED;
  //      log("Will look for %s", currentTarget.c_str());
  //      InitializePDFForObject(1-m_pout, currentTarget);
  //      PopulateLGMap();  
  //      pbVis->AddPDF(*m_map);
  //      m_map->clearDirty();
  //      m_command = NEXT_NBV;
  //    }
  //    else if (mode == DIRECT_INFORMED){
  //      currentSearchMode = DIRECT_INFORMED;
  //      log("Will look for %s", currentTarget.c_str());
  //      m_command = ASK_FOR_DISTRIBUTION;
  //    }
  //    else if(mode == INDIRECT){
  //      currentSearchMode = INDIRECT;
  //      ObjectPairRelation rel;
  //      while(true){
  //	rel = GetSecondaryObject(currentTarget);
  //	if(rel.primaryobject == "")
  //	  break;
  //	searchChain.push_back(rel);
  //	SetCurrentTarget(rel.secobject);
  //      }
  //      for(unsigned int i = 0; i < searchChain.size(); i++){
  //	cout << searchChain[i].primaryobject << " " << searchChain[i].secobject << " " << searchChain[i].prob << endl;
  //      }
  //      InitializePDFForObject(1-m_pout, currentTarget);
  //      PopulateLGMap();  
  //      pbVis->AddPDF(*m_map);
  //      m_map->clearDirty();
  //      m_command = NEXT_NBV;
  //    }
}

void VisualObjectSearch::AskForDistribution() {
  log("Asking for distribution");

  //if informed direct, get search chain, construct distribution parameters and ask for it
  m_pout = 0.0001;
  vector<ObjectPairRelation> strategyStep = getStrategyStep(
      currentSearchPolicy, currentPolicyStep);

  std::string topObject = strategyStep.back().primaryobject;
  SetCurrentTarget(topObject);
  //FIXME: if topObject is already detected than do not as for dist again

  // Set up query
  std::vector<std::string> labels;
  std::vector<FrontierInterface::ObjectRelation> relations;
  labels.push_back(topObject);
  string logstring = "Asking for: ";
  logstring = logstring + labels.back();
  for (int i = strategyStep.size() - 1; i >= 0; i--) {
    labels.push_back(strategyStep[i].secobject);
    relations.push_back(strategyStep[i].relation);
    logstring = logstring
        + (strategyStep[i].relation == FrontierInterface::ON ? string(" ON ")
            : string(" IN "));
    logstring = logstring + strategyStep[i].secobject;
  }
  log(logstring.c_str());
  writeStringToFile(logstring);
  if (find(m_recognizedobjects.begin(), m_recognizedobjects.end(), topObject)
      != m_recognizedobjects.end()) {
    log("we have already found this object");
    char buffer[256];
    sprintf(buffer,
        "We have already found object: %s, asking for the next step.",
        topObject.c_str());
    writeStringToFile((string) buffer);
    currentPolicyStep++;
    m_command = ASK_FOR_DISTRIBUTION;
    return;
  }
  FrontierInterface::WeightedPointCloudPtr queryCloud =
      new FrontierInterface::WeightedPointCloud;
  FrontierInterface::ObjectPriorRequestPtr objreq =
      new FrontierInterface::ObjectPriorRequest;
  objreq->relationTypes = relations; // ON or IN or whatnot
  objreq->objects = labels; // Names of objects, starting with the query object
  objreq->cellSize = m_cellsize; // Cell size of map (affects spacing of samples)
  objreq->outCloud = queryCloud; // Data struct to receive output
  objreq->totalMass = 1 - m_pout;

  string baseObject = strategyStep.front().secobject;

  if (baseObject.find("room") != string::npos) {
    if (strategyStep.size() == 1) {
      log("strategy contains room and has 1 step. Initing PDF for %s",
          strategyStep[0].primaryobject.c_str());
      InitializePDFForObject(1.0, strategyStep[0].primaryobject, m_map);
      PopulateLGMap();
      if (m_usePeekabot)
        pbVis->AddPDF(*m_map);
      m_map->clearDirty();
      m_command = NEXT_NBV;
    }

    else {
      GDProbScale reset(0.0);
      m_map->universalQuery(reset, true);
      log("strategy contains more than one relation");
      log("removing the room relation and then asking");
      // There's more than one relation. 
      // Strip off the "in room" relation from the policy for purposes
      // of relation query
      strategyStep.erase(strategyStep.begin());

      // remove the room relation from the request 
      objreq->objects.erase(objreq->objects.end() - 1);
      objreq->relationTypes.erase(objreq->relationTypes.end() - 1);

      // Huge HACK: REMOVEME
      if (objreq->objects.size() == 2 &&

      objreq->objects.back() == "table") {
        string filename = "table+with+" + objreq->objects.front() + ".txt";
        ifstream tableCloudFile(filename.c_str(), ios::in);
        if (tableCloudFile.is_open()) {

          log("Reading cloud from file");
          FrontierInterface::WeightedPointCloudPtr cloud =
              new FrontierInterface::WeightedPointCloud;
          tableCloudFile >> cloud->center.x;
          tableCloudFile >> cloud->center.y;
          tableCloudFile >> cloud->center.z;
          tableCloudFile >> cloud->interval;
          tableCloudFile >> cloud->xExtent;
          tableCloudFile >> cloud->yExtent;
          tableCloudFile >> cloud->zExtent;

          long int valn;

          tableCloudFile >> valn;

          cloud->values.resize(valn);

          for (unsigned long i = 0; i < cloud->values.size(); i++) {
            tableCloudFile >> cloud->values[i];
          }
          tableCloudFile >> cloud->isBaseObjectKnown;

          double totalMass;
          tableCloudFile >> totalMass;

          receivePointCloud(cloud, totalMass);

          m_command = NEXT_NBV;
        }
      }

      // Get the uninformed sample cloud from the ORM
      // Send request and wait for reply
      addToWorkingMemory(newDataID(), objreq);
      m_command = IDLE;
    }
  } else {
    GDProbScale reset(0.0);
    m_map->universalQuery(reset, true);
    log("strategy does not contain room");
    addToWorkingMemory(newDataID(), objreq);
    m_command = IDLE;
  }

  //    if (currentSearchMode == DIRECT_INFORMED){
  //      vector<FrontierInterface::ObjectRelation> relations;
  //      vector<string> labels;
  //      ObjectPairRelation rel;
  //      string name = currentTarget;
  //      while(true){
  //	rel = GetSecondaryObject(name);
  //	if(rel.primaryobject == "")
  //	  break;
  //	searchChain.push_back(rel); 
  //	name = rel.secobject;
  //	queryProbabilityMass *= rel.prob;
  //      }
  //      cout << queryProbabilityMass << endl;
  //      labels.push_back(currentTarget);
  //      for(unsigned int i = 0; i < searchChain.size(); i++){
  //	relations.push_back(searchChain[i].relation);
  //	labels.push_back(searchChain[i].secobject);
  //      }
  //      FrontierInterface::WeightedPointCloudPtr queryCloud = new FrontierInterface::WeightedPointCloud;
  //      //write lgm to WM
  //      FrontierInterface::ObjectPriorRequestPtr objreq =
  //	new FrontierInterface::ObjectPriorRequest;
  //      objreq->relationTypes = relations; // ON or IN or whatnot
  //      objreq->objects = labels;	// Names of objects, starting with the query object
  //      objreq->cellSize = m_cellsize;	// Cell size of map (affects spacing of samples)
  //      objreq->outCloud = queryCloud;	// Data struct to receive output
  //      objreq->totalMass = queryProbabilityMass;
  //      addToWorkingMemory(newDataID(), objreq);
  //
  //      cout << "Asked for: " << endl;
  //      cout << labels[0] << endl;
  //      for(unsigned int i = 0; i < relations.size(); i++){
  //	cout << (relations[i] == FrontierInterface::ON ? "ON" : "IN" ) << " " << labels[i + 1]  << endl;
  //      }
  //
  //    }
  //    //if indirect look where we are in the chain ask for it's distribution
  //    else if (currentSearchMode == INDIRECT){
  //      if(currentTarget == searchChain.back().secobject){
  //	// This is the first target, no relation exists
  //	// Just use the predefined uniform PDF
  //	return;
  //      }
  //
  //      vector<FrontierInterface::ObjectRelation> relations;
  //      vector<string> labels;
  //      unsigned int h;
  //      for(h=0; h<searchChain.size(); h++){
  //	if(searchChain[h].primaryobject == currentTarget){
  //	  log("breaking at %d",h);
  //	  break;
  //	}
  //      }
  //      labels.push_back(currentTarget);
  //      relations.push_back(searchChain[h].relation);
  //      labels.push_back(searchChain[h].secobject);
  //      cout << "Asked for: " << endl;
  //      for(unsigned int j = 0; j < labels.size(); j++){
  //	cout << labels[j]  << endl;
  //      }
  //      for(unsigned int j = 0; j < relations.size(); j++){
  //	cout << (relations[j] == FrontierInterface::ON ? "ON" : "IN" )  << endl;
  //      }
  //      FrontierInterface::WeightedPointCloudPtr queryCloud = new FrontierInterface::WeightedPointCloud;
  //      //write lgm to WM
  //      FrontierInterface::ObjectPriorRequestPtr objreq =
  //	new FrontierInterface::ObjectPriorRequest;
  //      objreq->relationTypes = relations; // ON or IN or whatnot
  //      objreq->objects = labels;	// Names of objects, starting with the query object
  //      objreq->cellSize = m_cellsize;	// Cell size of map (affects spacing of samples)
  //      objreq->outCloud = queryCloud;	// Data struct to receive output
  //      addToWorkingMemory(newDataID(), objreq);
  //    }
  if (m_usePeekabot)
    pbVis->AddPDF(*m_map);
  m_map->clearDirty();
}

void VisualObjectSearch::receivePointCloud(
    FrontierInterface::WeightedPointCloudPtr cloud, double totalMass) {

  log("KDEing cloud read from file");
  vector<Vector3> centers;
  centers.push_back(cloud->center);

  //totalMass represents the probability of the relation
  //GIVEN that the object is in the room.
  //We want m_pout to remain unchanged, and the probability mass
  //already in the map diminished so that the relative probability
  //inside the room coming from the cloud is totalMass.
  //I.e., after normalization its mass should be (1-pout)*totalMass.
  //Before normalization, it will need to be 1/(1-totalMass) times larger
  double weightToAdd;
  if (totalMass > 0.99999999999) {
    GDProbScale reset(0.0);
    m_map->universalQuery(reset, true);
    weightToAdd = (1 - m_pout);
  } else {
    weightToAdd = (1 - m_pout) * totalMass / (1 - totalMass);
  }

  cout << totalMass << endl;
  if (cloud->isBaseObjectKnown) {
    log("Got distribution around known object pose");
    m_sampler.kernelDensityEstimation3D(*m_map, centers, cloud->interval,
        cloud->xExtent, cloud->yExtent, cloud->zExtent, cloud->values, 1.0,
        weightToAdd, m_lgm);
    normalizePDF(*m_map, 1.0 - m_pout);
  } else {
    log("Got distribution around unknown object pose");
    vector<pair<int, int> > obstacleCells;
    const unsigned int nTargetKernelSamples = 100;
    const int step1 = 20;
    const int step2 = 1;

    // Form a diagonal striped pattern of samples across the map
    // This is a dirty trick, as it assumes the spatial frequency is low along the diagonal
    while (obstacleCells.size() < nTargetKernelSamples) {
      int offset1 = rand() % step1;
      for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
        for (int y = -m_lgm->getSize() + (offset1 + (x + m_lgm->getSize())
            * step2) % step1; y <= m_lgm->getSize(); y += step1) {
          int bloxelX = x + m_lgm->getSize();
          int bloxelY = y + m_lgm->getSize();
          for (int x2 = x - 1; x2 <= x + 1; x2++) {
            if (x2 >= -m_lgm->getSize() && x2 <= m_lgm->getSize()) {
              for (int y2 = y - 1; y2 <= y + 1; y2++) {
                if (y2 >= -m_lgm->getSize() && y2 <= m_lgm->getSize()) {
                  if ((*m_lgm)(x2, y2) == '1' || (*m_lgm)(x2, y2) == '3') {
                    obstacleCells.push_back(make_pair<int> (bloxelX, bloxelY));
                  }
                }
              }
            }
          }
        }
      }
    }

    vector<Vector3> centers;
    for (unsigned int i = 0; i < obstacleCells.size(); i++) {
      // place a KDE cloud around it, with the center given
      // by the relation manager (which includes a z-coordinate
      // for tables, desks etc)
      int bloxelX = obstacleCells[i].first;
      int bloxelY = obstacleCells[i].second;
      pair<double, double> kernelCoords = m_map->gridToWorldCoords(bloxelX,
          bloxelY);
      cloud->center.x = kernelCoords.first;
      cloud->center.y = kernelCoords.second;
      centers.push_back(cloud->center);
    }

    m_sampler.kernelDensityEstimation3D(*m_map, centers, cloud->interval,
        cloud->xExtent, cloud->yExtent, cloud->zExtent, cloud->values, 1.0
            / (m_lgm->getSize()), //Just an ad-hoc guess
        weightToAdd, m_lgm);
    normalizePDF(*m_map, 1.0 - m_pout);
  }

  //	vector< pair<double,double> > thresholds;
  //	thresholds.push_back(make_pair(1e-4,1e-3));
  //	thresholds.push_back(make_pair(1e-3,5e-3));
  //	thresholds.push_back(make_pair(5e-3,1e-2));
  //	thresholds.push_back(make_pair(1e-2,5e-2));
  //	thresholds.push_back(make_pair(5e-2,1e2));
  if (m_usePeekabot)
    pbVis->AddPDF(*m_map);
  m_map->clearDirty();
  gotPC = true;
}

void VisualObjectSearch::owtWeightedPointCloud(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    log("got weighted PC");
    FrontierInterface::ObjectPriorRequestPtr req = getMemoryEntry<
        FrontierInterface::ObjectPriorRequest> (objID.address);
    FrontierInterface::WeightedPointCloudPtr cloud = req->outCloud;

    if (m_bSimulation || m_bEvaluation) {
      log("in simulation");
      m_priorreq = req;
      m_bEvaluation = false;
      return;
    }

    receivePointCloud(cloud, req->totalMass);

    m_command = NEXT_NBV;
  } catch (DoesNotExistOnWMException excp) {
    log("Error!  WeightedPointCloud does not exist on WM!");
    return;
  }
  //      else {
  //	// The base object pose is unknown. The returned cloud->center
  //	// represents just a sample offset; we must populate the grid map with
  //	// a pdf based on 
  //      }
}
bool VisualObjectSearch::isCircleFree(double xW, double yW, double rad) {
  int xiC, yiC;
  if (m_lgm->worldCoords2Index(xW, yW, xiC, yiC) != 0)
    return false;

  double w = rad / m_lgm->getCellSize();
  int wi = int(w + 0.5);

  for (int x = xiC - wi; x <= xiC + wi; x++) {
    for (int y = yiC - wi; y <= yiC + wi; y++) {
      if (x >= -m_lgm->getSize() && x <= m_lgm->getSize() && y
          >= -m_lgm->getSize() && y <= m_lgm->getSize()) {
        if (hypot(x - xiC, y - yiC) < w) {
          if ((*m_lgm)(x, y) == '1' or (*m_lgm)(x, y) == '2')
            return false;
        }
      }
    }
  }
  return true;
}

bool isCircleFree2D(const CureObstMap &map, double xW, double yW, double rad) {
  int xiC, yiC;
  if (map.worldCoords2Index(xW, yW, xiC, yiC) != 0) {
    CureCERR(30) << "Querying area outside the map (xW=" << xW << ", yW=" << yW
        << ")\n";
    return true;
  }

  double w = rad / map.getCellSize();
  int wi = int(w + 0.5);
  int size = map.getSize();

  for (int x = xiC - wi; x <= xiC + wi; x++) {
    for (int y = yiC - wi; y <= yiC + wi; y++) {
      if (x >= -size && x <= size && y >= -size && y <= size) {
        if (hypot(x - xiC, y - yiC) < w) {
          if (map(x, y) != '0')
            return false;
        }
      }
    }
  }

  return true;
}

bool pathgridseen = false;
double VisualObjectSearch::GetPathLength(Cure::Pose3D start,
    Cure::Pose3D destination, CureObstMap* lgm) {

  if (lgm == 0)
    lgm = m_lgm;
  /*Checking if a point in x,y is reachable */

  /// Binary grid that is 0 for foree space and 1 for occupied
  Cure::BinaryMatrix m_NonFreeSpace;

  // We make the number of columns of the BinaryMatrix a multiple

  // of 32 so that we get the benefit of the representation.
  // Here m_LGMap is assumed to be the LocalGridMap
  int rows = 2 * lgm->getSize() + 1;
  int cols = ((2 * lgm->getSize() + 1) / 32 + 1) * 32;
  m_NonFreeSpace.reallocate(rows, cols);
  m_NonFreeSpace = 0; // Set all cells to zero

  // First we create a binary matrix where all cells that
  // corresponds to known obstacles are set to "1".
  for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
    for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
      if ((*lgm)(x, y) == '1') { // occupied OR Plane
        m_NonFreeSpace.setBit(x + lgm->getSize(), y + lgm->getSize(), true);
      }
    }
  }

  // Create an istance of BinaryMatrx which will hold the result of
  // expanding the obstacles
  Cure::BinaryMatrix m_PathGrid;
  m_PathGrid = 0;

  // Grow each occupied cell to account for the size of the
  // robot. We put the result in another binary matrix, m_PathGrid
  m_NonFreeSpace.growInto(m_PathGrid, 0.5 * 0.01 / lgm->getCellSize(), true);
  // 0.45 is robot width hard coded here.
  // We treat all unknown cells as occupied so that the robot only
  // uses paths that it knowns to be free. Note that we perfom this
  // operation directly on the m_PathGrid, i.e. the grid with the
  // expanded obstacle. The reasoning behind this is that we do not
  // want the unknown cells to be expanded as well as we would have
  // to recalculate the position of the frontiers otherwise, else
  // they might end up inside an obstacle (could happen now as well
  // from expanding the occupied cell but then it is known not to be
  // reachable).
  for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
    for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
      if ((*lgm)(x, y) == '2') {
        m_PathGrid.setBit(x + lgm->getSize(), y + lgm->getSize(), true);
      }
    }
  }
  // if(!pathgridseen)
  //  pbVis->Display2DBinaryMap(m_PathGrid, lgm);

  //	pathgridseen = true;
  /*Checking if a point in x,y is reachable */

  // Get the indices of the destination coordinates
  //		log("point reachable");
  int rS, cS, rE, cE;
  //log("robotpose: %f,%f", lastRobotPose->x,lastRobotPose->y);
  //log("1");
  if (m_lgm->worldCoords2Index(start.getX(), start.getY(), rS, cS) == 0
      && m_lgm->worldCoords2Index(destination.getX(), destination.getY(), rE,
          cE) == 0) {
    // Compensate for the fact that the PathGrid is just a normal matrix where the cells are numbers from the corner
    cS += m_lgm->getSize();
    rS += m_lgm->getSize();
    cE += m_lgm->getSize();

    rE += m_lgm->getSize();

    Cure::ShortMatrix path;
    double d = (m_PathGrid.path(rS, cS, rE, cE, path, 100 * m_lgm->getSize())
        * m_lgm->getCellSize());
    return d;
  } else {
    return -1;
  }

}
std::vector<Cure::Pose3D> VisualObjectSearch::Sample2DGrid() {
  srand ( time(NULL));
  std::vector<double> angles;

  std::vector<Cure::Pose3D> samples;
  log("Sampling first in in 2D grid.");
  /*Sampling free space BEGIN*/

  for (double rad = 0; rad < M_PI * 2; rad = rad + M_PI / 18) {
    angles.push_back(rad);
  }
  int i = 0;
  int randx, randy;
  double xW, yW, angle;
  //log("for sample size");
  bool haspoint;
  Cure::Pose3D singlesample;
  while (i < m_samplesize) {
    haspoint = false;
    randx = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
    randy = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
    int the = (int) (rand() % angles.size());
    angle = angles[the];
    //if we have that point already, skip.
    /* for (int j = 0; j < i; j++) {
     if (samples[2 * j] == randx && m_samples[2 * j + 1] == randy
     && m_samplestheta[j] == angle) {
     //log("we already have this point.");
     haspoint = true;
     break;
     }
     }*/
    m_lgm->index2WorldCoords(randx, randy, xW, yW);
    std::pair<int, int> sample;
    sample.first = randx;
    sample.second = randy;

    if (!haspoint && (*m_lgm)(randx, randy) == '0' && isCircleFree2D(*m_lgm,
        xW, yW, m_sampleawayfromobs)) {
      Cure::Pose3D start, dest;
      start.setX(lastRobotPose->x);
      start.setY(lastRobotPose->y);
      dest.setX(xW);
      dest.setX(yW);

      double d = GetPathLength(start, dest, m_lgm);
      //	log ("path to here: %3.2f", d);
      // There is a path to this destination
      //	    log("there's a path to this destination");
      if (d > 0 || true) {
        singlesample.setX(randx);
        singlesample.setY(randy);
        singlesample.setTheta(angle);
        samples.push_back(singlesample);
        i++;
      }

    } else {
      //	log("point either non free space or seen.");
    }
  }
  log("Displaying VP samples in PB");

  /* Display Samples in PB BEGIN */
  //samples.set_opacity(0.2);

  vector<vector<double> > sampled2Dpoints;
  double xW1, yW1;
  for (unsigned int i = 0; i < samples.size(); i++) {
    vector<double> point;
    m_lgm->index2WorldCoords(samples[i].getX(), samples[i].getY(), xW1, yW1);
    point.push_back(xW1);
    point.push_back(yW1);
    point.push_back(1.4);
    sampled2Dpoints.push_back(point);
  }
  //if(m_usePeekabot)	
  //pbVis->Add3DPointCloud(sampled2Dpoints, true, "sample_points");
  /* Display Samples in PB END */
  /*Sampling free space END*/
  return samples;
}
std::vector<std::vector<pair<int, int> > > VisualObjectSearch::GetViewCones(
    std::vector<Cure::Pose3D> samples2D) {
  log("Calculating 3D view cones for generated 2D samples");
  Cure::Pose3D candidatePose;
  XVector3D a;
  std::vector<pair<int, int> > tpoints;
  std::vector<std::vector<pair<int, int> > > ViewConePts;

  for (unsigned int y = 0; y < samples2D.size(); y++) { //calc. view cone for each sample

    m_lgm->index2WorldCoords(samples2D[y].getX(), samples2D[y].getY(), a.x, a.y);
    a.theta = samples2D[y].getTheta();
    tpoints = GetInsideViewCone(a, true);
    ViewConePts.push_back(tpoints);
  }
  return ViewConePts;
}

std::vector<pair<int, int> > VisualObjectSearch::GetInsideViewCone(
    XVector3D &a, bool addall) {
  //log("get inside view cone");
  std::vector<pair<int, int> > tpoints;
  XVector3D b, c, p;
  XVector3D m_a, m_b, m_c;
  int* rectangle = new int[4];
  int h, k;
  CalculateViewCone(a, a.theta, m_conedepth, m_horizangle, b, c);

  m_lgm->worldCoords2Index(a.x, a.y, h, k);
  m_a.x = h;
  m_a.y = k;
  m_lgm->worldCoords2Index(b.x, b.y, h, k);
  m_b.x = h;
  m_b.y = k;
  m_lgm->worldCoords2Index(c.x, c.y, h, k);
  m_c.x = h;
  m_c.y = k;
  //  log("Got Map triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", m_a.x,m_a.y,m_b.x,m_b.y,m_c.x,m_c.y);

  FindBoundingRectangle(m_a, m_b, m_c, rectangle);
  //log("XRectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",rectangle[0],rectangle[2]
  //,rectangle[1], rectangle[3]);
  for (int x = rectangle[0]; x < rectangle[1]; x++) // rectangle bounding triangle
  {
    for (int y = rectangle[2]; y < rectangle[3]; y++) {
      p.x = x;
      p.y = y;
      if (isPointInsideTriangle(p, m_a, m_b, m_c)) {
        tpoints.push_back(make_pair(x, y));
      }

    }
  }
  vector<pair<int, int> >::iterator theIterator = tpoints.begin();
  tpoints.insert(theIterator, 1, make_pair(a.x, a.y));
  return tpoints;

}
void VisualObjectSearch::CalculateViewCone(XVector3D a, double direction,
    double range, double fov, XVector3D &b, XVector3D &c) {
  //log("Direction: %f, FOV:%f, range: %f", direction, fov, range);
  float angle1 = direction + fov / 2;
  float angle2 = direction - fov / 2;
  b.x = cos(angle1) * range + a.x;
  c.x = cos(angle2) * range + a.x;
  b.y = sin(angle1) * range + a.y;
  c.y = sin(angle2) * range + a.y;
  //log("Got triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", a.x,a.y,b.x,b.y,c.x,c.y);
}

void VisualObjectSearch::FindBoundingRectangle(XVector3D a, XVector3D b,
    XVector3D c, int* rectangle) {
  int maxx, maxy, minx, miny;
  maxy = max(max(a.y, b.y), c.y);
  maxx = max(max(a.x, b.x), c.x);
  miny = min(min(a.y, b.y), c.y);
  minx = min(min(a.x, b.x), c.x);
  rectangle[0] = minx;
  rectangle[1] = maxx;
  rectangle[2] = miny;
  rectangle[3] = maxy;
  //log("Rectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",minx,miny,maxx, maxy);
}

bool VisualObjectSearch::isPointInsideTriangle(XVector3D p, XVector3D a,
    XVector3D b, XVector3D c) { //the first one is the point the rest triangle

  if (isPointSameSide(p, a, b, c) && isPointSameSide(p, b, a, c)
      && isPointSameSide(p, c, a, b)) {
    return true;
  } else {
    return false;
  }

}
bool VisualObjectSearch::isPointSameSide(XVector3D p1, XVector3D p2,
    XVector3D a, XVector3D b) {
  XVector3D cp1 = (b - a).crossVector3D((p1 - a));
  XVector3D cp2 = (b - a).crossVector3D((p2 - a));
  if (cp1.dotVector3D(cp2) >= 0) {
    return true;
  } else {
    return false;
  }

}

VisualObjectSearch::SensingAction VisualObjectSearch::SampleAndSelect(GridMap<
    GridMapData>* tmpMap) {
  if (tmpMap == 0)
    tmpMap = m_map;
  CurePDFMap* lgmpdf = PopulateLGMap(tmpMap);
  double cameraheight = 1.4;

  double lgmpdfsum = 0;
  for (int x = -lgmpdf->getSize(); x <= lgmpdf->getSize(); x++) {
    for (int y = -lgmpdf->getSize(); y <= lgmpdf->getSize(); y++) {
      lgmpdfsum += (*lgmpdf)(x, y);
    }
  }
  cout << "lgmpdf map sums to: " << lgmpdfsum << endl;
  std::vector<std::vector<pair<int, int> > > VCones;
  std::vector<Cure::Pose3D> samples2D = Sample2DGrid();
  VCones = GetViewCones(samples2D);
  double sum;
  int x, y;
  vector<pair<unsigned int, double> > orderedVClist, tmp;
  vector<unsigned int>::iterator it;
  for (unsigned int i = 0; i < VCones.size(); i++) {
    sum = 0;
    for (unsigned int j = 0; j < VCones[i].size(); j++) {
      x = VCones[i][j].first;
      y = VCones[i][j].second;
      if ((x > -lgmpdf->getSize() && x < lgmpdf->getSize()) && (y
          > -lgmpdf->getSize() && y < lgmpdf->getSize()))
        sum += (*lgmpdf)(x, y);
    }
    if (orderedVClist.size() == 0) {
      orderedVClist.push_back(make_pair(i, sum));
    } else {
      bool inserted = false;
      tmp = orderedVClist;
      for (unsigned int j = 0; j < tmp.size(); j++) {
        if (sum >= orderedVClist[j].second) {
          inserted = true;
          orderedVClist.insert(orderedVClist.begin() + j, make_pair(i, sum));
          break;
        }
      }
      if (!inserted) {
        //	  log("pushing back");
        orderedVClist.push_back(make_pair(i, sum));
      }
    }
  }
  /*for (unsigned int j=0; j < orderedVClist.size(); j++)
   cout << orderedVClist[j].first << "," << orderedVClist[j].second << "," << endl;*/

  SensingAction sample;
  std::vector<SensingAction> samplepoints;

  std::vector<double> angles;
  for (double rad = -30 * M_PI / 180; rad < 30 * M_PI / 180; rad = rad
      + m_tiltinterval) {
    angles.push_back(rad);
  }

  // We have the VC candidate list ordered according to their lgmpdf sums (2D)
  // now for the top X candidate get a bunch of tilt angles and calculate the 3D cone sums
  double xW, yW;
  for (unsigned int j = 0; j < orderedVClist.size() * m_best3DConeRatio; j++) {
    m_lgm->index2WorldCoords(samples2D[orderedVClist[j].first].getX(),
        samples2D[orderedVClist[j].first].getY(), xW, yW);

    for (unsigned int i = 0; i < angles.size(); i++) {
      std::vector<double> coord;
      // coord.push_back(m_SlamRobotPose.getX());
      // coord.push_back(m_SlamRobotPose.getY());
      coord.push_back(xW);
      coord.push_back(yW);
      coord.push_back(cameraheight);
      SensingAction sample;
      sample.pos = coord;
      sample.pan = samples2D[orderedVClist[j].first].getTheta();
      // sample.pan = m_SlamRobotPose.getTheta();
      sample.tilt = angles[i];
      samplepoints.push_back(sample);
    }
  }

  // keyed with room id
  int bestConeIndex = GetViewConeSums(samplepoints, tmpMap);
  if (bestConeIndex == -1) {
    log(
        "No View Cone is selected possible cause is grid map does not contain any obstacles or too little free space area, returning without doing anything");
    SensingAction wbv;
    wbv.totalprob = -1;
    return wbv;
  } else {
    delete lgmpdf;
    log("returning best nbv: %d", bestConeIndex);
    return samplepoints[bestConeIndex];
  }
}

int VisualObjectSearch::GetViewConeSums(
    std::vector<SensingAction> &samplepoints, GridMap<GridMapData> *map) {
  log("Getting view cone sums");
  //	log("Cone range is %f to %f", m_minDistance, m_conedepth);

  if (map == 0)
    map = m_map;

  GDProbSum sumcells;
  GDIsObstacle isobstacle;
  double maxpdf = -1000.0;
  int maxindex = -1;

  try {
    for (unsigned int i = 0; i < samplepoints.size(); i++) {
      SensingAction viewpoint = samplepoints[i];
      /*m_map->coneModifier(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan,samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, makeobstacle,makeobstacle);*/
      map->coneQuery(samplepoints[i].pos[0], samplepoints[i].pos[1],
          samplepoints[i].pos[2], samplepoints[i].pan, samplepoints[i].tilt,
          m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, sumcells,
          sumcells, m_minDistance);
      //  log("cone query done.");
      cout << "cone #" << i << " " << viewpoint.pos[0] << " "
          << viewpoint.pos[1] << " " << viewpoint.pos[2] << " "
          << viewpoint.pan << " " << viewpoint.tilt << " pdfsum of cone: "
          << sumcells.getResult() << endl;

      //    /* Show view cone on a temporary map, display the map and wipe it*/

      if (m_showconemap) {
        GDMakeObstacle makeobstacle;
        GDIsObstacle isobstacle;
        m_tempmap = map;
        m_tempmap->coneModifier(viewpoint.pos[0], viewpoint.pos[1],
            viewpoint.pos[2], viewpoint.pan, viewpoint.tilt, m_horizangle,
            m_vertangle, 3.0, 5, 5, isobstacle, makeobstacle, makeobstacle);
        if (m_usePeekabot)
          pbVis->DisplayMap(*m_tempmap, "lastconemap");
        sleepComponent(5000);

        GridMapData def;
        def.occupancy = UNKNOWN;
        GDSet wipemap(def);
        BloxelFalse<GridMapData> dummy;
        m_tempmap->coneModifier(viewpoint.pos[0], viewpoint.pos[1],
            viewpoint.pos[2], viewpoint.pan, viewpoint.tilt, m_horizangle,
            m_vertangle, 3.0, 5, 5, dummy, wipemap, wipemap);
      }

      samplepoints[i].totalprob = sumcells.getResult();
      if (sumcells.getResult() > maxpdf) {
        maxpdf = sumcells.getResult();
        maxindex = i;
      }
      sumcells.reset();
    }
  } catch (std::exception &e) {
    printf("Caught exception %s: \n", e.what());
  }
  return maxindex;
}

/*  void
 VisualObjectSearch::owtARTagCommand(const cast::cdl::WorkingMemoryChange &objID) {
 try{
 VisionData::ARTagCommandPtr cmd = (getMemoryEntry<
 VisionData::ARTagCommand> (objID.address));

 log("got ARTag overwrite command: %s", cmd->label.c_str());
 // First of all check if we have found the holy grail

 if(waitingForDetection.find(cmd->label) != waitingForDetection.end() ){
 //We were waiting for detection results on this object
 log("ARTag owt: erasing %s from the list", cmd->label.c_str());
 waitingForDetection.erase(cmd->label);

 string logString(" Recognizer Waiting list: ");
 for (std::set<string>::iterator it = waitingForObjects.begin(); it != waitingForObjects.end(); it++) {
 logString += *it + " ";
 }
 logString += " / ";
 for (std::set<string>::iterator it = waitingForDetection.begin(); it != waitingForDetection.end(); it++) {
 logString += *it + " ";
 }
 log(logString.c_str());

 if (waitingForObjects.empty() && waitingForDetection.empty()) {
 DetectionComplete(false);
 }

 }
 else{
 log("this is not the object we looked for: %s",cmd->label.c_str());
 string logString(" Recognizer Waiting list: ");
 for (std::set<string>::iterator it = waitingForObjects.begin(); it != waitingForObjects.end(); it++) {
 logString += *it + " ";
 }
 logString += " / ";
 for (std::set<string>::iterator it = waitingForDetection.begin(); it != waitingForDetection.end(); it++) {
 logString += *it + " ";
 }
 log(logString.c_str());

 // m_command = NEXT_NBV;
 }
 }
 catch (const CASTException &e) {
 log("failed to delete SpatialDataCommand: %s", e.message.c_str());
 }
 }*/

void VisualObjectSearch::owtRecognizer3DCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("got recognizer3D overwrite command:");
  try {
    VisionData::Recognizer3DCommandPtr cmd(getMemoryEntry<
        VisionData::Recognizer3DCommand> (objID.address));

    //FIXME: nah took the below out, otherwise planner never gets told anything
    // 	    if(m_publishSimCones)
    // 	      return;


    log("got recognizer3D overwrite command: %s", cmd->label.c_str());
    // First of all check if we have found the holy grail
    waitingForDetection.erase(cmd->label);

    string logString(" Recognizer Waiting list: ");
    logString += " / ";
    for (std::set<string>::iterator it = waitingForDetection.begin(); it
        != waitingForDetection.end(); it++) {
      logString += *it + " ";
    }
    log(logString.c_str());

    if (waitingForDetection.empty() && !m_ProcessVPID.empty()) {
      log("Letting planner know that view point is processed.");
      SpatialData::ProcessViewPointCommandPtr VPcmd =
          new SpatialData::ProcessViewPointCommand;
      VPcmd->status = SpatialData::SUCCESS;
      overwriteWorkingMemory(m_ProcessVPID, VPcmd);
      m_ProcessVPID = "";
      //Moving pant-tilt back (to zero?)
      MovePanTilt(0.0, 0.0, 0.08);
    }

    if (m_publishSimCones)
      ;
    return;
    if (waitingForDetection.find(cmd->label) != waitingForDetection.end()) {
      //We were waiting for detection results on this object

      string logString(" Recognizer Waiting list: ");
      for (std::set<string>::iterator it = waitingForObjects.begin(); it
          != waitingForObjects.end(); it++) {
        logString += *it + " ";
      }
      logString += " / ";
      for (std::set<string>::iterator it = waitingForDetection.begin(); it
          != waitingForDetection.end(); it++) {
        logString += *it + " ";
      }
      log(logString.c_str());

    } else {
      log("this is not the object we looked for: %s", cmd->label.c_str());
      string logString(" Recognizer Waiting list: ");
      for (std::set<string>::iterator it = waitingForObjects.begin(); it
          != waitingForObjects.end(); it++) {
        logString += *it + " ";
      }
      logString += " / ";
      for (std::set<string>::iterator it = waitingForDetection.begin(); it
          != waitingForDetection.end(); it++) {
        logString += *it + " ";
      }
      log(logString.c_str());

      // m_command = NEXT_NBV;
    }

  } catch (const CASTException &e) {
    log("owtRecognizer3DCommand disappeared from WM: %s", e.message.c_str());

  }
}

void VisualObjectSearch::MovePanTilt(double pan, double tilt, double tolerance) {

  if (m_usePTZ) {
    log(" Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
    ptz::PTZPose p;
    ptz::PTZReading ptuPose;
    p.pan = pan;
    p.tilt = tilt;

    p.zoom = 0;
    m_ptzInterface->setPose(p);
    bool run = true;
    ptuPose = m_ptzInterface->getPose();
    double actualpan = ptuPose.pose.pan;
    double actualtilt = ptuPose.pose.tilt;

    while (run) {
      m_ptzInterface->setPose(p);
      ptuPose = m_ptzInterface->getPose();
      actualpan = ptuPose.pose.pan;
      actualtilt = ptuPose.pose.tilt;

      log("desired pan tilt is: %f %f", pan, tilt);
      log("actual pan tilt is: %f %f", actualpan, actualtilt);
      log("tolerance is: %f", tolerance);

      //check that pan falls in tolerance range
      if (actualpan < (pan + tolerance) && actualpan > (pan - tolerance)) {
        run = false;
      }

      //only check tilt if pan is ok

      if (!run) {
        if (m_ignoreTilt) {
          run = false;
        } else {
          if (actualtilt < (tilt + tolerance) && actualtilt
              > (tilt - tolerance)) {
            run = false;
          } else {
            //if the previous check fails, loop again
            run = true;
          }
        }
      }

      usleep(100000);
    }
    log("Moved.");
    sleep(1);
  }
}
void VisualObjectSearch::Recognize() {
  log("Recognize called");
  //    waitingForDetection.insert(currentTarget);
  //    isWaitingForDetection = true;
  //DetectionComplete(false);
  // FIXME Below is commented out just for easy testing
  /*
   ptz::PTZReading ptz = m_ptzInterface->getPose();
   Cure::Pose3D currpos = m_SlamRobotPose;
   double anglediff = Cure::HelpFunctions::angleDiffRad(m_nbv.pan,currpos.getTheta());

   log("plantheta : %f, currtheta, %f", m_nbv.pan, currpos.getTheta());
   log("anglediff is: %f", anglediff);
   log("ptz reading: %f", ptz.pose.pan);

   MovePanTilt(anglediff,m_nbv.tilt,0.08);
   */
  for (unsigned int i = 0; i < m_objectlist.size(); i++) {
    /*if (find(m_recognizedobjects.begin(), m_recognizedobjects.end(),m_objectlist[i]) == m_recognizedobjects.end()){

     /* if(m_objectlist[i] == "metalbox" || m_objectlist[i] == "table1" || m_objectlist[i] == "table2"
     || m_objectlist[i] == "shelves" ){
     log("asking to ARTag for object %s", m_objectlist[i].c_str());
     addARTagCommand(m_objectlist[i]);
     waitingForDetection.insert(m_objectlist[i]);
     }
     else{*/
    log("Posting Recognition command for object %s", m_objectlist[i].c_str());
    addRecognizer3DCommand(VisionData::RECOGNIZE, m_objectlist[i], "");
    waitingForDetection.insert(m_objectlist[i]);
    //}
    //}
  }

}
/*      void VisualObjectSearch::addARTagCommand(std::string label){
 VisionData::ARTagCommandPtr cmd = new VisionData::ARTagCommand;
 cmd->label = label;
 AVSComponentPtr->addToWorkingMemory(AVSComponentPtr->newDataID(),"vision.sa",cmd);
 }*/

void VisualObjectSearch::DetectionComplete(bool isDetected,
    std::string detectedObject) {
  if (m_publishSimCones) {
    return;
  }

  waitingForDetection.clear();
  waitingForObjects.clear();

  cout << "Detection complete" << endl;
  if (isDetected) {
    m_recognizedobjects.push_back(currentTarget);
    log("Object Detected.");
    // check for success
    bool greatSuccess = false;
    if (targetObject == detectedObject) {
      greatSuccess = true;
    }
    if (greatSuccess) {
      isSearchFinished = true;
      m_command = STOP;

      log("Object Detected, Mission Completed.");
      return;
    }

    char buffer[1024];
    sprintf(
        buffer,
        "Object %s detected. Policy Step completed: %s, total # of VCones for this step: %d, (next step: %s)",
        detectedObject.c_str(), currentSearchPolicy[currentPolicyStep].c_str(),
        m_totalViewPoints, currentSearchPolicy[currentPolicyStep + 1].c_str());
    writeStringToFile((string) buffer);

    m_totalViewPoints = 0;
    currentPolicyStep++;

    m_command = ASK_FOR_DISTRIBUTION;

  }
  //      // if we are doing indirect search then ask & initialize next object
  //      if(currentSearchMode == INDIRECT){
  //
  //	// if we are doing indirect search and happen to find the middle object
  //	// then set the current target to middle object so that we will as for distribution of primary object SR middle object
  //	if(detectedObject == indirect_middle_object){
  //	  SetCurrentTarget(targetObject);
  //	}
  //
  //      }
  //      for(unsigned int i=0; i<searchChain.size(); i++){
  //	if(searchChain[i].secobject == currentTarget){
  //
  //	  SetCurrentTarget(searchChain[i].primaryobject);
  //
  //	}
  //	log("Object Detected, new target is %s", currentTarget.c_str());
  //	m_command = ASK_FOR_DISTRIBUTION;
  //      }
  else {
    // if we are not yet finished Go to NBV
    log("Object not detected");
    UnsuccessfulDetection( m_nbv);
    m_command = NEXT_NBV;
  }
}

void VisualObjectSearch::SetCurrentTarget(const string &label) {
  currentTarget = label;
  double objectSize = 0.25;
  if (label == "table1" || label == "table") {
    objectSize = 1.1;
  }
  if (label == "table2") {
    objectSize = 1.8;
  } else if (label == "bookcase_sm") {
    objectSize = 1.5;
  } else if (label == "bookcase_lg") {
    objectSize = 1.93;
  } else if (label == "shelves") {
    objectSize = 2.15;
  } else if (label == "desk") {
    objectSize = 2.0;
  } else if (label == "rice") {
    objectSize = 0.19;
  } else if (label == "dell") {
    objectSize = 0.49;
  } else if (label == "metalbox") {
    objectSize = 0.76;
  } else if (label == "book" || label == "frosties") {
    objectSize = 0.19;
  } else if (label == "rovio") {
    objectSize = 0.35;
  }
  m_minDistance = 0.5 * objectSize / tan(0.5 * m_vertangle);
  //      if (m_minDistance < 0.5) m_minDistance = 0.5;
  m_conedepth = m_minDistance * 6;
  if (m_conedepth > 5.0)
    m_conedepth = 5.0;
  log("Current target changed to: %s, conedepth %f", currentTarget.c_str(),
      m_conedepth);
}

void VisualObjectSearch::newVisualObject(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("new visual object");
  try {
    VisionData::VisualObjectPtr visualobject(getMemoryEntry<
        VisionData::VisualObject> (objID.address));

    //FIXME: is the below true ?
    string objLabel = visualobject->identLabels[0];
    log("new visual object %s", objLabel.c_str());
    if (waitingForDetection.find(objLabel) != waitingForDetection.end()) {
      //m_recognizedobjects.push_back(visualobject->label); 
      //This was an object we were looking for
      log("deleting object %s from list", objLabel.c_str());
      waitingForDetection.erase(objLabel);
      waitingForObjects.insert(objLabel);

      string logString("Waiting list: ");
      for (std::set<string>::iterator it = waitingForObjects.begin(); it
          != waitingForObjects.end(); it++) {
        logString += *it + " ";
      }
      logString += " / ";
      for (std::set<string>::iterator it = waitingForDetection.begin(); it
          != waitingForDetection.end(); it++) {
        logString += *it + " ";
      }
      log(logString.c_str());
    }
  } catch (const CASTException &e) {
    log("newVisualObject disappeared from WM: %s", e.message.c_str());

  }
}

void VisualObjectSearch::UnsuccessfulDetection(SensingAction viewcone, GridMap<
    GridMapData> *map) {

  if (map == 0)
    map = m_map;

  m_totalprob += m_nbv.totalprob;
  //log("total prob covered so far %f", m_totalprob);

  double sensingProb = 0.8;
  GDProbSum sumcells;
  GDIsObstacle isobstacle;
  /* DEBUG */
  GDProbSum conesum;
  map->coneQuery(viewcone.pos[0], viewcone.pos[1], viewcone.pos[2],
      viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10,
      10, isobstacle, conesum, conesum, m_minDistance);
  //	cout << "cone sums to " << conesum.getResult() << endl;

  /* DEBUG */

  //	cout << "m_pout before update  is:" <<m_pout << endl;
  //to get the denominator first sum all cells
  map->universalQuery(sumcells);

  double mapsum = 0;
  mapsum = sumcells.getResult();
  //	cout << "whole map PDF sums to: " << mapsum << endl;
  // then deal with those bloxels that belongs to this cone
  GDMeasUpdateGetDenominator getnormalizer(sensingProb, mapsum);
  map->coneQuery(viewcone.pos[0], viewcone.pos[1], viewcone.pos[2],
      viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10,
      10, isobstacle, getnormalizer, getnormalizer, m_minDistance);
  double normalizer = getnormalizer.getResult() + m_pout;
  //	cout << "normalizer is: " << normalizer << endl;

  GDProbScale scalefunctor(1.0 / normalizer);
  map->universalQuery(scalefunctor, false);

  GDUnsuccessfulMeasUpdate measupdate(normalizer, sensingProb);
  map->coneModifier(viewcone.pos[0], viewcone.pos[1], viewcone.pos[2],
      viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10,
      10, isobstacle, measupdate, measupdate, m_minDistance);

  m_pout = m_pout / normalizer;
  map->universalQuery(sumcells);
  //	cout << "m_pout after update  is:" <<m_pout << endl;
  //	cout << "map sums to: " << sumcells.getResult() << endl;

  normalizePDF(*map, (1 - m_pout));

  //vector< pair<double,double> > thresholds;
  //thresholds.push_back(make_pair(1e-4,1e-3));
  //thresholds.push_back(make_pair(1e-3,5e-3));
  //thresholds.push_back(make_pair(5e-3,1e-2));
  //thresholds.push_back(make_pair(1e-2,5e-2));
  //thresholds.push_back(make_pair(5e-2,1e2));

  //	if(m_usePeekabot)
  //	  pbVis->AddPDF(*map);
  //
  //	map->clearDirty();
}

void VisualObjectSearch::SaveSearchPerformance(std::string result) {

  char buffer[256];
  time_t rawtime;
  struct tm * timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%c", timeinfo);

  //  ofstream out("performancelog.txt",ios_base::app);
  //   out << currentSearchMode << " took " << m_totalViewPoints<< " view points" << ", with result:  " << result << " m_pout: " << m_pout << " " << " time: " << buffer << endl;
  //   out.close();

  std::abort();
}
void VisualObjectSearch::newRobotPose(const cdl::WorkingMemoryChange &objID) {
  try {
    lastRobotPose = getMemoryEntry<NavData::RobotPose2d> (objID.address);
    m_SlamRobotPose.setX(lastRobotPose->x);
    m_SlamRobotPose.setY(lastRobotPose->y);
    m_SlamRobotPose.setTheta(lastRobotPose->theta);

    Cure::Pose3D cp = m_SlamRobotPose;
    m_TOPP.defineTransform(cp);

  } catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }

}

void VisualObjectSearch::receiveOdometry(const Robotbase::Odometry &castOdom) {
  lockComponent(); //Don't allow any interface calls while processing a callback
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f", cureOdom.getX(),
      cureOdom.getY(), cureOdom.getTheta(), cureOdom.getTime().getDouble());
  m_TOPP.addOdometry(cureOdom);
  unlockComponent();
}

void VisualObjectSearch::receiveScan2d(const Laser::Scan2d &castScan) {
  return;
  lockComponent();
  debug("Got scan with n=%d and t=%ld.%06ld", castScan.ranges.size(),
      (long) castScan.time.s, (long) castScan.time.us);

  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);
  if (m_TOPP.isTransformDefined()) {
    Cure::Pose3D scanPose;
    if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
      Cure::Pose3D lpW;
      lpW.add(scanPose, m_LaserPoseR);
      //add tracer
      vector<double> LaserPose;
      LaserPose.push_back(lpW.getX());
      LaserPose.push_back(lpW.getY());
      LaserPose.push_back(lpW.getZ());
      LaserPose.push_back(lpW.getTheta());

      debug("Adding scan..");
      //m_tracer->addScanStationarySensor(castScan,LaserPose,obstacle,makefree,makeobstacle);
      //m_Glrt->addScan(cureScan, lpW,1.0);
    }
  }
  unlockComponent();
}

void VisualObjectSearch::PostNavCommand(Cure::Pose3D position,
    SpatialData::CommandType cmdtype) {
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->prio = SpatialData::URGENT;
  cmd->cmd = cmdtype;
  cmd->pose.resize(3);
  cmd->pose[0] = position.getX();
  cmd->pose[1] = position.getY();
  cmd->pose[2] = position.getTheta();
  cmd->tolerance.resize(1);
  cmd->tolerance[0] = 0.1;
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  new NavCommandReceiver(*this, cmd);
  log("posted nav command");
}

void VisualObjectSearch::addRecognizer3DCommand(
    VisionData::Recognizer3DCommandType cmd, std::string label,
    std::string visualObjectID) {
  log("posting recognizer command: %s.", label.c_str());
  VisionData::Recognizer3DCommandPtr rec_cmd =
      new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  log("constructed visual command, adding to WM");
  addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
  log("added to WM");
}

VisualObjectSearch::NavCommandReceiver::NavCommandReceiver(
    VisualObjectSearch & _component, SpatialData::NavCommandPtr _cmd) :
  m_component(_component), m_cmd(_cmd) {
  m_component.log("received NavCommandReceiver notification");
  string id(m_component.newDataID());
  m_component.log("ID post: %s", id.c_str());

  m_component.addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);
  m_component.addToWorkingMemory<SpatialData::NavCommand> (id, m_cmd);

}
void VisualObjectSearch::owtNavCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    if (isSearchFinished)
      return;
    log("nav command overwritten");
    SpatialData::NavCommandPtr cmd(getMemoryEntry<SpatialData::NavCommand> (
        objID.address));
    if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
      // if we were at the first policy step we just moved to a room
      if (currentPolicyStep == 0 && !m_publishSimCones) {
        log("arrived at the room");
        m_currentRoom = m_policyRoom;
        ChangeMaps( currentSearchPolicy[currentPolicyStep]);
        currentPolicyStep++;
        m_command = ASK_FOR_DISTRIBUTION;
        return;
      }
      log("NavCommand succeeded.");
      m_totalViewPoints++;
      if (m_publishSimCones) {
        MovePanTilt(0.0, m_tilt, 0.08);
        Recognize();
      } else {
        MovePanTilt(0.0, m_nbv.tilt, 0.08);
        m_command = RECOGNIZE;
      }
    } else if (cmd->comp == SpatialData::COMMANDFAILED) {
      log("NavCommand failed.Letting planner know.");
      log("Letting planner know that view point is processed.");
      SpatialData::ProcessViewPointCommandPtr VPcmd =
          new SpatialData::ProcessViewPointCommand;
      VPcmd->status = SpatialData::SUCCESS;
      overwriteWorkingMemory(m_ProcessVPID, VPcmd);
      //   if(currentPolicyStep == 0){
      // we could not reach to room try again
      //   m_command = GOTOROOM;
      // }
      // else{
      //   m_command=NEXT_NBV;
      // }
    } else if (cmd->comp == SpatialData::COMMANDPENDING) {
      log("NavCommand pending.");
    } else if (cmd->comp == SpatialData::COMMANDINPROGRESS) {
      log("NavCommand in progress");
    } else if (cmd->comp == SpatialData::COMMANDABORTED) {
      log("NavCommand aborted.");
    }
  } catch (const CASTException &e) {
    //log("failed to delete SpatialDataCommand: %s", e.message.c_str());
    log("CASTException in VisualObjectSearch::owtNavCommand");
  }
}

void VisualObjectSearch::NavCommandReceiver::workingMemoryChanged(
    const cast::cdl::WorkingMemoryChange &_wmc) {
  m_component.log("received inner notification");
  try {
    m_component.owtNavCommand(_wmc);
  } catch (const CASTException &e) {
    //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
  }

}

SpatialData::NavCommandPtr VisualObjectSearch::newNavCommand() {
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->prio = SpatialData::NORMAL;
  cmd->cmd = SpatialData::STOP;
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  return cmd;
}

}
