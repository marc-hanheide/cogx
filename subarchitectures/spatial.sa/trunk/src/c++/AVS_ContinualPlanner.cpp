/*
 * AVS_ContinualPlanner.cpp
 *
 *  Created on: Mar 1, 2011
 *      Author: alper
 *
 *      TODO
 *      1. Create bloxel maps for each object in generateViewCones
 *      2. in processConeGroup update each of these maps with according to cone_depth
 *      3. During ViewConeUpdate put the ObjectSearchResult for all objects there
 *      4. When a new object belief arrives place ObjectPlaceProperty in WM -- Done
 *
 */

#include <CureHWUtils.hpp>
#include <AddressBank/ConfigFileReader.hh>

#include "AVS_ContinualPlanner.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cast/core/CASTUtils.hpp>
#include "ComaData.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include "GridDataFunctors.hh"

#include "PBVisualization.hh"

#include <algorithm>

using namespace cast;
using namespace std;
using namespace boost;
using namespace SpatialGridMap;
using namespace cogx;
using namespace Math;
using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::history;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace de::dfki::lt::tr::beliefs::slice::distribs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;
using namespace eu::cogx::beliefs::slice;

namespace spatial {

extern "C" {
cast::CASTComponentPtr newComponent() {
  return new AVS_ContinualPlanner();
}
}

AVS_ContinualPlanner::AVS_ContinualPlanner() :
  m_sampler(&m_relationEvaluator) {
  // TODO Auto-generated constructor stub
  m_sensingProb = 1.0;
  m_defaultBloxelCell.pdf = 0;
  m_defaultBloxelCell.occupancy = SpatialGridMap::UNKNOWN;
  m_gotPC = false;
  m_gotNewGenerateViewCone = false;
  m_currentVPGenerationCommand
      = new SpatialData::RelationalViewPointGenerationCommand;
  m_currentConeGroup = 0;

  m_ptzWaitingStatus = NO_WAITING;
  m_waitingForPTZCommandID = "";
}

AVS_ContinualPlanner::~AVS_ContinualPlanner() {
  // TODO Auto-generated destructor stub
}

void AVS_ContinualPlanner::createFOV(peekabot::GroupProxy &proxy,
    peekabot::PolygonProxy* &proxyConeParts, double fovHorizAngle,
    double fovVertiAngle, double* color, double opacity,
    NavData::ViewPoint viewpoint) {

  const double coneLen = viewpoint.length;
  // The "half angle" of the field of view
  const double fovHoriz = fovHorizAngle / 2;
  const double fovVerti = fovVertiAngle / 2;

  proxyConeParts[0].add(proxy, "top");
  peekabot::VertexSet vs1;
  vs1.add(0, 0, 0);
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[0].add_vertices(vs1);
  vs1.clear();

  proxyConeParts[1].add(proxy, "bottom");
  vs1.add(0, 0, 0);
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  proxyConeParts[1].add_vertices(vs1);
  vs1.clear();

  proxyConeParts[2].add(proxy, "left");
  vs1.add(0, 0, 0);
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[2].add_vertices(vs1);
  vs1.clear();

  proxyConeParts[3].add(proxy, "right");
  vs1.add(0, 0, 0);
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[3].add_vertices(vs1);
  vs1.clear();

  proxyConeParts[4].add(proxy, "image");
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  vs1.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  vs1.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[4].add_vertices(vs1);

  for (int i = 0; i < 5; i++) {
    proxyConeParts[i].set_color(color[0], color[1], color[2]);
    proxyConeParts[i].set_opacity(opacity);
    proxyConeParts[i].set_scale(1); // This is how I make the cone
    // larger or smaller

  }
  proxy.rotate(viewpoint.pan, 0, 0, 1);
  proxy.rotate(viewpoint.tilt, 0, -1, 0);
  proxy.set_position(viewpoint.pos.x, viewpoint.pos.y, viewpoint.pos.z);
  m_PeekabotClient.sync();
}

void AVS_ContinualPlanner::ChangeCurrentViewConeColor(double r, double g,
    double b) {
  for (int i = 0; i < 5; i++)
    m_proxyConePolygons[i].set_color(r, g, b);
}

void AVS_ContinualPlanner::connectPeekabot() {
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

    if (m_usePeekabot) {
      m_ProxyViewPoints.add(m_PeekabotClient, "planned_viewpoints",
          peekabot::REPLACE_ON_CONFLICT);
    }

    log("Connection to Peekabot established");

  } catch (std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)", e.what());
    return;
  }
}

void AVS_ContinualPlanner::runComponent() {
  log("I am running");

  while (isRunning()) {

    if (m_ptzWaitingStatus != NO_WAITING && m_waitingForPTZCommandID == "") {

      if (m_ptzWaitingStatus == WAITING_TO_RECOGNIZE) {
        Recognize();
      }

      else if (m_ptzWaitingStatus == WAITING_TO_RETURN) {
        m_currentProcessConeGroup->status = SpatialData::SUCCESS;
        log("Overwriting command to change status to: SUCCESS");
        overwriteWorkingMemory<SpatialData::ProcessConeGroup> (
            m_processConeGroupCommandWMAddress, m_currentProcessConeGroup);
      }

      m_ptzWaitingStatus = NO_WAITING;

    }

    if (m_gotNewGenerateViewCone) {
      generateViewCones(m_currentVPGenerationCommand,
          m_generateViewConesCommandWMAddress);

    }
    sleepComponent(500);
  }
}

void AVS_ContinualPlanner::owtWeightedPointCloud(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    log("got weighted PC");
    FrontierInterface::ObjectPriorRequestPtr req = getMemoryEntry<
        FrontierInterface::ObjectPriorRequest> (objID.address);
    FrontierInterface::WeightedPointCloudPtr cloud = req->outCloud;

    m_cloud = req->outCloud;
    receivePointCloud(cloud, req->totalMass);
  } catch (DoesNotExistOnWMException excp) {
    log("Error!  WeightedPointCloud does not exist on WM!");
    return;
  }
}

void AVS_ContinualPlanner::receivePointCloud(
    FrontierInterface::WeightedPointCloudPtr cloud, double totalMass) {
  log("KDEing cloud read from file");
  vector<Vector3> centers;
  centers.push_back(cloud->center);

  if (cloud->isBaseObjectKnown) {
    log("Got distribution around known object pose");
    m_sampler.kernelDensityEstimation3D(*m_currentBloxelMap, centers,
        cloud->interval, cloud->xExtent, cloud->yExtent, cloud->zExtent,
        cloud->values, 1.0, totalMass, m_currentCureObstMap);
    normalizePDF(*m_currentBloxelMap, totalMass);
  } else {
    log(
        "Base object is not known! This means support object is not known to ObjectRelationManager. Something is very wrong!");
  }

  m_currentBloxelMap->clearDirty();
  m_gotPC = true;
}

void AVS_ContinualPlanner::start() {
  //Todo subscribe to View Cone Generation
  if (m_usePeekabot) {
    while (!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)) {
      sleepComponent( m_RetryDelay);
      connectPeekabot();
    }
    peekabot::ObjectProxy m_grid_map;
    peekabot::ObjectProxy m_frontiers;
    peekabot::ObjectProxy m_pdf;
    peekabot::ObjectProxy m_2DOccGridProxy;
    peekabot::Status s;
    s = m_grid_map.assign(m_PeekabotClient, "grid_map").status();
    if (s.succeeded())
      m_grid_map.remove();
    s = m_frontiers.assign(m_PeekabotClient, "frontiers").status();
    if (s.succeeded())
      m_frontiers.remove();
    s = m_pdf.assign(m_PeekabotClient, "pdf").status();
    if (s.succeeded())
      m_pdf.remove();
    s
        = m_2DOccGridProxy.assign(m_PeekabotClient, "combined_placemap2D").status();
    if (s.succeeded())
      m_2DOccGridProxy.remove();
    s = m_pdf.assign(m_PeekabotClient, "pdf1").status();
    if (s.succeeded())
      m_pdf.remove();
    s
        = m_2DOccGridProxy.assign(m_PeekabotClient, "combined_placemap2D1").status();
    if (s.succeeded())
      m_2DOccGridProxy.remove();
    s = m_pdf.assign(m_PeekabotClient, "pdf2").status();
    if (s.succeeded())
      m_pdf.remove();
    s
        = m_2DOccGridProxy.assign(m_PeekabotClient, "combined_placemap2D2").status();
    if (s.succeeded())
      m_2DOccGridProxy.remove();
    s = m_pdf.assign(m_PeekabotClient, "pdf3").status();
    if (s.succeeded())
      m_pdf.remove();
    s
        = m_2DOccGridProxy.assign(m_PeekabotClient, "combined_placemap2D3").status();
    if (s.succeeded())
      m_2DOccGridProxy.remove();

    m_ProxyForbiddenMap.add(m_PeekabotClient, "avs_forbidden",
        peekabot::REPLACE_ON_CONFLICT);
    m_ProxyForbiddenMap.set_position(0, 0, -0.005);
    int i = 0;
    for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin(); fbIt
        != m_forbiddenZones.end(); fbIt++) {
      peekabot::PolygonProxy* p = new peekabot::PolygonProxy();

      p->add(m_ProxyForbiddenMap, "zone");
      p->set_color(1, 0.1, 0.1);
      peekabot::VertexSet vs;
      vs.add(fbIt->minX < -30 ? -30 : fbIt->minX, fbIt->minY < -30 ? -30
          : fbIt->minY, i * 0.001);
      vs.add(fbIt->minX < -30 ? -30 : fbIt->minX, fbIt->maxY > 30 ? 30
          : fbIt->maxY, i * 0.001);
      vs.add(fbIt->maxX > 30 ? 30 : fbIt->maxX, fbIt->maxY > 30 ? 30
          : fbIt->maxY, i * 0.001);
      vs.add(fbIt->maxX > 30 ? 30 : fbIt->maxX, fbIt->minY < -30 ? -30
          : fbIt->minY, i * 0.001);
      p->add_vertices(vs);
      i++;
    }

  }
  addChangeFilter(createGlobalTypeFilter<
      SpatialData::RelationalViewPointGenerationCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
          &AVS_ContinualPlanner::newViewPointGenerationCommand));

  addChangeFilter(createGlobalTypeFilter<SpatialData::ProcessConeGroup> (
      cdl::ADD), new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
      &AVS_ContinualPlanner::newProcessConeCommand));

  addChangeFilter(
      createGlobalTypeFilter<FrontierInterface::ObjectPriorRequest> (
          cdl::OVERWRITE), new MemberFunctionChangeReceiver<
          AVS_ContinualPlanner> (this,
          &AVS_ContinualPlanner::owtWeightedPointCloud));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD),
      new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
          &AVS_ContinualPlanner::newRobotPose));

  addChangeFilter(createLocalTypeFilter<SpatialData::SpatialObject> (cdl::ADD),
      new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
          &AVS_ContinualPlanner::newSpatialObject));

  addChangeFilter(cast::createGlobalTypeFilter<GroundedBelief>(cast::cdl::ADD),
      new cast::MemberFunctionChangeReceiver<AVS_ContinualPlanner>(this,
          &AVS_ContinualPlanner::newGroundedBelief));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (this,
          &AVS_ContinualPlanner::newRobotPose));

  addChangeFilter(createGlobalTypeFilter<ptz::SetPTZPoseCommand> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (
      this, &AVS_ContinualPlanner::overwrittenPanTiltCommand));

  addChangeFilter(createGlobalTypeFilter<VisionData::ARTagCommand> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (
      this, &AVS_ContinualPlanner::owtARTagCommand));

  addChangeFilter(createGlobalTypeFilter<VisionData::Recognizer3DCommand> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<AVS_ContinualPlanner> (
      this, &AVS_ContinualPlanner::owtRecognizer3DCommand));
  log("getting ice queryHandlerServer");
  m_queryHandlerServerInterfacePrx = getIceServer<
      ConceptualData::QueryHandlerServerInterface> (m_queryHandlerName);
}

void AVS_ContinualPlanner::owtARTagCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  if (m_currentConeGroup) {
    VisionData::ARTagCommandPtr newObj = getMemoryEntry<
        VisionData::ARTagCommand> (objID.address);
    log("Overwritten ARTagCommand: %s (%s)", newObj->label.c_str(),
        objID.address.id.c_str());
    ViewConeUpdate(m_currentViewCone,
        m_objectBloxelMaps[m_currentConeGroup->bloxelMapId]);

    m_currentViewConeNumber++;
    if (m_currentViewConeNumber >= m_currentConeGroup->viewcones.size()) {
      startMovePanTilt(0.0, 0.0, 0.08);
      m_ptzWaitingStatus = WAITING_TO_RETURN;
    } else {
      m_processedViewConeIDs.insert(m_currentViewCone.first);
      m_currentViewCone.first = m_currentConeGroupNumber * 1000
          + m_currentViewConeNumber;
      m_currentViewCone.second
          = m_currentConeGroup->viewcones[m_currentViewConeNumber];
      if (m_usePeekabot) {
        m_proxyCone = &m_ProxyViewPointsList[m_currentViewCone.first];
        m_proxyConePolygons
            = m_ProxyViewPointsPolygonsList[m_currentViewCone.first];
        ChangeCurrentViewConeColor(0.1, 0.9, 0.1);
      }
      double angle = m_currentViewCone.second.pan - lastRobotPose->theta;
      if (angle < -M_PI)
        angle += 2 * M_PI;
      if (angle > M_PI)
        angle -= 2 * M_PI;

      startMovePanTilt(angle, m_currentViewCone.second.tilt, 0.08);
      m_ptzWaitingStatus = WAITING_TO_RECOGNIZE;
    }

    //	m_currentProcessConeGroup->status = SpatialData::SUCCESS;
    //	log("Overwriting command to change status to: SUCCESS");
    //	overwriteWorkingMemory<SpatialData::ProcessConeGroup>(m_processConeGroupCommandWMAddress , m_currentProcessConeGroup);
  }
}

void AVS_ContinualPlanner::owtRecognizer3DCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("entered owtRecognizer3DCommand");
  if (m_currentConeGroup) {
    VisionData::Recognizer3DCommandPtr newObj = getMemoryEntry<
        VisionData::Recognizer3DCommand> (objID.address);
    log("Overwritten Recognizer3D Command: %s (%s)", newObj->label.c_str(),
        objID.address.id.c_str());
    ViewConeUpdate(m_currentViewCone,
        m_objectBloxelMaps[m_currentConeGroup->bloxelMapId]);
    log("finished ViewConeUpdate");
    m_currentViewConeNumber++;
    if (m_currentViewConeNumber >= m_currentConeGroup->viewcones.size()) {
      startMovePanTilt(0.0, 0.0, 0.08);
      m_ptzWaitingStatus = WAITING_TO_RETURN;
    } else {
      m_processedViewConeIDs.insert(m_currentViewCone.first);
      m_currentViewCone.first = m_currentConeGroupNumber * 1000
          + m_currentViewConeNumber;
      m_currentViewCone.second
          = m_currentConeGroup->viewcones[m_currentViewConeNumber];
      if (m_usePeekabot) {
        m_proxyCone = &m_ProxyViewPointsList[m_currentViewCone.first];
        m_proxyConePolygons
            = m_ProxyViewPointsPolygonsList[m_currentViewCone.first];
        ChangeCurrentViewConeColor(0.1, 0.9, 0.1);
      }
      double angle = m_currentViewCone.second.pan - lastRobotPose->theta;
      if (angle < -M_PI)
        angle += 2 * M_PI;
      if (angle > M_PI)
        angle -= 2 * M_PI;

      startMovePanTilt(angle, m_currentViewCone.second.tilt, 0.08);
      m_ptzWaitingStatus = WAITING_TO_RECOGNIZE;
    }

    //	m_currentProcessConeGroup->status = SpatialData::SUCCESS;
    //	log("Overwriting command to change status to: SUCCESS");
    //	overwriteWorkingMemory<SpatialData::ProcessConeGroup>(m_processConeGroupCommandWMAddress , m_currentProcessConeGroup);
  }
  log("exited owtRecognizer3DCommand");
}

void AVS_ContinualPlanner::newSpatialObject(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    SpatialData::SpatialObjectPtr newObj = getMemoryEntry<
        SpatialData::SpatialObject> (objID.address);

    spatial::Object *model = generateNewObjectModel(newObj->label);
    log("Got Spatial Object: %s", newObj->label.c_str());
    model->pose = newObj->pose;
  } catch (DoesNotExistOnWMException e) {
    log("Error! SpatialObject disappeared from WM!");
  }

}

void AVS_ContinualPlanner::newProcessConeCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Got Process Cone Group Command %s", objID.address.id.c_str());
  SpatialData::ProcessConeGroupPtr cmd = getMemoryEntry<
      SpatialData::ProcessConeGroup> (objID.address);
  m_processConeGroupCommandWMAddress = objID.address.id;
  m_currentProcessConeGroup = cmd;
  processConeGroup(cmd->coneId);
}

void AVS_ContinualPlanner::newGroundedBelief(
    const cast::cdl::WorkingMemoryChange &objID) {
  // If this is an visualobject belief then add as an ObjectPlaceProperty
  // Based on the current ConeGroup know the location of the object
  try {
    dBeliefPtr belief = new dBelief;
    belief = getMemoryEntry<dBelief> (objID.address);

    if (belief->type == "visualobject") {
      CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(
          belief->content));
      if (dist->distribs.count("label") != 0) {
        // Let's get it's Id;
        log("Got a new Visual Object!");
        log("getting percept WMid for this grounded belief");
        CASTBeliefHistoryPtr history(CASTBeliefHistoryPtr::dynamicCast(
            belief->hist));

        log(
            "getting percept belief associated with this grounded object at: %s",
            history->ancestors[0]->address.id.c_str());

        PerceptBeliefPtr visualpercept = new PerceptBelief;
        visualpercept = getMemoryEntry<PerceptBelief> (
            history->ancestors[0]->address);
        CASTBeliefHistoryPtr history2(CASTBeliefHistoryPtr::dynamicCast(
            visualpercept->hist));
        std::string visualobjectid = history2->ancestors[0]->address.id;

        BasicProbDistributionPtr basicdist(
            BasicProbDistributionPtr::dynamicCast(dist->distribs["label"]));
        FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(
            basicdist->values));
        ElementaryFormulaPtr elformula(ElementaryFormulaPtr::dynamicCast(
            formulaValues->values[0].val));
        log("Visual Object Id: %s", elformula->prop.c_str());

        //m_fromBeliefIdtoVisualLabel[objID.address.id] = elformula->prop.c_str();

        m_fromBeliefIdtoVisualLabel[objID.address.id] = visualobjectid;

        SpatialProperties::ObjectPlacePropertyPtr result =
            new SpatialProperties::ObjectPlaceProperty;
        result->category = m_currentConeGroup->searchedObjectCategory;
        result->relation = m_currentConeGroup->relation;
        result->supportObjectCategory
            = m_currentConeGroup->supportObjectCategory;
        result->supportObjectId = m_currentConeGroup->supportObjectId;
        result->placeId = m_currentConeGroup->placeId;

        SpatialProperties::DiscreteProbabilityDistributionPtr spadist =
            new SpatialProperties::DiscreteProbabilityDistribution;
        SpatialProperties::ValueProbabilityPair pair1, pair2;
        SpatialProperties::BinaryValuePtr binval1 =
            new SpatialProperties::BinaryValue;
        SpatialProperties::BinaryValuePtr binval2 =
            new SpatialProperties::BinaryValue;
        binval1->value = true;
        binval2->value = false;
        pair1.value = binval1;
        pair1.probability = 1.0;
        pair2.value = binval2;
        pair2.probability = 0;

        spadist->data.push_back(pair1);
        spadist->data.push_back(pair2);
        result->distribution = spadist;

        //result->distribution
        log(
            "Publishing ObjectPlaceProperty with: category: %s, relation: %s, supportObjectCategory: %s, supportObjectId: %s",
            result->category.c_str(),
            relationToString(result->relation).c_str(),
            result->supportObjectCategory.c_str(),
            result->supportObjectId.c_str());

        addToWorkingMemory(newDataID(), result);
      } else {
        log("Empty VisualObject belief!");
      }
    }
  }

  catch (DoesNotExistOnWMException) {
    log("Belief disappeared from WM!");
    return;
  }
}
void AVS_ContinualPlanner::newViewPointGenerationCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  try {
    log("got new GenerateViewPointCommand");
    SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand =
        getMemoryEntry<SpatialData::RelationalViewPointGenerationCommand> (
            objID.address);
    SpatialData::RelationalViewPointGenerationCommandPtr cmd =
        new SpatialData::RelationalViewPointGenerationCommand(*newVPCommand);

    m_currentVPGenerationCommand = cmd;
    m_gotNewGenerateViewCone = true;
    m_generateViewConesCommandWMAddress = objID.address.id;

    //
  } catch (const CASTException &e) {
    log("owtRecognizer3DCommand disappeared from WM: %s", e.message.c_str());

  }
}

/* Generate view cones for <object,relation , object/room, room> */
void AVS_ContinualPlanner::generateViewCones(
    SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand,
    std::string WMAddress) {

  m_gotNewGenerateViewCone = false;

  string plus = "p(+";
  string closebracket = ")";
  string id = plus + m_namegenerator.getUnexploredObjectVarName(
      newVPCommand->roomId, newVPCommand->searchedObjectCategory,
      newVPCommand->relation, newVPCommand->supportObjectCategory,
      newVPCommand->supportObject) + closebracket;

  log("Generating View Cones for %s", id.c_str());

  SpatialData::MapInterfacePrx mapPrx(getIceServer<SpatialData::MapInterface> (
      "spatial.control"));

  SpatialData::HeightMap KH = mapPrx->getHeightMap();

  Cure::LocalGridMap<double> lgmKH(KH.size, KH.cellSize, FLT_MAX,
      Cure::LocalGridMap<double>::MAP1, KH.xCenter, KH.yCenter);

  // Convert from SpatialData::LocalGridMap to Cure::LocalGridMap
  int lp = 0;
  for (int x = -KH.size; x <= KH.size; x++) {
    for (int y = -KH.size; y <= KH.size; y++) {
      lgmKH(x, y) = (KH.data[lp]);
      lp++;
    }
  }

  // if we already don't have a room map for this then get the combined map
  if (m_templateRoomBloxelMaps.count(newVPCommand->roomId) == 0) {
    log("Creating a new BloxelMap for room: %d", newVPCommand->roomId);
    SpatialData::LocalGridMap combined_lgm;

    /*
     SpatialData::LocalGridMapMap grid = mapPrx->getGridMap();

     Cure::LocalGridMap<unsigned char>* lgm = new Cure::LocalGridMap<unsigned char>(grid.size, grid.cellSize, '2', Cure::LocalGridMap<unsigned char>::MAP1, grid.xCenter, grid.yCenter);

     // Convert from SpatialData::LocalGridMap to Cure::LocalGridMap
     int lp1 = 0;
     for(int x = -grid.size ; x <= grid.size; x++){
     for(int y = -grid.size ; y <= grid.size; y++){ 
     lgm(x,y) = (grid.data[lp1]);
     lp1++;
     }
     }
     */

    vector<comadata::ComaRoomPtr> comarooms;
    getMemoryEntries<comadata::ComaRoom> (comarooms, "coma");

    log("Got %d rooms", comarooms.size());

    if (comarooms.size() == 0) {
      log("No such ComaRoom with id %d! Returning", newVPCommand->roomId);
      return;
    }
    int comarooms_i = -1;
    for (size_t i = 0; i < comarooms.size(); i++) {
      log("Got coma room with room id: %d", comarooms[i]->roomId);
      if (comarooms[i]->roomId == newVPCommand->roomId) {
        comarooms_i = i;
        break;
      }
    }
    if (comarooms_i == -1) {
      error("no comaroom");
      return;
    }
    for (size_t j = 0; j < comarooms[comarooms_i]->containedPlaceIds.size(); j++) {
      log("getting room which contains, placeid: %d",
          comarooms[comarooms_i]->containedPlaceIds[j]);
    }

    /* Remove all free space and obstacle which does not belong to this room
     * This is to avoid spillage of metric space from other rooms
     * */
    cout << "Removing all free space not belongin to this room" << endl;

    log(
        "Throwing away all known space in LGMap of this room belonging to another room");
    FrontierInterface::PlaceInterfacePrx agg(getIceServer<
        FrontierInterface::PlaceInterface> ("place.manager"));
    log("got interface");

    SpatialData::PlaceIDSeq currentRoomPlaceIds;

    double xW, yW;

    vector<SpatialData::PlacePtr> placesInMap;
    getMemoryEntries<SpatialData::Place> (placesInMap, "spatial.sa");

    std::vector<NavData::AEdgePtr> edges;
    getMemoryEntries<NavData::AEdge> (edges);

    vector<NavData::FNodePtr> nodesForPlaces;
    for (unsigned int i = 0; i < placesInMap.size(); i++) {
      NavData::FNodePtr node = agg->getNodeFromPlaceID(placesInMap[i]->id);
      nodesForPlaces.push_back(node);
    }

    for (unsigned int j = 0; j
        < comarooms[comarooms_i]->containedPlaceIds.size(); j++) {
      currentRoomPlaceIds.push_back(
          comarooms[comarooms_i]->containedPlaceIds[j]);
      NavData::FNodePtr node = agg->getNodeFromPlaceID(
          comarooms[comarooms_i]->containedPlaceIds[j]);
      m_roomNodes[newVPCommand->roomId].push_back(node);
      for (std::vector<NavData::AEdgePtr>::iterator it = edges.begin(); it
          != edges.end(); ++it) {

        NavData::FNodePtr node1 = agg->getNodeFromPlaceID(
            agg->getPlaceFromNodeID((*it)->startNodeId)->id);
        NavData::FNodePtr node2 = agg->getNodeFromPlaceID(
            agg->getPlaceFromNodeID((*it)->endNodeId)->id);

        if ((node1->gateway == 1) && (node2->nodeId == node->nodeId)) {
          m_roomNodes[newVPCommand->roomId].push_back(node1);
          currentRoomPlaceIds.push_back(
              agg->getPlaceFromNodeID(node1->nodeId)->id);
          break;
        } else if ((node2->gateway == 1) && (node1->nodeId == node->nodeId)) {
          m_roomNodes[newVPCommand->roomId].push_back(node2);
          currentRoomPlaceIds.push_back(
              agg->getPlaceFromNodeID(node2->nodeId)->id);
          break;
        }
      }
    }

    FrontierInterface::LocalMapInterfacePrx agg2(getIceServer<
        FrontierInterface::LocalMapInterface> ("map.manager"));
    combined_lgm = agg2->getCombinedGridMap(currentRoomPlaceIds);

    m_templateRoomBloxelMaps[newVPCommand->roomId]
        = new SpatialGridMap::GridMap<GridMapData>(combined_lgm.size * 2 + 1,
            combined_lgm.size * 2 + 1, m_cellsize, m_minbloxel, 0, 2.0,
            combined_lgm.xCenter, combined_lgm.yCenter, 0, m_defaultBloxelCell);

    //convert 2D map to 3D
    CureObstMap* lgm = new CureObstMap(combined_lgm.size, m_cellsize, '2',
        CureObstMap::MAP1, combined_lgm.xCenter, combined_lgm.yCenter);
    IcetoCureLGM(combined_lgm, lgm);

    for (int x = -lgm->getSize(); x < lgm->getSize(); x++) {
      for (int y = -lgm->getSize(); y < lgm->getSize(); y++) {
        if ((*lgm)(x, y) != '2') {
          lgm->index2WorldCoords(x, y, xW, yW);
          double minDistance = FLT_MAX;
          unsigned int closestNodeIdx = 0;
          bool excluded = false;
          for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin(); fbIt
              != m_forbiddenZones.end(); fbIt++) {
            //    	  log("checking forbidden zone: %.02g, %.02g, %.02g, %.02g,", fbIt->minX, fbIt->minY, fbIt->maxX, fbIt->maxY);
            //    	  log("checking against: %.02g, %.02g", newX, newY);
            double dx, dy;
            (*lgm).index2WorldCoords(x, y, dx, dy);
            if ((dx <= fbIt->maxX && dx >= fbIt->minX && dy <= fbIt->maxY && dy
                >= fbIt->minY)) {
              log("point in forbidden zone excluded");
              (*lgm)(x, y) = '2';
              excluded = true;
              break;
            }
          }
          if (excluded)
            continue;

          for (unsigned int i = 0; i < nodesForPlaces.size(); i++) {
            try {
              if ((nodesForPlaces[i] != 0) && (nodesForPlaces[i]->gateway == 0)) {
                double nX = nodesForPlaces[i]->x;
                double nY = nodesForPlaces[i]->y;

                double distance = (xW - nX) * (xW - nX) + (yW - nY) * (yW - nY);
                if (distance < minDistance) {
                  closestNodeIdx = i;
                  minDistance = distance;
                }
              }
            } catch (IceUtil::NullHandleException e) {
              log("Error! FNode suddenly disappeared!");
            }
          }

          SpatialData::PlacePtr closestPlace = placesInMap[closestNodeIdx];

          //					placemem = agg->getPlaceMembership(xW,yW);

          if (find(currentRoomPlaceIds.begin(), currentRoomPlaceIds.end(),
              closestPlace->id) == currentRoomPlaceIds.end()) {
            (*lgm)(x, y) = '2';
          }
        }
      }
    }
    log("removed");
    cout << " done " << endl;

    m_templateRoomGridMaps[newVPCommand->roomId] = lgm;
    GDMakeObstacle makeobstacle;
    for (int x = -combined_lgm.size; x < combined_lgm.size; x++) {
      for (int y = -combined_lgm.size; y < combined_lgm.size; y++) {
        if ((*lgm)(x, y) == '1') {
          double dx, dy;
          (*lgm).index2WorldCoords(x, y, dx, dy);
          int nx, ny;
          if (lgmKH.worldCoords2Index(dx, dy, nx, ny) == 0) {
            if (lgmKH(nx, ny) != FLT_MAX) {
              if (lgmKH(nx, ny) - 0.1 > 0)
                m_templateRoomBloxelMaps[newVPCommand->roomId]->boxSubColumnModifier(
                    x + combined_lgm.size, y + combined_lgm.size, lgmKH(nx, ny)
                        / 2, lgmKH(nx, ny) - 0.1, makeobstacle);
            } else {
              m_templateRoomBloxelMaps[newVPCommand->roomId]->boxSubColumnModifier(
                  x + combined_lgm.size, y + combined_lgm.size,
                  m_LaserPoseR.getZ(), m_minbloxel, makeobstacle);
              //m_LaserPoseR.getZ()+0.775,  m_mapceiling, makeobstacle);
            }
          }
        }
      }
    }
  } else {
    log("Already have this bloxel map");
  }

  // FIXME !!! This check should be done for all objects that we can find !!!


  bool alreadyGenerated = false;

  if (m_objectBloxelMaps.count(id) > 0) {
    log("A bloxel map for this location is already generated.");
    alreadyGenerated = true;
  }
  // if we already have a bloxel map for configuration
  // else create a new one
  if (!alreadyGenerated) {
    log("A new bloxelmap for this location is being created.");
    //Since we don't have this location currently, first initialize a bloxel map for it from the template room map
    m_objectBloxelMaps[id] = new SpatialGridMap::GridMap<GridMapData>(
        *m_templateRoomBloxelMaps[newVPCommand->roomId]);
    // set so far explored bit to zero;
    m_locationToBeta[id] = 0;
  }

  if (m_usePeekabot) {
    //log("Displaying Map in PB.");
    //pbVis->DisplayMap(*m_objectBloxelMaps[id]);
  }
  m_currentBloxelMap = m_objectBloxelMaps[id];
  m_currentCureObstMap = m_templateRoomGridMaps[newVPCommand->roomId];

  // now we have our room map let's fill it
  //Query Conceptual to learn the initial pdf values
  log("Querying for %s", id.c_str());
  ConceptualData::ProbabilityDistributions conceptualProbdist =
      m_queryHandlerServerInterfacePrx->query(id);
  SpatialProbabilities::ProbabilityDistribution probdist =
      conceptualProbdist[0];

  if (probdist.massFunction.size() == 0) {
    log("Got an empty distribution!");
  }
  double pdfmass = probdist.massFunction[0].probability;
  log("Got probability for %s Conceptual %f", id.c_str(), pdfmass);

  m_locationToInitialPdfmass[id] = pdfmass;

  //Todo: Generate viewpoints on the selected room's bloxel map and for a given pdf id
  if (newVPCommand->relation != SpatialData::INROOM && !alreadyGenerated) {
    log("Searching with a support object");
    // indirect search this must be known object
    // Ask for the cloud

    // Construct Request object
    std::vector<std::string> labels;
    vector<Vector3> centers;
    std::vector<FrontierInterface::ObjectRelation> relation;
    if (m_fromBeliefIdtoVisualLabel.count(newVPCommand->supportObject) == 0) {
      log("No such visual object! I am not going to do anything!");
      log("The ones we know are:");
      map<string, string>::const_iterator end =
          m_fromBeliefIdtoVisualLabel.end();
      for (map<string, string>::const_iterator it =
          m_fromBeliefIdtoVisualLabel.begin(); it != end; ++it) {
        log("beliefid: %s", it->first.c_str());
        log("visualid: %s", it->second.c_str());
      }
      return;
    }

    labels.push_back(newVPCommand->searchedObjectCategory);
    labels.push_back(m_fromBeliefIdtoVisualLabel[newVPCommand->supportObject]);

    for (unsigned int j = 0; j < labels.size(); j++) {
      log("ObjectPriorRequest for: %s", labels[j].c_str());
    }
    relation.push_back(
        (newVPCommand->relation == SpatialData::INOBJECT ? FrontierInterface::IN
            : FrontierInterface::ON));

    FrontierInterface::WeightedPointCloudPtr queryCloud =
        new FrontierInterface::WeightedPointCloud;
    FrontierInterface::ObjectPriorRequestPtr objreq =
        new FrontierInterface::ObjectPriorRequest;
    objreq->relationTypes = relation; // ON or IN or whatnot
    objreq->objects = labels; // Names of objects, starting with the query object
    objreq->cellSize = m_cellsize; // CelGOTONODEll size of map (affects spacing of samples)
    objreq->outCloud = queryCloud; // Data struct to receive output
    objreq->totalMass = 1.0; // we will normalize anyways
    //wait until we get the cloud back
    {
      log("Asking for ObjectPriorRequest");
      m_gotPC = false;
      addToWorkingMemory(newDataID(), objreq);
      log("ObjectPriorRequest added to WM");
      while (!m_gotPC)
        usleep(2500);
      log("got PC for direct search");
    }

    if (m_cloud->isBaseObjectKnown) {
      log("Got distribution around known object pose");
      m_sampler.kernelDensityEstimation3D(*m_objectBloxelMaps[id], centers,
          m_cloud->interval, m_cloud->xExtent, m_cloud->yExtent,
          m_cloud->zExtent, m_cloud->values, 1.0, pdfmass,
          m_templateRoomGridMaps[newVPCommand->roomId]);
      normalizePDF(*m_objectBloxelMaps[id], pdfmass);
    }
    m_conedepth = 2; //FIXME

  } else if (newVPCommand->relation == SpatialData::INROOM && !alreadyGenerated) {
    log("Searching in the room, assuming uniform probability");
    // uniform over the room
    GDProbSet resetter(0.0);
    m_objectBloxelMaps[id]->universalQuery(resetter, true);
    double fixedpdfvalue = pdfmass
        / (m_objectBloxelMaps[id]->getZBounds().second
            - m_objectBloxelMaps[id]->getZBounds().first);

    GDProbInit initfunctor(fixedpdfvalue / 25);
    GDProbInit initfunctor2(fixedpdfvalue);
    log(
        "Setting each bloxel near an obstacle to a fixed value of %f, in total: %f",
        fixedpdfvalue, pdfmass);
    pair<double, double> insideroom;
    insideroom.first = 0;
    insideroom.second = 0;

    CureObstMap* lgm = m_templateRoomGridMaps[newVPCommand->roomId];
    for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
      for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
        int bloxelX = x + lgm->getSize();
        int bloxelY = y + lgm->getSize();

        if ((*lgm)(x, y) == '1') {
          // For each "high" obstacle cell, assign a uniform probability density to its immediate neighbors
          // (Only neighbors which are not unknown in the Cure map. Unknown 3D
          // space is still assigned, though)
          double dx, dy;
          (*lgm).index2WorldCoords(x, y, dx, dy);
          int nx, ny;
          if (lgmKH.worldCoords2Index(dx, dy, nx, ny) == 0) {
            if (lgmKH(nx, ny) != FLT_MAX) {
              m_objectBloxelMaps[id]->boxSubColumnModifier(bloxelX, bloxelY,
                  lgmKH(nx, ny) + 0.025, 0.05, initfunctor2);
            }

            if (m_bUseWallPrior) {
              for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                  if ((*lgm)(x + i, y + j) == '0' && (bloxelX + i
                      <= m_objectBloxelMaps[id]->getMapSize().first && bloxelX
                      + i > 0) && (bloxelY + i
                      <= m_objectBloxelMaps[id]->getMapSize().second && bloxelY
                      + i > 0)) {
                    //								/log("modifying bloxelmap pdf");
                    if (lgmKH(nx, ny) != FLT_MAX) {
                      m_objectBloxelMaps[id]->boxSubColumnModifier(bloxelX + i,
                          bloxelY + j, lgmKH(nx, ny) / 2 + 0.025, lgmKH(nx, ny)
                              + 0.05, initfunctor);
                    } else {
                      m_objectBloxelMaps[id]->boxSubColumnModifier(bloxelX + i,
                          bloxelY + j, .5, 1.0, initfunctor);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    double massAfterInit = initfunctor.getTotal() + initfunctor2.getTotal();
    //    double normalizeTo = (initfunctor.getTotal()*m_pout)/(1 - m_pout);
    normalizePDF(*m_objectBloxelMaps[id], pdfmass, massAfterInit);

    log("Done setting.");
  } else if (!alreadyGenerated) {
    log("Something weird happened!");
  }

  //Now that we've got our map generate cones for this
  //Todo: and generateViewCones based on this
  if (m_usePeekabot) {
    log("Displaying PDF Map in PB.");
    pbVis->AddPDF(*m_objectBloxelMaps[id], true);
    pbVis->Display2DCureMap(m_templateRoomGridMaps[newVPCommand->roomId],
        "roommap", true);
  }
  log("getting cones..");

  vector<comadata::ComaRoomPtr> comarooms;
  getMemoryEntries<comadata::ComaRoom> (comarooms, "coma");

  log("Got %d rooms", comarooms.size());

  if (comarooms.size() == 0) {
    log("No such ComaRoom with id %d! Returning", newVPCommand->roomId);
    return;
  }
  unsigned int i = 0;
  for (; i < comarooms.size(); i++) {
    log("Got coma room with room id: %d", comarooms[i]->roomId);
    if (comarooms[i]->roomId == newVPCommand->roomId) {
      break;
    }
  }

  FrontierInterface::PlaceInterfacePrx agg2(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  NavData::FNodePtr node = new NavData::FNode;
  node = agg2->getNodeFromPlaceID(comarooms[i]->containedPlaceIds[0]);

  ViewPointGenerator coneGenerator(this,
      m_templateRoomGridMaps[newVPCommand->roomId], m_objectBloxelMaps[id],
      m_samplesize, m_sampleawayfromobs, m_conedepth, m_tiltstep, m_panstep,
      m_horizangle, m_vertangle, m_minDistance, pdfmass, m_pdfthreshold,
      node->x, node->y);

  vector<ViewPointGenerator::SensingAction> viewcones =
      coneGenerator.getBest3DViewCones(m_roomNodes[newVPCommand->roomId]);

  log("got %d cones..", viewcones.size());

  // normalizing cone probabilities
  log("normalizing viewcone probabilities");
  m_locationToConeGroupNormalization[id] = 0;
  for (unsigned int i = 0; i < viewcones.size(); i++) {
    m_locationToConeGroupNormalization[id] += viewcones[i].totalprob;
  }
  m_locationToConeGroupNormalization[id] /= pdfmass;
  log("%f Total prob %f, normalizing constant %f", pdfmass,
      m_locationToConeGroupNormalization[id], 1
          / m_locationToConeGroupNormalization[id]);

  for (unsigned int i = 0; i < viewcones.size(); i++) {
    log("viewcone %d before blow up  %f", i, viewcones[i].totalprob);

    viewcones[i].totalprob = viewcones[i].totalprob * (1
        / m_locationToConeGroupNormalization[id]);

    log("viewcone %d after blow up  %f", i, viewcones[i].totalprob);
    log("viewcone %d pos %f %f %f", i, viewcones[i].pos[0],
        viewcones[i].pos[1], viewcones[i].pos[2]);
  }

  //Getting the place belief pointer


  // 1. Create conegroups out of viewcones
  // 2. Fill in relevant fields
  // 3. Create beliefs about them

  map<int, vector<ViewPointGenerator::SensingAction> > place_cones;
  for (size_t i = 0; i < viewcones.size(); i++) {
    int closestnode = GetClosestNodeId(viewcones[i].pos[0],
        viewcones[i].pos[1], viewcones[i].pos[2]);
    int conePlaceId = GetPlaceIdFromNodeId(closestnode);
    place_cones[conePlaceId].push_back(viewcones[i]);
  }

  vector<vector<ViewPointGenerator::SensingAction> > grouped_cones;
  vector<double> grouped_cones_minAngle;
  vector<double> grouped_cones_maxAngle;

  for (map<int, vector<ViewPointGenerator::SensingAction> >::iterator plIt =
      place_cones.begin(); plIt != place_cones.end(); plIt++) {
    int min_counter = 4;
    double opt_minAngle = 0;

    for (size_t i = 0; i < (*plIt).second.size(); i++) {
      double minAngle = (*plIt).second[i].pan;
      double stAngle = minAngle;
      int counter = 0;
      map<int, bool> collected_vc;
      while (collected_vc.size() < (*plIt).second.size()) {
        double maxAngle = stAngle + m_maxRange;
        double min_dist = 2 * M_PI;
        for (size_t k = 0; k < (*plIt).second.size(); k++) {
          double dist1 = (*plIt).second[k].pan - stAngle;
          while (dist1 < 0)
            dist1 += 2 * M_PI;
          while (dist1 > 2 * M_PI)
            dist1 -= 2 * M_PI;

          if (dist1 < m_maxRange) {
            collected_vc[k] = true;
          }

          double dist = (*plIt).second[k].pan - maxAngle;
          while (dist < 0)
            dist += 2 * M_PI;
          while (dist > 2 * M_PI)
            dist -= 2 * M_PI;
          if (dist < min_dist) {
            min_dist = dist;
          }
        }
        stAngle = maxAngle + min_dist - 0.0001;
        counter++;
      }
      if (counter < min_counter) {
        min_counter = counter;
        opt_minAngle = minAngle;
      }
    }
    log("alex viewcones in place %d", (*plIt).second.size());
    double stAngle = opt_minAngle;
    map<int, bool> collected_vc;
    while (collected_vc.size() < (*plIt).second.size()) {
      double maxAngle = stAngle + m_maxRange;
      double min_dist = 2 * M_PI;
      double max_dist = 0;
      vector<ViewPointGenerator::SensingAction> gr;
      for (size_t k = 0; k < (*plIt).second.size(); k++) {

        double dist1 = (*plIt).second[k].pan - stAngle;
        while (dist1 < 0)
          dist1 += 2 * M_PI;
        while (dist1 > 2 * M_PI)
          dist1 -= 2 * M_PI;

        if (dist1 < m_maxRange) {
          if (dist1 > max_dist)
            max_dist = dist1;
          if (collected_vc.count(k) == 0)
            gr.push_back((*plIt).second[k]);
          collected_vc[k] = true;
        }

        double dist = (*plIt).second[k].pan - maxAngle;
        while (dist < 0)
          dist += 2 * M_PI;
        while (dist > 2 * M_PI)
          dist -= 2 * M_PI;
        if (dist < min_dist) {
          min_dist = dist;
        }
      }
      grouped_cones_minAngle.push_back(stAngle);
      grouped_cones_maxAngle.push_back(stAngle + max_dist);

      bool swapped = true;
      int j = 0;
      ViewPointGenerator::SensingAction tmp;
      while (swapped) {
        swapped = false;
        j++;
        for (size_t k = 0; k < gr.size() - j; k++) {
          double dist1 = gr[k].pan - stAngle;
          while (dist1 < 0)
            dist1 += 2 * M_PI;
          while (dist1 > 2 * M_PI)
            dist1 -= 2 * M_PI;
          double dist2 = gr[k + 1].pan - stAngle;
          while (dist2 < 0)
            dist2 += 2 * M_PI;
          while (dist2 > 2 * M_PI)
            dist2 -= 2 * M_PI;

          if (dist1 < dist2) {
            tmp = gr[k];
            gr[k] = gr[k + 1];
            gr[k + 1] = tmp;
            swapped = true;
          }
        }
      }

      stAngle = maxAngle + min_dist - 0.0001;
      log("alex group size %d", gr.size());
      grouped_cones.push_back(gr);

    }

  }
  log("alex gcs %d", grouped_cones.size());

  for (unsigned int i = 0; i < grouped_cones.size(); i++) {
    /* GETTING PLACE BELIEFS */

    int closestnode = GetClosestNodeId(grouped_cones[i][0].pos[0],
        grouped_cones[i][0].pos[1], grouped_cones[i][0].pos[2]);
    int conePlaceId = GetPlaceIdFromNodeId(closestnode);
    cast::cdl::WorkingMemoryAddress placeWMaddress;
    // Get the place id for this cone
    vector<boost::shared_ptr<cast::CASTData<GroundedBelief> > > placeBeliefs;
    getWorkingMemoryEntries<GroundedBelief> ("spatial.sa", 0, placeBeliefs);
    if (placeBeliefs.size() == 0) {
      log(
          "Could not get any  GroundedBeliefs from spatial.sa returning without doing anything...");
      return;
    } else {
      log("Got %d GroundedBeliefs from spatial.sa", placeBeliefs.size());
    }

    for (unsigned int j = 0; j < placeBeliefs.size(); j++) {
      if (placeBeliefs[j]->getData()->type != "place") {
        log("Not a place belief, but a %s belief",
            placeBeliefs[j]->getData()->type.c_str());
        continue;
      }
      CondIndependentDistribsPtr dist(CondIndependentDistribsPtr::dynamicCast(
          placeBeliefs[j]->getData()->content));
      BasicProbDistributionPtr basicdist(BasicProbDistributionPtr::dynamicCast(
          dist->distribs["PlaceId"]));
      FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(
          basicdist->values));
      BasicProbDistributionPtr basicdist1(
          BasicProbDistributionPtr::dynamicCast(dist->distribs["placestatus"]));
      FormulaValuesPtr formulaValues1(FormulaValuesPtr::dynamicCast(
          basicdist1->values));

      IntegerFormulaPtr intformula(IntegerFormulaPtr::dynamicCast(
          formulaValues->values[0].val));
      ElementaryFormulaPtr elformula(ElementaryFormulaPtr::dynamicCast(
          formulaValues1->values[0].val));

      int placeid = intformula->val;
      //	log("Place Id for this belief: %d", placeid);
      if (conePlaceId == placeid && elformula->prop == "TRUEPLACE") {
        //		log("Got  place belief from roomid: %d", conePlaceId);
        placeWMaddress.id = placeBeliefs[j]->getID();
        placeWMaddress.subarchitecture = "spatial";
        break;
      }
    }

    // FIXME ASSUMPTION: One cone per conegroup
    ConeGroup c;
    c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
    c.bloxelMapId = id;
    for (size_t j = 0; j < grouped_cones[i].size(); j++) {
      c.viewcones.push_back(grouped_cones[i][j]);
    }
    c.minAngle = grouped_cones_minAngle[i];
    c.maxAngle = grouped_cones_maxAngle[i];

    c.roomId = newVPCommand->roomId;
    c.placeId = conePlaceId;
    c.relation = newVPCommand->relation;

    if (newVPCommand->relation == SpatialData::INROOM) {
      c.supportObjectId = "";
      c.supportObjectCategory = "";
      c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
    } else {
      c.relation
          = (newVPCommand->relation == SpatialData::INOBJECT ? SpatialData::INOBJECT
              : SpatialData::ON);
      c.supportObjectId = newVPCommand->supportObject;
      //FIXME: Get category from object ID
      c.supportObjectCategory = "";
      c.searchedObjectCategory = newVPCommand->searchedObjectCategory;
    }

    /* GETTING COMAROOM BELIEFS */
    log("Looking for Coma room beliefs...");
    cast::cdl::WorkingMemoryAddress WMaddress;
    if (newVPCommand->relation == SpatialData::INROOM) {
      //get the roomid belief WMaddress
      vector<boost::shared_ptr<cast::CASTData<GroundedBelief> > >
          comaRoomBeliefs;
      getWorkingMemoryEntries<GroundedBelief> ("coma", 0, comaRoomBeliefs);

      if (comaRoomBeliefs.size() == 0) {
        log(
            "Could not get any grounded beliefs returning without doing anything...");
        return;
      } else {
        log("Got %d Grounded beliefs", comaRoomBeliefs.size());
      }

      for (unsigned int j = 0; j < comaRoomBeliefs.size(); j++) {
        if ((comaRoomBeliefs[j]->getData()->type != "comaroom")) {
          log("Not a place belief, but a %s belief",
              comaRoomBeliefs[j]->getData()->type.c_str());
          continue;
        }
        CondIndependentDistribsPtr dist(
            CondIndependentDistribsPtr::dynamicCast(
                comaRoomBeliefs[j]->getData()->content));
        BasicProbDistributionPtr basicdist(
            BasicProbDistributionPtr::dynamicCast(dist->distribs["RoomId"]));
        FormulaValuesPtr formulaValues(FormulaValuesPtr::dynamicCast(
            basicdist->values));

        IntegerFormulaPtr intformula(IntegerFormulaPtr::dynamicCast(
            formulaValues->values[0].val));
        int roomid = intformula->val;
        log("ComaRoom Id for this belief: %d", roomid);
        if (newVPCommand->roomId == roomid) {
          log("Got room belief from roomid: %d", newVPCommand->roomId);
          WMaddress.id = comaRoomBeliefs[j]->getID();
          WMaddress.subarchitecture = "coma";
          break;
        }
      }
    } else if (newVPCommand->relation == SpatialData::INOBJECT
        || newVPCommand->relation == SpatialData::ON) {
      //else get the visualobject belief pointer instead
      log("Getting VisualObject beliefs");

      WMaddress.id = newVPCommand->supportObject;
      WMaddress.subarchitecture = "vision.sa";
    }

    log("Creating ConeGroup belief");
    m_coneGroupId++;

    if (m_usePeekabot) {
      for (size_t coneNumber = 0; coneNumber < c.viewcones.size(); coneNumber++) {
        PostViewCone(c.viewcones[coneNumber], m_coneGroupId * 1000 + coneNumber);
        log("post viewcone %d pos %f %f %f", m_coneGroupId * 1000 + coneNumber,
            c.viewcones[coneNumber].pos[0], c.viewcones[coneNumber].pos[1],
            c.viewcones[coneNumber].pos[2]);
      }
    }

    m_beliefConeGroups[m_coneGroupId] = c;
    eu::cogx::beliefs::slice::GroundedBeliefPtr b =
        new eu::cogx::beliefs::slice::GroundedBelief;
    epstatus::PrivateEpistemicStatusPtr beliefEpStatus =
        new epstatus::PrivateEpistemicStatus;
    beliefEpStatus->agent = "self";

    b->estatus = beliefEpStatus;
    b->type = "conegroup";
    b->id = newDataID();
    CondIndependentDistribsPtr CondIndProbDist = new CondIndependentDistribs;

    BasicProbDistributionPtr coneGroupIDProbDist = new BasicProbDistribution;
    BasicProbDistributionPtr searchedObjectLabelProbDist =
        new BasicProbDistribution;
    BasicProbDistributionPtr relationLabelProbDist = new BasicProbDistribution;
    BasicProbDistributionPtr supportObjectLabelProbDist =
        new BasicProbDistribution;
    BasicProbDistributionPtr coneProbabilityProbDist =
        new BasicProbDistribution;
    BasicProbDistributionPtr placeFromWhichObjectCanBeSeenProbDist =
        new BasicProbDistribution;
    BasicProbDistributionPtr isVisitedProbDist = new BasicProbDistribution;

    FormulaProbPairs pairs;
    FormulaProbPair searchedObjectFormulaPair, coneGroupIDLabelFormulaPair,
        relationLabelFormulaPair, supportObjectLabelFormulaPair,
        coneProbabilityFormulaPair, placeFromWhichObjectCanBeSeenFormulaPair,
        isVisitedFormulaPair;

    IntegerFormulaPtr coneGroupIDLabelFormula = new IntegerFormula;
    ElementaryFormulaPtr searchedObjectLabelFormula = new ElementaryFormula;
    ElementaryFormulaPtr relationLabelFormula = new ElementaryFormula;
    PointerFormulaPtr supportObjectLabelFormula = new PointerFormula;
    PointerFormulaPtr placeFromWhichObjectCanBeSeenFormula = new PointerFormula;
    FloatFormulaPtr coneProbabilityFormula = new FloatFormula;
    BooleanFormulaPtr isVisitedFormula = new BooleanFormula;

    coneGroupIDLabelFormula->val = m_coneGroupId;
    searchedObjectLabelFormula->prop = c.searchedObjectCategory;
    relationLabelFormula->prop
        = (newVPCommand->relation == SpatialData::ON ? "on" : "in");
    supportObjectLabelFormula->pointer = WMaddress; //c.supportObjectId; // this should be a pointer ideally
    placeFromWhichObjectCanBeSeenFormula->pointer = placeWMaddress;
    coneProbabilityFormula->val = c.getTotalProb();
    isVisitedFormula->val = false;

    searchedObjectFormulaPair.val = searchedObjectLabelFormula;
    searchedObjectFormulaPair.prob = 1;

    coneGroupIDLabelFormulaPair.val = coneGroupIDLabelFormula;
    coneGroupIDLabelFormulaPair.prob = 1;

    relationLabelFormulaPair.val = relationLabelFormula;
    relationLabelFormulaPair.prob = 1;

    supportObjectLabelFormulaPair.val = supportObjectLabelFormula;
    supportObjectLabelFormulaPair.prob = 1;

    coneProbabilityFormulaPair.val = coneProbabilityFormula;
    coneProbabilityFormulaPair.prob = 1;

    placeFromWhichObjectCanBeSeenFormulaPair.val
        = placeFromWhichObjectCanBeSeenFormula;
    placeFromWhichObjectCanBeSeenFormulaPair.prob = 1;

    isVisitedFormulaPair.val = isVisitedFormula;
    isVisitedFormulaPair.prob = 1;

    pairs.push_back(coneGroupIDLabelFormulaPair);
    FormulaValuesPtr formulaValues1 = new FormulaValues;
    formulaValues1->values = pairs;
    coneGroupIDProbDist->values = formulaValues1;
    pairs.clear();

    pairs.push_back(searchedObjectFormulaPair);
    FormulaValuesPtr formulaValues2 = new FormulaValues;
    formulaValues2->values = pairs;
    searchedObjectLabelProbDist->values = formulaValues2;
    pairs.clear();

    pairs.push_back(coneProbabilityFormulaPair);
    FormulaValuesPtr formulaValues3 = new FormulaValues;
    formulaValues3->values = pairs;
    coneProbabilityProbDist->values = formulaValues3;
    pairs.clear();

    pairs.push_back(relationLabelFormulaPair);
    FormulaValuesPtr formulaValues4 = new FormulaValues;
    formulaValues4->values = pairs;
    relationLabelProbDist->values = formulaValues4;
    pairs.clear();

    pairs.push_back(supportObjectLabelFormulaPair);
    FormulaValuesPtr formulaValues5 = new FormulaValues;
    formulaValues5->values = pairs;
    supportObjectLabelProbDist->values = formulaValues5;
    pairs.clear();

    pairs.push_back(placeFromWhichObjectCanBeSeenFormulaPair);
    FormulaValuesPtr formulaValues6 = new FormulaValues;
    formulaValues6->values = pairs;
    placeFromWhichObjectCanBeSeenProbDist->values = formulaValues6;
    pairs.clear();

    pairs.push_back(isVisitedFormulaPair);
    FormulaValuesPtr formulaValues7 = new FormulaValues;
    formulaValues7->values = pairs;
    isVisitedProbDist->values = formulaValues7;
    pairs.clear();

    coneGroupIDProbDist->key = "id";
    CondIndProbDist->distribs["id"] = coneGroupIDProbDist;
    searchedObjectLabelProbDist->key = "cg-label";
    CondIndProbDist->distribs["cg-label"] = searchedObjectLabelProbDist;
    relationLabelProbDist->key = "cg-relation";
    CondIndProbDist->distribs["cg-relation"] = relationLabelProbDist;
    supportObjectLabelProbDist->key = "cg-related-to";
    CondIndProbDist->distribs["cg-related-to"] = supportObjectLabelProbDist;
    coneProbabilityProbDist->key = "p-visible";
    CondIndProbDist->distribs["p-visible"] = coneProbabilityProbDist;
    placeFromWhichObjectCanBeSeenProbDist->key = "cg-place";
    CondIndProbDist->distribs["cg-place"]
        = placeFromWhichObjectCanBeSeenProbDist;

    isVisitedProbDist->key = "is-visited";
    CondIndProbDist->distribs["is-visited"] = isVisitedProbDist;

    b->content = CondIndProbDist;
    log("writing belief to WM..");
    m_coneGroupIdToBeliefId[m_coneGroupId] = b->id;
    addToWorkingMemory(b->id, "binder", b);
    log("wrote belief to WM..");
    if (m_usePeekabot) {
      showProbability( m_coneGroupId);
    }
  }

  if (WMAddress != "") {
    newVPCommand->status = SpatialData::SUCCESS;
    log("Overwriting command to change status to: SUCCESS");
    overwriteWorkingMemory<SpatialData::RelationalViewPointGenerationCommand> (
        WMAddress, newVPCommand);
  }

}

void AVS_ContinualPlanner::IcetoCureLGM(SpatialData::LocalGridMap icemap,
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

/* Process ConeGroup with id */
void AVS_ContinualPlanner::processConeGroup(int id, bool skipNav) {
  // Todo: Loop over cones in conegroup
  // FIXME ASSUMPTION ConeGroup contains one cone
  // FIXME ASSUMPTION Always observe one object at a time

  // Todo: Once we get a response from vision, calculate the remaining prob. value
  // Get ConeGroup

  if (m_beliefConeGroups.count(id) == 0) {
    log("No Cone Groups with id: %d, this is an indication of id mismatch!", id);
    log("We have %d cone groups", m_beliefConeGroups.size());

    MapConeType::const_iterator end = m_beliefConeGroups.end();
    for (MapConeType::const_iterator it = m_beliefConeGroups.begin(); it != end; ++it) {
      log("key: %d", it->first);
    }
    return;
  }
  log("Processing Cone Group with id %d, totalprob %f", id,
      m_beliefConeGroups[id].getTotalProb());
  m_currentConeGroupNumber = id;
  m_currentViewConeNumber = 0;
  m_currentConeGroup = &m_beliefConeGroups[m_currentConeGroupNumber];
  m_currentViewCone.first = m_currentConeGroupNumber * 1000
      + m_currentViewConeNumber;
  m_currentViewCone.second
      = m_currentConeGroup->viewcones[m_currentViewConeNumber];

  if (skipNav) {
    ViewConeUpdate(m_currentViewCone,
        m_objectBloxelMaps[m_currentConeGroup->bloxelMapId]);
  } else {

    // TODO: If we've already processed this viewcone, serve another one
    // slightly at a different place but looking at the same region
    // Otherwise just process the viewcone and add it to m_processedViewConeIDs
    // list
    // RSS update
    if (m_processedViewConeIDs.count(m_currentViewCone.first) > 0
        && m_randomViewCones) {
      // This view cone was processed before so now we need to serve a
      // different one
      log(
          "We've processed this viewcone before, so serve a random one observing the same space");
      ViewPointGenerator::SensingAction s = getRandomViewCone(
          m_currentViewCone.second);
      m_coneGroupId++;
      if (m_usePeekabot) {
        PostViewCone(s, m_currentViewCone.first);
        m_proxyCone = &m_ProxyViewPointsList[m_currentViewCone.first];
        m_proxyConePolygons
            = m_ProxyViewPointsPolygonsList[m_currentViewCone.first];
        ChangeCurrentViewConeColor(0.1, 0.1, 0.9);
      }

      Cure::Pose3D pos;
      pos.setX(s.pos[0]);
      pos.setY(s.pos[1]);
      pos.setTheta(s.pan);
      PostNavCommand(pos, SpatialData::GOTOPOSITION);
      log("Posting a nav command with a random view cone");
      log(
          "With parameters: x: %4.2f, y:%4.2f, z:%4.2f, pan: %4.2f, tilt: %4.2f, radius: %4.2f, horizontal fov: %4.2f, vertical fov: %4.2f",
          s.pos[0], s.pos[1], s.pos[2], s.pan, s.tilt, s.conedepth,
          s.horizangle, s.vertangle);
    } else {
      m_processedViewConeIDs.insert(m_currentViewCone.first);
      if (m_usePeekabot) {
        m_proxyCone = &m_ProxyViewPointsList[m_currentViewCone.first];
        m_proxyConePolygons
            = m_ProxyViewPointsPolygonsList[m_currentViewCone.first];
        ChangeCurrentViewConeColor(0.1, 0.1, 0.9);
      }
      Cure::Pose3D pos;
      double dist = 0.2;
      pos.setX(m_currentViewCone.second.pos[0] - dist * cos(
          m_currentViewCone.second.pan));
      pos.setY(m_currentViewCone.second.pos[1] - dist * sin(
          m_currentViewCone.second.pan));
      double minAngle = m_currentConeGroup->minAngle;
      double maxAngle = m_currentConeGroup->maxAngle;
      //double range = maxAngle - minAngle;
      double tol = 0;//m_maxRange - range;
      double theta = (maxAngle + minAngle) / 2;
      double diff1 = fabs(maxAngle - theta);
      if (diff1 > M_PI)
        diff1 -= M_PI;
      double diff2 = fabs(minAngle - theta);
      if (diff2 > M_PI)
        diff2 -= M_PI;

      if ((diff1 > M_PI / 2) || (diff2 > M_PI / 2)) {
        theta = theta + M_PI;
      }
      pos.setTheta(theta); //TODO not nessacerely - maybe between
      debug("nav %f %f", theta, m_currentViewCone.second.pan);
      PostNavCommand(pos, SpatialData::GOTOPOSITION, tol);
      log("Posting a nav command");
    }
  }
}

ViewPointGenerator::SensingAction AVS_ContinualPlanner::getRandomViewCone(
    ViewPointGenerator::SensingAction s) {

  double panRad = s.pan * M_PI / 180;
  //  double tiltRad = s.tilt * M_PI / 180;

  double centerx = cos(panRad) * s.conedepth + s.pos[0];
  double centery = sin(panRad) * s.conedepth + s.pos[1];

  int low = -10;
  int high = 10;

  srand(time(0));
  int n = 0;
  while (n == 0)
    n = rand() % (low - high) + low;

  double theta = panRad + n * M_PI / 180;

  double rimx = cos(theta - M_PI) * s.conedepth + centerx;
  double rimy = sin(theta - M_PI) * s.conedepth + centery;
  ViewPointGenerator::SensingAction ret;

  ret.pos.push_back(rimx);
  ret.pos.push_back(rimy);
  ret.pos.push_back(s.pos[2]);
  ret.pan = theta;
  ret.tilt = s.tilt;
  ret.conedepth = s.conedepth;
  ret.horizangle = s.horizangle;
  ret.vertangle = s.vertangle;
  ret.minDistance = s.minDistance;
  return ret;
}

void AVS_ContinualPlanner::showProbability(int coneGroupID) {
  char buf[32];
  peekabot::LabelProxy text;
  sprintf(buf, "%d", coneGroupID);
  text.add(m_ProxyViewPoints, buf, peekabot::REPLACE_ON_CONFLICT);
  sprintf(buf, "%.0f", round(m_beliefConeGroups[coneGroupID].getTotalProb()
      * 100));
  text.set_text(buf);

  double dist = 0.2;
  double x = m_beliefConeGroups[coneGroupID].viewcones[0].pos[0] - dist * cos(
      m_beliefConeGroups[coneGroupID].viewcones[0].pan);
  double y = m_beliefConeGroups[coneGroupID].viewcones[0].pos[1] - dist * sin(
      m_beliefConeGroups[coneGroupID].viewcones[0].pan);
  double minAngle = m_beliefConeGroups[coneGroupID].minAngle;
  double maxAngle = m_beliefConeGroups[coneGroupID].maxAngle;
  double theta = (maxAngle + minAngle) / 2;
  double diff1 = fabs(maxAngle - theta);
  if (diff1 > M_PI)
    diff1 -= M_PI;
  double diff2 = fabs(minAngle - theta);
  if (diff2 > M_PI)
    diff2 -= M_PI;

  if ((diff1 > M_PI / 2) || (diff2 > M_PI / 2)) {
    theta = theta + M_PI;
  }

  text.set_pose(x + dist * 3 * cos(theta), y + dist * 3 * sin(theta), 2, 0, 0,
      0);
  text.set_rotation(theta, 0, 0);
  text.set_scale(20, 20, 20);
  text.set_alignment(peekabot::ALIGN_CENTER);
  text.set_color(0, 0, 0);
}

void AVS_ContinualPlanner::ViewConeUpdate(std::pair<int,
    ViewPointGenerator::SensingAction> viewcone, BloxelMap* map) {

  log("Making VC update at: %f, %f, %f", viewcone.second.pos[0],
      viewcone.second.pos[1], viewcone.second.pos[2]);
  m_currentConeGroup->isprocessed = true;
  bool isAllConeGroupsProcessed = true;

  int coneGroupID = viewcone.first / 1000;

  /* FIXME
   *  !!! HACK !!!
   *  Since creating cones to cover %100 of the space is intractable with planner
   *  We actually create cones to cover a certain percentage of initial PDF
   *  But if the last cone is being processed return %100 as beta to Conceptual
   *  So the CP planner won't ask for this again
   *  Otherwise CP-DT enters a loop
   */
  // If there's even one conegroup which is not yet processed then it means this is not the last ConeGroup for this location
  for (std::map<int, ConeGroup>::const_iterator it = m_beliefConeGroups.begin(); it
      != m_beliefConeGroups.end(); ++it) {
    if (it->second.bloxelMapId == m_currentConeGroup->bloxelMapId) {
      if (!it->second.isprocessed) {
        log("Not all cone groups for this location are processed yet");
        isAllConeGroupsProcessed = false;
        break;
      }
    }
  }

  //GDProbSum sumcells;
  GDIsObstacle isobstacle;
  ///* DEBUG */
  //GDProbSum conesum;
  //map->coneQuery(viewcone.second.pos[0],viewcone.second.pos[1],
  //    viewcone.second.pos[2], viewcone.second.pan, viewcone.second.tilt, m_horizangle, m_vertangle, m_conedepth, 5, 5, isobstacle, conesum, conesum, m_minDistance);
  //	log("ViewConeUpdate: Cone sums to %f",conesum.getResult());

  /* DEBUG */

  //map->universalQuery(sumcells);
  //double initialMapPDFSum = sumcells.getResult();
  //log("ViewConeUpdate: Whole map PDF sums to: %f", initialMapPDFSum);
  // then deal with those bloxels that belongs to this cone

  GDProbScale scalefunctor(1 - m_sensingProb);
  map->coneModifier(viewcone.second.pos[0], viewcone.second.pos[1],
      viewcone.second.pos[2], viewcone.second.pan, viewcone.second.tilt,
      m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, scalefunctor,
      scalefunctor, m_minDistance);
  //sumcells.reset();
  //map->universalQuery(sumcells);

  log("process viewcone pos %f %f %f", viewcone.second.pos[0],
      viewcone.second.pos[1], viewcone.second.pos[2]);

  //double finalMapPDFSum = sumcells.getResult();
  //double differenceMapPDFSum = initialMapPDFSum - finalMapPDFSum;
  //log("diff map pdf sum %f",differenceMapPDFSum);

  //if (differenceMapPDFSum < 0){
  //	error("We managed to observe negative probability, something is very wrong!");
  //}

  //log("ViewConeUpdate: After cone update map sums to: %f", sumcells.getResult());

  SpatialData::ObjectSearchResultPtr result =
      new SpatialData::ObjectSearchResult;
  result->searchedObjectCategory = m_currentConeGroup->searchedObjectCategory;
  result->relation = m_currentConeGroup->relation;
  result->supportObjectCategory = m_currentConeGroup->supportObjectCategory;
  result->supportObjectId = m_currentConeGroup->supportObjectId;
  result->roomId = m_currentConeGroup->roomId;

  double
      oldConeProbability =
          m_beliefConeGroups[coneGroupID].viewcones[m_currentViewConeNumber].totalprob;
  double lostProbability = oldConeProbability * m_sensingProb;
  double newConeProbability = oldConeProbability - lostProbability;

  // ASSUMPTION: m_locationToBeta has such a key since it's filled in generateViewCones first!
  if (isAllConeGroupsProcessed) {
    log(
        "All cone groups for this location are processed return very high beta to Conceptual!");

    result->beta = 0.99;
  } else {
    result->beta = m_locationToBeta[m_currentConeGroup->bloxelMapId]
        + lostProbability
            / m_locationToInitialPdfmass[m_currentConeGroup->bloxelMapId];
  }
  m_locationToBeta[m_currentConeGroup->bloxelMapId] = result->beta;

  log("The observed ratio of location: %f", result->beta);

  log(
      "Publishing ObjectSearchResult with: category: %s, relation: %s, supportObjectCategory: %s, supportObjectId: %s",
      result->searchedObjectCategory.c_str(),
      relationToString(result->relation).c_str(),
      result->supportObjectCategory.c_str(), result->supportObjectId.c_str());

  log("Updating conegroup probability");

  //get the belief id
  try {
    log("Getting relevant conegroup belief with %s",
        m_coneGroupIdToBeliefId[coneGroupID].c_str());
    eu::cogx::beliefs::slice::GroundedBeliefPtr belief = getMemoryEntry<
        GroundedBelief> (m_coneGroupIdToBeliefId[coneGroupID], "binder");

    CondIndependentDistribsPtr dist = CondIndependentDistribsPtr::dynamicCast(
        belief->content);
    BasicProbDistributionPtr basicdist = BasicProbDistributionPtr::dynamicCast(
        dist->distribs["p-visible"]);
    FormulaValuesPtr formulaValues = FormulaValuesPtr::dynamicCast(
        basicdist->values);
    FloatFormulaPtr floatformula = FloatFormulaPtr::dynamicCast(
        formulaValues->values[0].val);
    log("Got conegroup beliefs probability %f ", floatformula->val);
    //		log("Will subtract %f from it:", differenceMapPDFSum* (1/ m_locationToConeGroupNormalization[m_currentConeGroup->bloxelMapId]));

    log("Sum probabilities check (before): %f",
        m_beliefConeGroups[coneGroupID].getTotalProb());
    log(
        "current viewcone prob: %f",
        m_beliefConeGroups[coneGroupID].viewcones[m_currentViewConeNumber].totalprob);

    m_beliefConeGroups[coneGroupID].viewcones[m_currentViewConeNumber].totalprob
        = newConeProbability;
    //    m_beliefConeGroups[coneGroupID].viewcones[m_currentViewConeNumber].totalprob -= differenceMapPDFSum*(1/m_locationToConeGroupNormalization[m_currentConeGroup->bloxelMapId]);

    log(
        "Sum probabilities check (should be equal to conegroup probability above): %f",
        m_beliefConeGroups[coneGroupID].getTotalProb());

    floatformula->val = m_beliefConeGroups[coneGroupID].getTotalProb();
    //		double newValue = floatformula->val - differenceMapPDFSum*(1/m_locationToConeGroupNormalization[m_currentConeGroup->bloxelMapId]);
    //		floatformula->val = newValue; // remove current conesum's result
    log("Changing conegroup probability to %f", floatformula->val);

    overwriteWorkingMemory(m_coneGroupIdToBeliefId[coneGroupID], "binder",
        belief);

    if (m_usePeekabot) {
      showProbability(coneGroupID);
    }

  } catch (DoesNotExistOnWMException e) {
    log("Error! ConeGroup belief missing on WM!");
  }

  // this means it's the first time we're reporting search result
  if (m_locationToBetaWMAddress.count(m_currentConeGroup->bloxelMapId) == 0) {
    log(
        "this is the first time we're adding a ObjectSearchResult for this id: %s",
        m_currentConeGroup->bloxelMapId.c_str());
    string wmid = newDataID();
    m_locationToBetaWMAddress[m_currentConeGroup->bloxelMapId] = wmid;
    addToWorkingMemory(wmid, result);
  } else {
    try {
      log("overwriting object search result at WMAdress %s",
          m_locationToBetaWMAddress[m_currentConeGroup->bloxelMapId].c_str());
      overwriteWorkingMemory(
          m_locationToBetaWMAddress[m_currentConeGroup->bloxelMapId], result);
    } catch (DoesNotExistOnWMException e) {
      log("Error! ObjectSearchResult missing on WM!");
    }
  }
}

void AVS_ContinualPlanner::Recognize() {
  log("Sending a recognition command");
  if (m_usePeekabot) {
    ChangeCurrentViewConeColor(0.1, 0.9, 0.9);
  }
  for (unsigned int i = 0; i < m_siftObjects.size(); i++) {
    //todo: ask for visual object recognition
    if (m_siftObjects[i] == m_currentConeGroup->searchedObjectCategory) {
      log("This is a sift object posting a 3DRecognizer command \n");
      addRecognizer3DCommand(VisionData::RECOGNIZE,
          m_currentConeGroup->searchedObjectCategory, "");
      break;
    }
  }

  for (unsigned int i = 0; i < m_ARtaggedObjects.size(); i++) {
    //todo: ask for tagged objects
    if (m_ARtaggedObjects[i] == m_currentConeGroup->searchedObjectCategory) {
      log("This is an ARTag object posting an ARTagcommand \n");
      VisionData::ARTagCommandPtr cmd = new VisionData::ARTagCommand;
      cmd->label = m_currentConeGroup->searchedObjectCategory;
      addToWorkingMemory(newDataID(), "vision.sa", cmd);
      break;
    }
  }
}

std::string AVS_ContinualPlanner::convertLocation2Id(
    SpatialData::RelationalViewPointGenerationCommandPtr newVPCommand) {
  string id = "";
  string relationstring =
      (newVPCommand->relation == SpatialData::INOBJECT) ? "INOBJECT" : "INROOM";
  if (newVPCommand->supportObject != "")
    id = newVPCommand->searchedObjectCategory + relationstring
        + newVPCommand->supportObject + boost::lexical_cast<string>(
        newVPCommand->roomId);
  else
    id = newVPCommand->searchedObjectCategory + relationstring
        + newVPCommand->supportObject + boost::lexical_cast<string>(
        newVPCommand->roomId);
  return id;
}

void AVS_ContinualPlanner::configure(
    const std::map<std::string, std::string>& _config) {
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

  if (cfg->getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose for laser");
    std::abort();
  }

  m_PbPort = 5050;
  cfg->getRoboLookHost(m_PbHost);
  std::string usedCfgFile, tmp;
  if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
    m_PbHost = tmp;
  }
  m_maxRange = 0.8 * M_PI;
  m_RetryDelay = 1000;
  if (_config.find("--retry-interval") != _config.end()) {
    std::istringstream str(_config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

  if (_config.find("--sensing-probability") != _config.end()) {
    std::istringstream str(_config.find("--sensing-probability")->second);
    str >> m_sensingProb;
    if (m_sensingProb <= 0.0 || m_sensingProb > 1.0) {
      error("Warning! Sensing probability set to %f!", m_sensingProb);
    }
  }

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

  m_samplesize = 100;

  it = _config.find("--samplesize");
  if (it != _config.end()) {
    m_samplesize = (atof(it->second.c_str()));
    log("Samplesize set to: %d", m_samplesize);
  }

  m_minbloxel = 0.05;
  it = _config.find("--minbloxel");
  if (it != _config.end()) {
    m_minbloxel = (atof(it->second.c_str()));
    log("Min bloxel height set to: %f", m_minbloxel);
  }

  m_mapceiling = 2.0;
  it = _config.find("--mapceiling");
  if (it != _config.end()) {
    m_mapceiling = (atof(it->second.c_str()));
    log("Map ceiling set to: %d", m_mapceiling);
  }

  it = _config.find("--kernel-width-factor");
  if (it != _config.end()) {
    m_sampler.setKernelWidthFactor(atoi(it->second.c_str()));
  }

  m_horizangle = M_PI / 4;
  it = _config.find("--cam-horizangle");
  // NOTE: expecting angle in degrees in cast file!
  if (it != _config.end()) {
    m_horizangle = (atof(it->second.c_str())) * M_PI / 180.0;
    log("Camera FoV horizontal angle set to: %f", m_horizangle);
  }

  m_vertangle = M_PI / 4;
  // NOTE: expecting angle in degrees in cast file!
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

  m_bUseWallPrior = true;
  it = _config.find("--no-wall-prior");
  if (it != _config.end()) {
    m_bUseWallPrior = false;
  }

  m_panstep = 30.0;
  it = _config.find("--panstep");
  if (it != _config.end()) {
    m_panstep = (atof(it->second.c_str()));
    log("panstep set to: %f", m_panstep);
  }

  m_tiltstep = 30.0;
  it = _config.find("--tiltstep");
  if (it != _config.end()) {
    m_tiltstep = (atof(it->second.c_str()));
    log("tiltstep set to: %f", m_tiltstep);
  }

  m_sampleawayfromobs = 0.2;
  it = _config.find("--sampleawayfromobs");
  if (it != _config.end()) {
    m_sampleawayfromobs = (atof(it->second.c_str()));
    log("Distance to nearest obs for samples set to: %f", m_sampleawayfromobs);
  }

  m_minDistance = m_conedepth / 4.0;

  m_usePeekabot = false;
  if (_config.find("--usepeekabot") != _config.end()) {
    m_usePeekabot = true;

    connectPeekabot();
  }

  if (m_usePeekabot) {
    pbVis = new VisualPB_Bloxel(m_PbHost, 5050, m_gridsize, m_gridsize,
        m_cellsize, 1, true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time
    pbVis->connectPeekabot();
  }

  if (_config.find("--exclude-from-exploration") != _config.end()) {
    std::istringstream str(_config.find("--exclude-from-exploration")->second);

    ForbiddenZone newZone;
    newZone.minX = -DBL_MAX;
    newZone.maxX = DBL_MAX;
    newZone.minY = -DBL_MAX;
    newZone.maxY = DBL_MAX;
    while (!str.eof()) {
      string buf;
      str >> buf;

      if (buf == "or") {
        println("new forbidden zone: %.02g, %.02g, %.02g, %.02g", newZone.minX,
            newZone.minY, newZone.maxX, newZone.maxY);

        m_forbiddenZones.push_back(newZone);
        newZone.minX = -DBL_MAX;
        newZone.maxX = DBL_MAX;
        newZone.minY = -DBL_MAX;
        newZone.maxY = DBL_MAX;
      } else if (buf == "x") {
        str >> buf;
        if (buf == ">") {
          str >> newZone.minX;
        } else if (buf == "<") {
          str >> newZone.maxX;
        } else {
          log("Warning: Malformed --exclude-from-exploration string");
          break;
        }
      } else if (buf == "y") {
        str >> buf;
        if (buf == ">") {
          str >> newZone.minY;
        } else if (buf == "<") {
          str >> newZone.maxY;
        } else {
          log("Warning: Malformed --exclude-from-exploration string");
          break;
        }
      } else {
        log("Warning: Malformed --exclude-from-exploration string");
        break;
      }
    }
    println("new forbidden zone: %.02g, %.02g, %.02g, %.02g", newZone.minX,
        newZone.minY, newZone.maxX, newZone.maxY);

    m_forbiddenZones.push_back(newZone);
  }

  m_usePTZ = false;
  if (_config.find("--ctrl-ptu") != _config.end()) {
    m_usePTZ = true;
    log("will use ptu");
  }

  m_sampleRandomPoints = false;
  if (_config.find("--sample-random-points") != _config.end()) {
    m_sampleRandomPoints = true;
  }

  m_randomViewCones = false;
  if (m_sampleRandomPoints) {
    // Disallow randomization of view cones if only locations at nodes are permitted
    if (_config.find("--random-vc") != _config.end()) {
      m_randomViewCones = true;
      log("Will generate random view cones.");
    }
  }

  m_runInSimulation = false;
  if (_config.find("--simulation") != _config.end()) {
    m_runInSimulation = true;
    log("will run in simulation");
  }

  m_ignoreTilt = false;
  if (_config.find("--ignore-tilt") != _config.end())
    m_ignoreTilt = true;

  // QueryHandler name
  if ((it = _config.find("--queryhandler")) != _config.end()) {
    m_queryHandlerName = it->second;
  }

  //	if (m_usePTZ) {
  //			log("connecting to PTU");
  //			Ice::CommunicatorPtr ic = getCommunicator();
  //
  //			Ice::Identity id;
  //			id.name = "PTZServer";
  //			id.category = "PTZServer";
  //
  //			std::ostringstream str;
  //			str << ic->identityToString(id) << ":default" << " -h localhost"
  //					<< " -p " << cast::cdl::CPPSERVERPORT;
  //
  //			Ice::ObjectPrx base = ic->stringToProxy(str.str());
  //			m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
  //		}

  if ((it = _config.find("--siftobjects")) != _config.end()) {
    istringstream istr(it->second);
    string label;
    while (istr >> label) {
      m_siftObjects.push_back(label);
    }
  }
  log("Loaded sift objects.");
  for (unsigned int i = 0; i < m_siftObjects.size(); i++)
    log("%s , ", m_siftObjects[i].c_str());

  if ((it = _config.find("--ARtagobjects")) != _config.end()) {
    istringstream istr(it->second);
    string label;
    while (istr >> label) {
      m_ARtaggedObjects.push_back(label);
    }
  }
  log("Loaded tagged objects.");
  for (unsigned int i = 0; i < m_ARtaggedObjects.size(); i++)
    log("%s , ", m_ARtaggedObjects[i].c_str());

  m_pdfthreshold = 0.9;
  if ((it = _config.find("--pdfthreshold")) != _config.end()) {
    m_pdfthreshold = atof(it->second.c_str());
  }

  m_allObjects.reserve((m_siftObjects.size() + m_ARtaggedObjects.size()));
  m_allObjects.insert(m_allObjects.end(), m_siftObjects.begin(),
      m_siftObjects.end());
  m_allObjects.insert(m_allObjects.end(), m_ARtaggedObjects.begin(),
      m_ARtaggedObjects.end());

  m_currentProcessConeGroup = new SpatialData::ProcessConeGroup;
  m_coneGroupId = 0;

  SpatialData::AVSInterfacePtr servant = new AVSServer(this);
  registerIceServer<SpatialData::AVSInterface, SpatialData::AVSInterface> (
      servant);

}

void AVS_ContinualPlanner::AVSServer::simulateViewCones(
    const SpatialData::RelationalViewPointGenerationCommandPtr &cmd,
    const Ice::Current &) {
  SpatialData::RelationalViewPointGenerationCommandPtr cmd2 =
      new SpatialData::RelationalViewPointGenerationCommand;
  cmd2->searchedObjectCategory = cmd->searchedObjectCategory;
  cmd2->relation = cmd->relation;
  cmd2->supportObject = cmd->supportObject;
  cmd2->roomId = cmd->roomId;
  cmd2->supportObjectCategory = cmd->supportObjectCategory;

  m_pOwner->generateViewCones(cmd2, "");
}

void AVS_ContinualPlanner::AVSServer::processViewCones(const int id,
    const Ice::Current &) {
  m_pOwner->processConeGroup(id, true);
}

void AVS_ContinualPlanner::newRobotPose(const cdl::WorkingMemoryChange &objID) {
  try {
    lastRobotPose = getMemoryEntry<NavData::RobotPose2d> (objID.address);

  } catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
    return;
  }

}

AVS_ContinualPlanner::NavCommandReceiver::NavCommandReceiver(
    AVS_ContinualPlanner & _component, SpatialData::NavCommandPtr _cmd) :
  m_component(_component), m_cmd(_cmd) {
  m_component.log("received NavCommandReceiver notification");
  string id(m_component.newDataID());
  m_component.log("ID post: %s", id.c_str());

  m_component.addChangeFilter(createIDFilter(id, cdl::OVERWRITE), this);
  m_component.addToWorkingMemory<SpatialData::NavCommand> (id, m_cmd);

}

void AVS_ContinualPlanner::NavCommandReceiver::workingMemoryChanged(
    const cast::cdl::WorkingMemoryChange &objID) {
  m_component.log("received inner notification");
  try {
    m_component.owtNavCommand(objID);
  } catch (const CASTException &e) {
    //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
  }

}

void AVS_ContinualPlanner::owtNavCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  SpatialData::NavCommandPtr cmd(getMemoryEntry<SpatialData::NavCommand> (
      objID.address));
  try {
    if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
      log("Nav Command succeeded");
      // it means we've reached viewcone position
      if (!m_runInSimulation) {
        log("Not running in simulation mode, moving pan tilt");
        if (m_usePeekabot) {
          ChangeCurrentViewConeColor(0.1, 0.9, 0.1);
        }
        double angle = m_currentViewCone.second.pan - lastRobotPose->theta;
        while (angle < -M_PI)
          angle += 2 * M_PI;
        while (angle > M_PI)
          angle -= 2 * M_PI;

        startMovePanTilt(angle, m_currentViewCone.second.tilt, 0.08);
        m_ptzWaitingStatus = WAITING_TO_RECOGNIZE;
        //			Recognize();
      }
      if (m_runInSimulation) {
        log("Updating the %s bloxel map",
            m_currentConeGroup->bloxelMapId.c_str());
        ViewConeUpdate(m_currentViewCone,
            m_objectBloxelMaps[m_currentConeGroup->bloxelMapId]);
        m_currentProcessConeGroup->status = SpatialData::SUCCESS;
        log("Overwriting command to change status to: SUCCESS");
        overwriteWorkingMemory<SpatialData::ProcessConeGroup> (
            m_processConeGroupCommandWMAddress, m_currentProcessConeGroup);
      }
    } else if (cmd->comp == SpatialData::COMMANDFAILED) {
      // it means we've failed to reach the viewcone position
      if (m_usePeekabot) {
        ChangeCurrentViewConeColor(0.9, 0.1, 0.1);
      }

      ViewConeUpdate(m_currentViewCone,
          m_objectBloxelMaps[m_currentConeGroup->bloxelMapId]);
      m_currentProcessConeGroup->status = SpatialData::FAILED;
      log("Overwriting command to change status to: FAILED");
      overwriteWorkingMemory<SpatialData::ProcessConeGroup> (
          m_processConeGroupCommandWMAddress, m_currentProcessConeGroup);
    }
  } catch (const CASTException &e) {
    //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
  }
}

void AVS_ContinualPlanner::PostNavCommand(Cure::Pose3D position,
    SpatialData::CommandType cmdtype, double tol) {
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->prio = SpatialData::URGENT;
  cmd->cmd = cmdtype;
  cmd->pose.resize(3);
  cmd->pose[0] = position.getX();
  cmd->pose[1] = position.getY();
  cmd->pose[2] = position.getTheta();
  if (tol == 0) {
    cmd->tolerance.resize(1);
    cmd->tolerance[0] = 0.1;
  } else {
    cmd->tolerance.resize(2);
    cmd->tolerance[0] = 0.1;
    cmd->tolerance[1] = tol;
  }

  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  new NavCommandReceiver(*this, cmd);
  log("posted nav command");
}

void AVS_ContinualPlanner::addRecognizer3DCommand(
    VisionData::Recognizer3DCommandType cmd, std::string label,
    std::string visualObjectID) {
  log("posting recognizer command: %s.", label.c_str());
  VisionData::Recognizer3DCommandPtr rec_cmd =
      new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  string id = newDataID();
  log("constructed visual command, adding to WM (%s)", id.c_str());
  addToWorkingMemory(id, "vision.sa", rec_cmd);
  log("added to WM");
}

void AVS_ContinualPlanner::startMovePanTilt(double pan, double tilt,
    double tolerance) {
  ptz::SetPTZPoseCommandPtr newPTZPoseCommand = new ptz::SetPTZPoseCommand;
  newPTZPoseCommand->pose.pan = pan;
  newPTZPoseCommand->pose.tilt = tilt;
  newPTZPoseCommand->comp = ptz::COMPINIT;

  string cmdId = newDataID();
  addToWorkingMemory(cmdId, newPTZPoseCommand);

  m_waitingForPTZCommandID = cmdId;
}

void AVS_ContinualPlanner::overwrittenPanTiltCommand(
    const cdl::WorkingMemoryChange &objID) {
  if (objID.address.id == m_waitingForPTZCommandID) {
    try {
      ptz::SetPTZPoseCommandPtr overwritten = getMemoryEntry<
          ptz::SetPTZPoseCommand> (objID.address);
      if (overwritten->comp == ptz::SUCCEEDED) {

      } else {
        log("Warning! Failed to move PTZ before moving!");
      }
      deleteFromWorkingMemory(objID.address);
    } catch (DoesNotExistOnWMException e) {
      log("Error: SetPTZPoseCommand went missing! ");
    }

    m_waitingForPTZCommandID = "";
  }
}

//void AVS_ContinualPlanner::MovePanTilt(double pan, double tilt, double tolerance) {
//
//	if (m_usePTZ) {
//		log(" Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
//		ptz::PTZPose p;
//		ptz::PTZReading ptuPose;
//		p.pan = pan;
//		p.tilt = tilt;
//
//		p.zoom = 0;
//		m_ptzInterface->setPose(p);
//		bool run = true;
//		ptuPose = m_ptzInterface->getPose();
//		double actualpan = ptuPose.pose.pan;
//		double actualtilt = ptuPose.pose.tilt;
//
//		while (run) {
//
//			m_ptzInterface->setPose(p);
//			ptuPose = m_ptzInterface->getPose();
//			actualpan = ptuPose.pose.pan;
//			actualtilt = ptuPose.pose.tilt;
//
//			log("desired pan tilt is: %f %f", pan, tilt);
//			log("actual pan tilt is: %f %f", actualpan, actualtilt);
//			log("tolerance is: %f", tolerance);
//
//			//check that pan falls in tolerance range
//			if (actualpan < (pan + tolerance) && actualpan > (pan - tolerance)) {
//				run = false;
//			}
//
//			//only check tilt if pan is ok
//
//			if (!run) {
//				if (m_ignoreTilt) {
//					run = false;
//				} else {
//					if (actualtilt < (tilt + tolerance) && actualtilt > (tilt
//							- tolerance)) {
//						run = false;
//					} else {
//						//if the previous check fails, loop again
//						run = true;
//					}
//				}
//			}
//
//			usleep(100000);
//		}
//		log("Moved.");
//		sleep(1);
//	}
//}

void AVS_ContinualPlanner::addARTagCommand() {
  //todo: add new AR tag command
}
string AVS_ContinualPlanner::relationToString(SpatialData::SpatialRelation rel) {
  if (rel == SpatialData::INROOM)
    return "inroom";
  else if (rel == SpatialData::INOBJECT)
    return "inobject";
  else if (rel == SpatialData::ON)
    return "on";
  else
    return "This is not a relation, something bad happened";
}

int AVS_ContinualPlanner::GetPlaceIdFromNodeId(int nodeId) {
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  SpatialData::PlacePtr p = new SpatialData::Place;
  p = agg->getPlaceFromNodeID(nodeId);
  if (p != 0)
    return p->id;
  else
    return -1;
}
int AVS_ContinualPlanner::GetClosestNodeId(double x, double y, double a) {

  NavData::NavGraphInterfacePrx agg(getIceServer<NavData::NavGraphInterface> (
      "navgraph.process"));
  int d = agg->getClosestNodeId(x, y, a, 3);
  return d;
}

void AVS_ContinualPlanner::PostViewCone(
    const ViewPointGenerator::SensingAction &nbv, int id) {
  NavData::ViewPoint viewpoint;
  viewpoint.pos.x = nbv.pos[0];
  viewpoint.pos.y = nbv.pos[1];
  viewpoint.pos.z = nbv.pos[2];
  viewpoint.length = m_conedepth;
  viewpoint.pan = nbv.pan;
  viewpoint.tilt = nbv.tilt;

  char path[32];
  double color[3];
  color[0] = 0.9;
  color[1] = 0.1;
  color[2] = 0.1;
  sprintf(path, "viewpoint_%d", id);

  m_ProxyViewPointsList[id].add(m_ProxyViewPoints, path,
      peekabot::REPLACE_ON_CONFLICT);
  m_ProxyViewPointsPolygonsList[id] = new peekabot::PolygonProxy[5];
  createFOV(m_ProxyViewPointsList[id], m_ProxyViewPointsPolygonsList[id],
      m_horizangle, m_vertangle, color, 0.15, viewpoint);
}
void AVS_ContinualPlanner::putObjectInMap(GridMap<GridMapData> &map,
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
    log("Posting a nav command");
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

void AVS_ContinualPlanner::displayPDF(BloxelMap map) {
  //	pbVis->AddPDF(map,true);
}

} //namespace

