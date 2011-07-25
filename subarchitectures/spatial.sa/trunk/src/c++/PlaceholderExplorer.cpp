#include "PlaceholderExplorer.hpp"

#include <NavData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <queue>
#include <limits>
#include <math.h>

using namespace navsa;

extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceholderExplorer();
  }
}

PlaceholderExplorer::PlaceholderCompare::PlaceholderCompare(FrontierInterface::PlaceInterfacePrx _proxy, double x, double y)
: proxy(_proxy), robotX(x), robotY(y)
{
}

bool PlaceholderExplorer::PlaceholderCompare::operator()(const SpatialData::PlacePtr& lhs, const SpatialData::PlacePtr& rhs) const
{
  return getCost(lhs) > getCost(rhs);
}

double PlaceholderExplorer::PlaceholderCompare::getCost(const SpatialData::PlacePtr& placeholder) const
{
  FrontierInterface::NodeHypothesisPtr node = proxy->getHypFromPlaceID(placeholder->id); 

  if (!node)
    return std::numeric_limits<float>::max();

  double distanceSq = (node->x - robotX) * (node->x - robotX) + (node->y - robotY) * (node->y - robotY);

  return distanceSq;
}

PlaceholderExplorer::PlaceholderExplorer() {
  m_status = NavData::SUCCEEDED;
  m_hasPosition = false;
}

void PlaceholderExplorer::configure(const std::map<std::string, std::string>& config) {
}

void PlaceholderExplorer::start() {
  addChangeFilter(cast::createLocalTypeFilter<NavData::RobotPose2d>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<PlaceholderExplorer>(this, &PlaceholderExplorer::poseChange));

  addChangeFilter(cast::createLocalTypeFilter<SpatialData::NavCommand>(cast::cdl::OVERWRITE),
      new cast::MemberFunctionChangeReceiver<PlaceholderExplorer>(this, &PlaceholderExplorer::navCommandResponse));
}



SpatialData::PlacePtr PlaceholderExplorer::getPlaceholder()
{
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));
  PlaceholderCompare comparator(agg, m_x, m_y);
  priority_queue<SpatialData::PlacePtr, vector<SpatialData::PlacePtr>, PlaceholderCompare> sortedPlaces(comparator);

  vector<SpatialData::PlacePtr> places;
  getMemoryEntries<SpatialData::Place>(places);

  for (vector<SpatialData::PlacePtr>::iterator it = places.begin();
      it != places.end(); ++it) {
    if ((*it)->status == SpatialData::PLACEHOLDER) {
      sortedPlaces.push(*it);
    }
  }

  if (sortedPlaces.empty()) {
    return NULL;
  }
  else {
    return sortedPlaces.top();
  }
}

void PlaceholderExplorer::runComponent() {
  SpatialData::PlacePtr place = NULL;

  while(true) {
    if(m_status == SpatialData::COMMANDINPROGRESS || m_status == SpatialData::COMMANDPENDING) {
      sleep(1);
      continue;
    }

    if (!m_hasPosition || !(place = getPlaceholder())) {
      sleep(1);
      continue;
    }

    log("Trying to go to place id: %d", place->id);

    SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand;
    cmd->prio = SpatialData::NORMAL;
    cmd->cmd = SpatialData::GOTOPLACE;
    cmd->destId.push_back(place->id);
    cmd->comp = SpatialData::COMMANDPENDING;
    cmd->status = SpatialData::NONE;
    /* Lower the default tolerance so we don't get as much rejected hypotheses */
    cmd->tolerance.push_back(0.1);

    log("Going to %d", place->id);

    m_status = NavData::INPROGRESS;

    std::string id = newDataID();
    addToWorkingMemory<SpatialData::NavCommand>(id,cmd);
  }
}

void PlaceholderExplorer::poseChange(const cast::cdl::WorkingMemoryChange &objID) {
  NavData::RobotPose2dPtr oobj = getMemoryEntry<NavData::RobotPose2d>(objID.address);

  log("Got new position (%f, %f)", oobj->x, oobj->y);

  m_x = oobj->x;
  m_y = oobj->y;
  m_theta = oobj->theta;

  m_hasPosition = true;
}

void PlaceholderExplorer::navCommandResponse(const cast::cdl::WorkingMemoryChange &objID) {

  SpatialData::NavCommandPtr oobj = getMemoryEntry<SpatialData::NavCommand>(objID.address);

  m_status = oobj->comp;

  switch(oobj->comp) {
    case 0:
      log("Navigation command status: PENDING");
      break;
    case 1:
      log("Navigation command status: INPROGRESS");
      break;
    case 2:
      log("Navigation command status: ABORTED");
      break;
    case 3:
      log("Navigation command status: FAILED");
      break;
    case 4:
      log("Navigation command status: SUCCEEDED");
      break;
  }
}




