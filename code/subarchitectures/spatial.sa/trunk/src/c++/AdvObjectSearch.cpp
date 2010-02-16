/*
 * AdvObjectSearch.cpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#include "AdvObjectSearch.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <stdlib.h>

#include <CureHWUtils.hpp>
#include <AddressBank/ConfigFileReader.hh>

#include <NavData.hpp>
#include <SpatialData.hpp>
#include <iostream>
#include <fstream>

using namespace cast;
using namespace Cure;
using namespace spatial;
using namespace std;
using namespace boost;

namespace spatial
{

  extern "C" {
    FrameworkProcess* newComponent(const string &_id) {
      return new NavTester(_id);
    }
  }

  AdvObjectSearch::AdvObjectSearch() {
    // TODO Auto-generated constructor stub
    // If we're not building the map it means we're using an already built one. Hence, read it.

  }

  AdvObjectSearch::~AdvObjectSearch() {
    // TODO Auto-generated destructor stub
    log("Destructor called.");
    if (m_table_phase) {
      log("Saving map with planes to planemap.txt");
      SavePlaneMap();
    }

  }
  void
  AdvObjectSearch::SavePlaneMap() {
    char buf[32];
    sprintf(buf, "planemap.txt");
    ofstream fout(buf);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        fout << (*m_lgm)(x, y) << " ";
      }
      fout << endl;
    }
    fout.close();
  }

  void
  AdvObjectSearch::configure(const std::map<std::string, std::string>& _config) {

    map<string, string>::const_iterator it = _config.find("-c");

    if (it == _config.end()) {
      log("configure(...) Need config file (use -c option)\n");
      std::abort();
    }
    std::string configfile = it->second;
    Cure::ConfigFileReader cfg;
    if (cfg.init(configfile)) {
      log("configure(...) Failed to open with %s\n", configfile.c_str());
      std::abort();
    }



    m_table_phase = (_config.find("--table-phase") == _config.end());
    m_usePTZ = (_config.find("--ctrl-ptu") == _config.end());
    int gridsize = 400;
    float cellsize = 0.1;
    it = _config.find("--gridsize");
    if (it != _config.end()) {

      gridsize = (atoi(it->second.c_str()));
      log("Gridsize set to: %f", gridsize);
    }
    it = _config.find("--cellsize");
    if (it != _config.end()) {
      cellsize = (atof(it->second.c_str()));
      log("Cellsize set to: %f", cellsize);
    }

    m_lgm = new Cure::LocalGridMap<unsigned int>(gridsize / 2, cellsize, 255,
        Cure::LocalGridMap<unsigned int>::MAP1);
    log("Used gridsize=%d cellsize=%d", gridsize, cellsize);
    try {
      if (!m_table_phase) {
        log("Restoring plane map");
        ifstream file;
        file.open("planemap.txt");
        char c;
        for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
          for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
            if (file.good()) {
              c = file.get();
              if (c != ' ')
                (*m_lgm)(x, y) = c;
            }
          }
          file.close();
        }
      }
    }
    catch (std::exception e) {
      log("Could not initialize planemap.txt");
    }
    if (m_table_phase)
      m_Glrt = new Cure::GridLineRayTracer<unsigned int>(*m_lgm);

    if (m_usePTZ) {
        log("connecting to PTU");
        Ice::CommunicatorPtr ic = getCommunicator();

        Ice::Identity id;
        id.name = "PTZServer";
        id.category = "PTZServer";

        std::ostringstream str;
        str << ic->identityToString(id)
          << ":default"
          << " -h localhost"
          << " -p " << cast::cdl::CPPSERVERPORT;

        Ice::ObjectPrx base = ic->stringToProxy(str.str());
        m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);

        ptz::PTZPose p;
         p.pan = 20*3.141562/180;
         p.tilt = 0;
         p.zoom = 0;
         m_ptzInterface->setPose(p);
      }



  }
  void
  AdvObjectSearch::start() {

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (cdl::ADD),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::newPlanePointCloud));

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (
        cdl::OVERWRITE), new MemberFunctionChangeReceiver<AdvObjectSearch> (
        this, &AdvObjectSearch::newPlanePointCloud));

    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::newRobotPose));

    addChangeFilter(
        createLocalTypeFilter<NavData::RobotPose2d> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::newRobotPose));

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (cdl::ADD),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::newPlanePointCloud));

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (
        cdl::OVERWRITE), new MemberFunctionChangeReceiver<AdvObjectSearch> (
        this, &AdvObjectSearch::newPlanePointCloud));

  }
  void
  AdvObjectSearch::runComponent() {

  }

  void
  AdvObjectSearch::receiveScan2d(const Laser::Scan2d &castScan) {
    // only add scans if we are in the map construction phase
    if (m_table_phase) {
      lockComponent(); //Don't allow any interface calls while processing a callback
      debug("Got scan with n=%d and t=%ld.%06ld", castScan.ranges.size(),
          (long) castScan.time.s, (long) castScan.time.us);

      Cure::LaserScan2d cureScan;
      CureHWUtils::convScan2dToCure(castScan, cureScan);

      if (m_TOPP.isTransformDefined()) {

        Cure::Pose3D scanPose;
        if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
          Cure::Pose3D lpW;
          lpW.add(scanPose, m_LaserPoseR);
          m_Mutex.lock();
          m_Glrt->addScan(cureScan, lpW, 5.0);
          m_Mutex.unlock();
        }
      }
      unlockComponent();
    }
  }

  void
  AdvObjectSearch::newRobotPose(const cast::cdl::WorkingMemoryChange &objID) {

    try {
      lastRobotPose = getMemoryEntry<NavData::RobotPose2d> (objID.address);
    }
    catch (DoesNotExistOnWMException e) {
      log("Error! robotPose missing on WM!");
      return;
    }

    m_SlamRobotPose.setX(lastRobotPose->x);
    m_SlamRobotPose.setY(lastRobotPose->y);
    m_SlamRobotPose.setTheta(lastRobotPose->theta);

    Cure::Pose3D cp = m_SlamRobotPose;
    m_TOPP.defineTransform(cp);
  }

  void
  AdvObjectSearch::newPlanePointCloud(
      const cast::cdl::WorkingMemoryChange &objID) {
    log("new PlanePointCloud received.");

    try {

      SpatialData::PlanePointsPtr objData = getMemoryEntry<
          SpatialData::PlanePoints> (objID.address);

      //TODO: Add plane points to m_lgm
      //add plane points
      int xG, yG; //grid coordinates
      float x, y;
      log("points size: %d", objData->points.size());
      for (unsigned int i = 0; i < objData->points.size(); i++) {
        x = objData->points.at(i).x;
        y = objData->points.at(i).y;
        m_lgm->worldCoords2Index(x, y, xG, yG);
        (*m_lgm)(xG, yG) = 't';
      }
    }
    catch (DoesNotExistOnWMException) {
      log("Error! plane point cloud disappeared from WM.");
    }
  }
}
