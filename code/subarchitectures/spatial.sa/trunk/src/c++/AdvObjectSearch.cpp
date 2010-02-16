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
#include <VisionData.hpp>
#include <iostream>
#include <fstream>
#include "XVector3D.h"
namespace spatial
{
  using namespace cast;
  using namespace spatial;
  using namespace std;
  using namespace boost;

  extern "C"
  {
    cast::CASTComponentPtr
    newComponent() {
      return new AdvObjectSearch();
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
  AdvObjectSearch::BuildPrior() {

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

    m_table_phase = false;
    if (_config.find("--table-phase") != _config.end()) {
      m_table_phase = true;
      log("Plane phase");
    }

    m_usePTZ = false;
    if (_config.find("--ctrl-ptu") != _config.end()) {
      m_usePTZ = true;
      log("will use ptu");
    }

    m_samplesize = 100;
    it = _config.find("--samplesize");
    if (it != _config.end()) {
      m_samplesize = (atof(it->second.c_str()));
      log("Samplesize set to: %d", m_samplesize);
    }
    m_samples = new int[2 * m_samplesize];

    int gridsize = 400;
    float cellsize = 0.1;
    it = _config.find("--gridsize");
    if (it != _config.end()) {

      gridsize = (atoi(it->second.c_str()));
      log("Gridsize set to: %d", gridsize);
    }
    it = _config.find("--cellsize");
    if (it != _config.end()) {
      cellsize = (atof(it->second.c_str()));
      log("Cellsize set to: %f", cellsize);
    }

    m_fov = M_PI / 4;
    it = _config.find("--cam-fov");
    if (it != _config.end()) {
      m_fov = (atof(it->second.c_str())) * M_PI / 180;
    }

    m_CamRange = 1;
    it = _config.find("--cam-range");
    if (it != _config.end()) {
      m_CamRange = (atof(it->second.c_str()));
    }
    log("Camera range set to: %f", m_CamRange);

    if ((it = _config.find("--objects")) != _config.end()) {
      istringstream istr(it->second);
      string label;
      while (istr >> label) {
        m_objectlist.push_back(label);
      }
    }
    log("Loaded objects.");

    m_lgm = new Cure::LocalGridMap<char>(gridsize / 2, cellsize, 2,
        Cure::LocalGridMap<char>::MAP1);
    m_lgm_prior = new Cure::LocalGridMap<double>(gridsize / 2, cellsize, 2,
        Cure::LocalGridMap<double>::MAP1);
    m_lgm_posterior = new Cure::LocalGridMap<double>(gridsize / 2, cellsize, 2,
        Cure::LocalGridMap<double>::MAP1);

    m_lgm_seen = new Cure::LocalGridMap<bool>(gridsize / 2, cellsize, false,
        Cure::LocalGridMap<char>::MAP1);

    m_Dlgm = new Cure::X11DispLocalGridMap<char>(*m_lgm);
    m_Glrt = new Cure::ObjGridLineRayTracer<char>(*m_lgm);

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


    if (m_usePTZ) {
      log("connecting to PTU");
      Ice::CommunicatorPtr ic = getCommunicator();

      Ice::Identity id;
      id.name = "PTZServer";
      id.category = "PTZServer";

      std::ostringstream str;
      str << ic->identityToString(id) << ":default" << " -h localhost"
          << " -p " << cast::cdl::CPPSERVERPORT;

      Ice::ObjectPrx base = ic->stringToProxy(str.str());
      m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);

      ptz::PTZPose p;
      p.pan = 20 * 3.141562 / 180;
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

    addChangeFilter(createChangeFilter<VisionData::VisualObject> (cdl::ADD, "",
        "", "vision.sa", cdl::ALLSA), new MemberFunctionChangeReceiver<
        AdvObjectSearch> (this, &AdvObjectSearch::newObjectDetected));

    addChangeFilter(createChangeFilter<VisionData::VisualObject> (
        cdl::OVERWRITE, "", "", "vision.sa", cdl::ALLSA),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::newObjectDetected));

  }

  void
  AdvObjectSearch::runComponent() {
    setupPushScan2d(*this, 0.1);
    setupPushOdometry(*this);

    log("hey I'm running.");
     while (isRunning()) {
       lockComponent();
       m_Dlgm->updatePlaneDisplay();
       unlockComponent();
       sleepComponent(1000);
     }

  }


  void AdvObjectSearch::receiveOdometry(const Robotbase::Odometry &castOdom)
  {
    lockComponent(); //Don't allow any interface calls while processing a callback
   // Cure::Pose3D CurrPose;
    Cure::Pose3D cureOdom;
    CureHWUtils::convOdomToCure(castOdom, cureOdom);

    debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
          cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
          cureOdom.getTime().getDouble());

    m_TOPP.addOdometry(cureOdom);

  //  m_CurrPose = m_TOPP.getPose();
    unlockComponent();
  }


  void
  AdvObjectSearch::receiveScan2d(const Laser::Scan2d &castScan) {

    debug("Got scan with n=%d and t=%ld.%06ld", castScan.ranges.size(),
              (long) castScan.time.s, (long) castScan.time.us);
      lockComponent(); //Don't allow any interface calls while processing a callback


      Cure::LaserScan2d cureScan;
      CureHWUtils::convScan2dToCure(castScan, cureScan);

      if (m_TOPP.isTransformDefined()) {

        Cure::Pose3D scanPose;
        if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
          Cure::Pose3D lpW;
          lpW.add(scanPose, m_LaserPoseR);
          m_Mutex.lock();
          log("adding scan to glrt");
          m_Glrt->addScan(cureScan, lpW, 5.0);
          m_Mutex.unlock();
        }
      }
      unlockComponent();
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

  int*
  AdvObjectSearch::NextBestView() {
    // TODO: Get the highest scoring cone mutha_uckas!
    int* nbv = new int[2];
    std::vector<std::vector<int> > VCones;
    SampleGrid();
    VCones = GetViewCones();
    double highest_sum = -10000;
    double sum;
    int highest_VC_index = 0;
    int x, y;
    for (unsigned int i = 0; i < VCones.size(); i++) {
      sum = 0;
      for (unsigned int j = 0; j < VCones[i].size() / 2; j++) {
        x = VCones[i][2 * j];
        y = VCones[i][2 * j + 1];
        if (!(*m_lgm_seen)(x, y))
          sum += (*m_lgm_posterior)(x, y);
      }
      if (sum > highest_sum) {
        highest_sum = sum;
        highest_VC_index = i;
      }
    }
    nbv[0] = m_samples[2*highest_VC_index];
    nbv[1] = m_samples[2*highest_VC_index + 1];
    return nbv;
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
        (*m_lgm)(xG, yG) = 3;
      }
    }
    catch (DoesNotExistOnWMException) {
      log("Error! plane point cloud disappeared from WM.");
    }
  }


  void AdvObjectSearch::SetPrior(){

  }
  void
  AdvObjectSearch::newObjectDetected(
      const cast::cdl::WorkingMemoryChange &objID) {
    shared_ptr<CASTData<VisionData::VisualObject> > oobj =
        getWorkingMemoryEntry<VisionData::VisualObject> (objID.address);

    VisionData::VisualObjectPtr obj = oobj->getData();
    if (obj->detectionConfidence >= 0.5) {
      // TODO: measurement update

    }
    else {
      // TODO: measurement update
    }

  }

  void
  AdvObjectSearch::SampleGrid() {
    srand (
    time(NULL));

    /*Sampling free space BEGIN*/
    /*Checking if a point in x,y is reachable */

    /// Binary grid that is 0 for foree space and 1 for occupied
    Cure::BinaryMatrix m_NonFreeSpace;

    // We make the number of columns of the BinaryMatrix a multiple
    // of 32 so that we get the benefit of the representation.
    // Here m_LGMap is assumed to be the LocalGridMap
    int rows = 2 * m_lgm->getSize() + 1;
    int cols = ((2 * m_lgm->getSize() + 1) / 32 + 1) * 32;
    m_NonFreeSpace.reallocate(rows, cols);
    m_NonFreeSpace = 0; // Set all cells to zero

    // First we create a binary matrix where all cells that
    // corresponds to known obstacles are set to "1".
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 1 || (*m_lgm)(x, y) == 3) { // occupied OR Plane
          m_NonFreeSpace.setBit(x + m_lgm->getSize(), y + m_lgm->getSize(),
              true);
        }
      }
    }

    // Create an istance of BinaryMatrx which will hold the result of
    // expanding the obstacles
    Cure::BinaryMatrix m_PathGrid;
    m_PathGrid = 0;

    // Grow each occupied cell to account for the size of the
    // robot. We put the result in another binary matrix, m_PathGrid
    m_NonFreeSpace.growInto(m_PathGrid, 0.5 * 0.45 / m_lgm->getCellSize(), // 0.45 is robot width hard coded here.
        true);

    // We treat all unknown cells as occupied so that the robot only
    // uses paths that it knowns to be free. Note that we perfom this
    // operation directly on the m_PathGrid, i.e. the grid with the
    // expanded obstacle. The reasoning behind this is that we do not
    // want the unknown cells to be expanded as well as we would have
    // to recalculate the position of the frontiers otherwise, else
    // they might end up inside an obstacle (could happen now as well
    // from expanding the occupied cell but then it is known not to be
    // reachable).
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2) {
          m_PathGrid.setBit(x + m_lgm->getSize(), y + m_lgm->getSize(), true);
        }
      }
    }

    /*Checking if a point in x,y is reachable */
    int i = 0;
    int randx, randy;
    double xW, yW;
    while (i < m_samplesize) {
      randx = (randx % (2 * m_lgm->getSize())) - m_lgm->getSize();
      randy = (randy % (2 * m_lgm->getSize())) - m_lgm->getSize();
      m_lgm->index2WorldCoords(randx, randy, xW, yW);
      if ((*m_lgm)(randx, randy) == 0 && !(*m_lgm_seen)(randx, randy)) {
        /*if reachable*/
        // Get the indices of the destination coordinates
        int rS, cS, rE, cE;
        if (m_lgm->worldCoords2Index(lastRobotPose->x, lastRobotPose->y, rS, cS)
            == 0 && m_lgm->worldCoords2Index(xW, yW, rE, cE) == 0) {
          // Compensate for the fact that the PathGrid is just a normal matrix where the cells are numbers from the corner
          cS += m_lgm->getSize();
          rS += m_lgm->getSize();
          cE += m_lgm->getSize();

          rE += m_lgm->getSize();

          Cure::ShortMatrix path;
          double d = (m_PathGrid.path(rS, cS, rE, cE, path, 20
              * m_lgm->getSize()) * m_lgm->getCellSize());
          if (d > 0 && d < 2) {
            // There is a path to this destination
            m_samples[2 * i] = randx;
            m_samples[2 * i + 1] = randy;
            i++;
          }
          /*if reachable*/
        }

      }
    }

    /*Sampling free space END*/
  }

  std::vector<std::vector<int> >
  AdvObjectSearch::GetViewCones() {
    log("Calculating view cones for generated samples");
    Cure::Pose3D candidatePose;
    XVector3D a;
    std::vector<int> tpoints;
    std::vector<std::vector<int> > ViewConePts;

    for (int y = 0; y < m_samplesize; y++) { //calc. view cone for each sample

      m_lgm->index2WorldCoords(m_samples[y * 2], m_samples[2 * y + 1], a.x, a.y);
      tpoints = GetInsideViewCone(a, true);
      ViewConePts.push_back(tpoints);
      candidatePose.setX(a.x);
      candidatePose.setY(a.y);
      //log("CurrentPose.Theta : %f", candidatePose.getTheta());
      //candidatePoses.push_back(candidatePose);
    }
    /*log("View Cones calculated.");
     if (m_Displaykrsjlgm == 0)
     m_Displaykrsjlgm = new Cure::X11DispLocalGridMap<unsigned char>(*m_lgm);*/

    return ViewConePts;
  }

  std::vector<int>
  AdvObjectSearch::GetInsideViewCone(XVector3D &a, bool addall) {
    std::vector<int> tpoints;
    XVector3D b, c, p;
    XVector3D m_a, m_b, m_c;
    int* rectangle = new int[4];
    int h, k;
    CalculateViewCone(a, a.theta, m_CamRange, m_fov, b, c);

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
          tpoints.push_back(x);
          tpoints.push_back(y);
        }

      }
    }
    vector<int>::iterator theIterator = tpoints.begin();
    tpoints.insert(theIterator, 1, m_a.y);
    theIterator = tpoints.begin();
    tpoints.insert(theIterator, 1, m_a.x);
    return tpoints;

  }
  void
  AdvObjectSearch::CalculateViewCone(XVector3D a, double direction,
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

  void
  AdvObjectSearch::FindBoundingRectangle(XVector3D a, XVector3D b, XVector3D c,
      int* rectangle) {
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

  bool
  AdvObjectSearch::isPointInsideTriangle(XVector3D p, XVector3D a, XVector3D b,
      XVector3D c) { //the first one is the point the rest triangle

    if (isPointSameSide(p, a, b, c) && isPointSameSide(p, b, a, c)
        && isPointSameSide(p, c, a, b)) {
      return true;
    }
    else {
      return false;
    }

  }
  bool
  AdvObjectSearch::isPointSameSide(XVector3D p1, XVector3D p2, XVector3D a,
      XVector3D b) {
    XVector3D cp1 = (b - a).crossVector3D((p1 - a));
    XVector3D cp2 = (b - a).crossVector3D((p2 - a));
    if (cp1.dotVector3D(cp2) >= 0) {
      return true;
    }
    else {
      return false;
    }

  }

  void
  AdvObjectSearch::PostRecognitionCommand() {
    log("Posting Recog. Command now");
    VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
    cmd->labels = m_objectlist;
    addToWorkingMemory(newDataID(), "vision.sa", cmd);
    log("DetectionCommand added.");
  }

}
