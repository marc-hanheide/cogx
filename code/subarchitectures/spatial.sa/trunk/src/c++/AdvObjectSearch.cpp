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

  }
  void
  AdvObjectSearch::SavePlaneMap() {
    ofstream fout("planemap.txt");
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        fout << (*m_lgm)(x, y);
      }
      //fout << endl;
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
    /* if (_config.find("--table-phase") != _config.end()) {
     m_table_phase = true;
     log("Plane phase");
     }*/

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
    m_samplestheta = new double[m_samplesize];
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

    m_MaxExplorationRange = 1.5;
    it = _config.find("--explore-range");
    if (it != _config.end()) {
      m_MaxExplorationRange = (atof(it->second.c_str()));
    }

    m_CamRange = 1;
    it = _config.find("--cam-range");
    if (it != _config.end()) {
      m_CamRange = (atof(it->second.c_str()));
    }
    log("Camera range set to: %f", m_CamRange);

    if ((it = _config.find("--probs")) != _config.end()) {
      istringstream istr(it->second);
      string tmp;
      istr >> tmp;
      pFree = atof(tmp.c_str());

      istr >> tmp;
      pObs = atof(tmp.c_str());

      istr >> tmp;
      pPlanar = atof(tmp.c_str());

      istr >> tmp;
      pIn = atof(tmp.c_str());

      istr >> tmp;
      pOut = atof(tmp.c_str());
    }
    log("Loaded probs %f,%f,%f,%f,%f", pFree, pObs, pPlanar, pIn, pOut);

    if ((it = _config.find("--objects")) != _config.end()) {
      istringstream istr(it->second);
      string label;
      while (istr >> label) {
        m_objectlist.push_back(label);
      }
    }
    log("Loaded objects.");

    PDFData def;
    def.prob = pIn / pow(double((2 * gridsize + 1)), 2);
    def.isSeen = false;
    def.isChecked = false;

    m_lgm = new Cure::LocalGridMap<unsigned int>(gridsize, cellsize, 2,
        Cure::LocalGridMap<unsigned int>::MAP1);

    m_pdf_total = new Cure::LocalGridMap<PDFData>(gridsize, cellsize, def,
        Cure::LocalGridMap<unsigned int>::MAP1);

    m_pdf_obj = new Cure::LocalGridMap<double>(gridsize, cellsize,  pIn / pow(double((2 * gridsize + 1)), 2), Cure::LocalGridMap<unsigned int>::MAP1);


    m_Dlgm = new Cure::X11DispLocalGridMap<unsigned int>(*m_lgm);
    m_Glrt = new Cure::ObjGridLineRayTracer<unsigned int>(*m_lgm);

    m_ProbGivenObjectIsPresent = 0.7;

    m_pPlaneGivenObj = 0.7;
    m_pFreeGivenObj = 0.05;
    m_pObsGivenObj = 0.25;

    m_pPlaneGivenNotObj = 0.05;
    m_pFreeGivenNotObj = 0.8;
    m_pObsGivenNotObj = 0.15;

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

    try {
      m_PeekabotClient.connect("localhost", 5050, true);
      m_ProxyPrior.add(m_PeekabotClient, "root.Prior",
          peekabot::REPLACE_ON_CONFLICT);
      m_ProxyPDFIn.add(m_PeekabotClient, "root.PDFIn",
          peekabot::REPLACE_ON_CONFLICT);
      m_ProxySeenMap.add(m_PeekabotClient, "root.SeenMap",
          peekabot::REPLACE_ON_CONFLICT);
    }
    catch (std::exception e) {
      log("Could not connect to PB, %s", e.what());
    }

    img = 0;
    cvNamedWindow("test", CV_WINDOW_AUTOSIZE);
    img = cvLoadImage("lolcat.jpg");
    cvShowImage("test", img);
    log("hey I'm running.");
    while (isRunning()) {
      lockComponent();
      
      m_Dlgm->updatePlaneDisplay(&m_SlamRobotPose);
      unlockComponent();
      //log("update plane display");
      int key = cvWaitKey(100);
      
      if (key == 115) {
        m_table_phase = true;
        log("Saving plane map!");
        SavePlaneMap();
        cvReleaseImage(&img);
      }
      else if (key == 116) {
        log("Table mode!");
        //SetPrior();
        m_table_phase = true;

      }
      else if (key == 117){
        GetTotalPDF();
      }
      else if (key == 112) {
        log("Getting next view");
        GoToNBV();
      }
      else if (key == 114) {
        m_table_phase = false;
        log("Reading plane map!");
        int length;
        char * buffer;
        // m_Mutex.lock();
        ifstream file("planemap.txt");

        file.seekg(0, ios::end);
        length = file.tellg();
        file.seekg(0, ios::beg);
        buffer = new char[length];
        file.read(buffer, length);
        int index = 0;
        for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
          for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
            char c = buffer[index];
            int ii = atoi(&c);
            (*m_lgm)(x, y) = ii;
            index++;
          }
        }

        SpatialData::PlanePointsPtr PlanePoints;
        PlanePoints = new SpatialData::PlanePoints;
        cogx::Math::Vector3 point;
        double wX, wY;
        std::pair<int, int> CoordPair;
        std::set<std::pair<int, int> > NewPlanePoints;
        for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
          for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
            if ((*m_lgm)(x, y) == 3) {
              CoordPair.first = x;
              CoordPair.second = y;
              NewPlanePoints.insert(CoordPair);
              m_lgm->index2WorldCoords(x, y, wX, wY);
              point.x = wX;
              point.y = wY;
              point.z = 0.5; // FIXME: Z information is lost so we're making this up
              PlanePoints->points.push_back(point);
            }
          }
        }
      }
      //unlockComponent();
    }

  }
  void
  AdvObjectSearch::GoToNBV() {
    int nbv;
    nbv = NextBestView();
    double Wx, Wy;

    m_lgm->index2WorldCoords(m_samples[2 * nbv], m_samples[2 * nbv + 1], Wx, Wy);
    log("Best view coords: %f, %f, %f", Wx, Wy, m_samplestheta[nbv]);
    Cure::Pose3D pos;
    pos.setX(Wx);
    pos.setY(Wy);
    pos.setTheta(m_samplestheta[nbv]);
    m_currentViewPoint = pos;
    /* Add plan to PB BEGIN */
    NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
    cogx::Math::Vector3 a;
    a.x = Wx;
    a.y = Wy;
    a.z = m_samplestheta[nbv];
    obs->planlist.push_back(a);
    addToWorkingMemory(newDataID(), obs);
    /* Add plan to PB END */

    // Assume:
    // 1. posted nav comamnd
    // 2. reached position
    // 3. sent recognize command
    // 4. received visualobject
    // 5. ????
    // 6. PROFIT!
    //PostNavCommand(pos);
    log("calling measurement update");
    MeasurementUpdate(false);

  }
  void
  AdvObjectSearch::receiveOdometry(const Robotbase::Odometry &castOdom) {
    lockComponent(); //Don't allow any interface calls while processing a callback
    // Cure::Pose3D CurrPose;
    Cure::Pose3D cureOdom;
    CureHWUtils::convOdomToCure(castOdom, cureOdom);

    debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f", cureOdom.getX(),
        cureOdom.getY(), cureOdom.getTheta(), cureOdom.getTime().getDouble());

    m_TOPP.addOdometry(cureOdom);

    //  m_CurrPose = m_TOPP.getPose();
    unlockComponent();
  }

  void
  AdvObjectSearch::receiveScan2d(const Laser::Scan2d &castScan) {
    //log("Ignoring laser scans!!");
    //return;

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
        m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);
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

  int
  AdvObjectSearch::NextBestView() {

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
        sum += (*m_pdf_total)(x, y).prob;
      }
      if (sum > highest_sum) {
        highest_sum = sum;
        highest_VC_index = i;
      }
    }
    for (unsigned int j = 0; j < VCones[highest_VC_index].size() / 2; j++) {
      x = VCones[highest_VC_index][2 * j];
      y = VCones[highest_VC_index][2 * j + 1];
      if (!(*m_pdf_total)(x, y).isSeen)
        (*m_pdf_total)(x, y).isSeen = true;
    }
    m_CurrentViewPoint_Points = VCones[highest_VC_index];

    /* Display SeenMap in PB BEGIN */
    double color[3] =
      { 0.5, 0.5, 0.5 };
    double xW, yW;
    m_ProxySeenMap.clear_vertices();
    m_ProxySeenMap.set_color(color[0], color[1], color[2]);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_pdf_total)(x, y).isSeen == true || (*m_lgm)(x, y) == 2)
          continue;
        m_lgm->index2WorldCoords(x, y, xW, yW);
        m_ProxySeenMap.add_vertex(xW + 6, yW, 0);
      }
    }
    /* Display SeenMap in PB END */

    return highest_VC_index;
  }

  void
  AdvObjectSearch::newPlanePointCloud(
      const cast::cdl::WorkingMemoryChange &objID) {
    debug("Got new plane points");
    if (!m_table_phase)
      return;
    try {

      SpatialData::PlanePointsPtr objData = getMemoryEntry<
          SpatialData::PlanePoints> (objID.address);
      debug("points size: %d", objData->points.size());
      //add plane points
      std::set<std::pair<int, int> > NewPlanePoints;
      std::pair<int, int> CoordPair;
      for (unsigned int i = 0; i < objData->points.size(); i++) {
        m_lgm->worldCoords2Index(objData->points.at(i).x,
            objData->points.at(i).y, CoordPair.first, CoordPair.second);
        if ((*m_lgm)(CoordPair.first, CoordPair.second) != 3) {
            NewPlanePoints.insert(CoordPair);
        }
	
        (*m_lgm)(CoordPair.first, CoordPair.second) = 3;

        // TODO: do not initialize PDF with fixed value if on the fly adding of planes is on.
      }
    }
    catch (DoesNotExistOnWMException) {
      log("Error! plane point cloud disappeared from WM.");
    }
  }


  void
  AdvObjectSearch::DisplayPriorinPB() {

    double uUnit = pIn / pow(double((2 * m_lgm->getSize() + 1)), 2);

    /* DEBUG */
    double sumin = 0.0;
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        sumin += (*m_pdf_obj)(x, y);
      }
    }
    log("P(c_i) sums to: %f", sumin);
    log("P(c_i) + Cout sums to: %f", sumin + pOut);

    /* DEBUG */

    /* Display PDF in PB as Line Cloud BEGIN */

    double multiplier1 = 500.0;
    double xW2, yW2, xW3, yW3;
    peekabot::LineCloudProxy linecloudp;

    linecloudp.add(m_PeekabotClient, "root.LC_BEFORE",
        peekabot::REPLACE_ON_CONFLICT);
    linecloudp.clear_vertices();
    linecloudp.set_line_width(2);
    linecloudp.set_color(0.9, 0, 0);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2 || y == m_lgm->getSize())
          continue;
        m_lgm->index2WorldCoords(x, y, xW2, yW2);
        m_lgm->index2WorldCoords(x, y + 1, xW3, yW3);
        linecloudp.add_line(xW2 + 6, yW2 - 8,
            (*m_pdf_obj)(x, y) * multiplier1, xW3 + 6, yW3 - 8, (*m_pdf_obj)(x, y
                + 1) * multiplier1);
      }
    }

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2 || x == m_lgm->getSize())
          continue;
        m_lgm->index2WorldCoords(x + 1, y, xW2, yW2);
        m_lgm->index2WorldCoords(x, y, xW3, yW3);
        linecloudp.add_line(xW2 + 6, yW2 - 8,
            (*m_pdf_obj)(x, y) * multiplier1, xW3 + 6, yW3 - 8, (*m_pdf_obj)(x, y
                + 1) * multiplier1);
      }
    }
    /* Display pdfIn in as line cloud PB END */

  }

  void
  AdvObjectSearch::newObjectDetected(
      const cast::cdl::WorkingMemoryChange &objID) {
    shared_ptr<CASTData<VisionData::VisualObject> > oobj =
        getWorkingMemoryEntry<VisionData::VisualObject> (objID.address);

    VisionData::VisualObjectPtr obj = oobj->getData();
    if (obj->detectionConfidence >= 0.5) {
      // TODO: measurement update
      MeasurementUpdate(true);
    }
    else {
      // TODO: measurement update
      MeasurementUpdate(false);
    }

  }

  double
  AdvObjectSearch::ActionProbabilityPerCell(int x, int y,
      std::vector<int> ViewConePoints) {
    /* This is the probability that for a cell in the map
     * what are the chances that the recognition algorithm will spot it.
     * If the cell in question is out of FOV then this is zero. If not
     * then it's a constant. */

    for (unsigned int i = 0; i < ViewConePoints.size(); i++) {
      if (ViewConePoints[2 * i] == x && ViewConePoints[2 * i + 1] == y) {
        log("ActionProbcell returned %f", 0.7);
        return 0.7; // FIXME: this is lame.
      }
    }
    return 0;

  }

  void
  AdvObjectSearch::MeasurementUpdate(bool result) {
    log("in here..");
    if (result) {
      log("you're done. go play outside.");
      return;
    }

    double denomsum = 0.0;
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
	denomsum += (*m_pdf_obj)(x, y) * (1 - ActionProbabilityPerCell(x, y,
            m_CurrentViewPoint_Points));
      }
    }
    denomsum += pOut;

    log("pOut is: %f", pOut);
    log("denomsum is: %f", denomsum);
    // For everything inside meaning: pIn
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        (*m_pdf_obj)(x, y) = ((*m_pdf_obj)(x, y) * (1
            - ActionProbabilityPerCell(x, y, m_CurrentViewPoint_Points)))
            / denomsum;
      }
    }

    pOut = pOut / denomsum;
    log("pOut has become: %f", pOut);




    //GetTotalPDF();


       /* Display pdfIn in PB as Line Cloud BEGIN */

         double multiplier1 = 10000.0;
         double xW2, yW2, xW3, yW3;
         peekabot::LineCloudProxy linecloudp;

         linecloudp.add(m_PeekabotClient, "root.LC_OBJ",
             peekabot::REPLACE_ON_CONFLICT);
         linecloudp.clear_vertices();
         linecloudp.set_line_width(2);
         linecloudp.set_color(0.2, 0.9, 0.2);

         for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             if ((*m_lgm)(x, y) == 2 || y == m_lgm->getSize())
               continue;
             m_pdf_total->index2WorldCoords(x, y, xW2, yW2);
             m_pdf_total->index2WorldCoords(x, y + 1, xW3, yW3);
             linecloudp.add_line(xW2 -6, yW2,
                 (*m_pdf_obj)(x, y) * multiplier1, xW3 -6, yW3, (*m_pdf_obj)(x, y + 1) * multiplier1);
           }
         }

        /* for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             if ((*m_lgm)(x, y) == 2 || x == m_lgm->getSize())
               continue;
             m_pdf_total->index2WorldCoords(x + 1, y, xW2, yW2);
             m_pdf_total->index2WorldCoords(x, y, xW3, yW3);
             linecloudp.add_line(xW2 + 6, yW2 - 8,
                 (*m_pdf_total)(x, y).prob * multiplier1, xW3 + 6, yW3 - 8, (*m_pdf_total)(x, y
                     + 1).prob * multiplier1);
           }
         }*/
         /* Display pdfIn in as line cloud PB END */


  }


  void AdvObjectSearch::GetTotalPDF(){

    // TODO: Do shit.

    double InsideBeforeSum, InsideAfterSum;
    InsideBeforeSum = 0;
    InsideAfterSum = 0;

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        InsideBeforeSum += (*m_pdf_obj)(x, y);
      }
    }
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
          for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
            if ((*m_lgm)(x,y) == 3){
              (*m_pdf_total)(x, y).prob = m_pPlaneGivenObj * (*m_pdf_obj)(x, y)
                          / (m_pPlaneGivenObj * (*m_pdf_obj)(x, y) + m_pPlaneGivenNotObj
                              * (*m_pdf_obj)(x, y));
            }
            else if ((*m_lgm)(x,y) == 0){
              (*m_pdf_total)(x, y).prob = m_pFreeGivenObj * (*m_pdf_obj)(x, y)
                              / (m_pFreeGivenObj * (*m_pdf_obj)(x, y) + m_pFreeGivenNotObj
                                  * (*m_pdf_obj)(x, y));

            }
            else if ((*m_lgm)(x,y) == 1){
              (*m_pdf_total)(x, y).prob = m_pObsGivenObj * (*m_pdf_obj)(x, y)
                              / (m_pObsGivenObj * (*m_pdf_obj)(x, y) + m_pObsGivenNotObj
                                  * (*m_pdf_obj)(x, y));

            }

          }
    }

    // Normalize
       // Get Sum after update
       for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
         for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             InsideAfterSum += (*m_pdf_total)(x, y).prob;
         }
       }

       for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
         for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
           (*m_pdf_total)(x, y).prob = (*m_pdf_total)(x, y).prob * InsideBeforeSum
               / InsideAfterSum;
         }
       }

       

       /* Display pdfIn in PB as Line Cloud BEGIN */

         double multiplier1 = 100.0;
         double xW2, yW2, xW3, yW3;
         peekabot::LineCloudProxy linecloudp;

         linecloudp.add(m_PeekabotClient, "root.LC_TOTAL",
             peekabot::REPLACE_ON_CONFLICT);
         linecloudp.clear_vertices();
         linecloudp.set_line_width(2);
         linecloudp.set_color(0.9, 0, 0);

         for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             if ((*m_lgm)(x, y) == 2 || y == m_lgm->getSize())
               continue;
             m_pdf_total->index2WorldCoords(x, y, xW2, yW2);
             m_pdf_total->index2WorldCoords(x, y + 1, xW3, yW3);
             linecloudp.add_line(xW2 + 6, yW2 - 8,
                 (*m_pdf_total)(x, y).prob * multiplier1, xW3 + 6, yW3 - 8, (*m_pdf_total)(x, y
                     + 1).prob * multiplier1);
           }
         }

        /* for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             if ((*m_lgm)(x, y) == 2 || x == m_lgm->getSize())
               continue;
             m_pdf_total->index2WorldCoords(x + 1, y, xW2, yW2);
             m_pdf_total->index2WorldCoords(x, y, xW3, yW3);
             linecloudp.add_line(xW2 + 6, yW2 - 8,
                 (*m_pdf_total)(x, y).prob * multiplier1, xW3 + 6, yW3 - 8, (*m_pdf_total)(x, y
                     + 1).prob * multiplier1);
           }
         }*/
         /* Display pdfIn in as line cloud PB END */


  }


  /*void
  AdvObjectSearch::PlaneObservationUpdate(
      std::set<std::pair<int, int> > NewPlanePoints) {
    // say plane probability is .8


    /* there are two terms which are now constants
     * one is p(plane|z,c_i)  given an object is at c_i probability
     * of there being a plane. and p(plane|z,c_i^-1) given there isn't
     * any object at c_i probability of there being a table. Plus another term
     * for the probability of there being a plane only from plane observations (pPlane).
     * so this is
     * p(c_i | z,plane) = p(c_i|z)p(plane|ci)
     *                 ---------------------
     *                 p(plane|c_i)p(c_i|z) + p(plane |z, not(c_i))p(not(c_i) | z) for i = 0...N

    log("Plane observation update called! size: %d", NewPlanePoints.size());

    double InsideBeforeSum, InsideAfterSum;
    InsideBeforeSum = 0;
    InsideAfterSum = 0;

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        InsideBeforeSum += (*m_pdf)(x, y).prob;
      }
    }
    log("pdf sums to: %f", InsideBeforeSum);
    log("pdf + Cout sums to: %f", InsideBeforeSum + pOut);

    // For a new plane observation shift probabilities accordingly
    std::pair<int, int> tmp;
    std::set<pair<int, int> >::iterator end = NewPlanePoints.end();
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        tmp.first = x;
        tmp.second = y;
        if (NewPlanePoints.find(tmp) != end) { // this is a new plane point
          (*m_pdf)(x, y).isChecked = true;
          (*m_pdf)(x, y).prob = m_pPlaneGivenObj * (*m_pdf)(x, y).prob
              / (m_pPlaneGivenObj * (*m_pdf)(x, y).prob + m_pPlaneGivenNotObj
                  * (*m_pdf)(x, y).prob);
        }
        else {
          if (((*m_lgm)(x, y) == 0 ||(*m_lgm)(x, y) == 2) && !(*m_pdf)(x, y).isChecked) {
            (*m_pdf)(x, y).isChecked = true;
            (*m_pdf)(x, y).prob = m_pFreeGivenObj * (*m_pdf)(x, y).prob
                / (m_pFreeGivenObj * (*m_pdf)(x, y).prob + m_pFreeGivenNotObj
                    * (*m_pdf)(x, y).prob);
          }
          else if ((*m_lgm)(x, y) == 1 && !(*m_pdf)(x, y).isChecked) {
            (*m_pdf)(x, y).isChecked = true;
            (*m_pdf)(x, y).prob = m_pObsGivenObj * (*m_pdf)(x, y).prob
                / (m_pObsGivenObj * (*m_pdf)(x, y).prob + m_pObsGivenNotObj
                    * (*m_pdf)(x, y).prob);
          }
        }
      }
    }

    // Normalize
    // Get Sum after update
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
          InsideAfterSum += (*m_pdf)(x, y).prob;
      }
    }

    log("aftersum is: %f", InsideAfterSum);
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        (*m_pdf)(x, y).prob = (*m_pdf)(x, y).prob * InsideBeforeSum
            / InsideAfterSum;
      }
    }

  }*/
  void
  AdvObjectSearch::SampleGrid() {
    srand (
    time(NULL));
    std::vector<double> angles;

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

    for (double rad = 0; rad < M_PI * 2; rad = rad + M_PI / 3) {
      angles.push_back(rad);
    }

    int i = 0;
    int randx, randy;
    double xW, yW, angle;
    while (i < m_samplesize) {
      randx = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
      randy = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
      int the = (int) (rand() % angles.size());
      angle = angles[the];
      //if we have that point already, skip.
      for (int j = 0; j < i; j++) {
        if (m_samples[2 * j] == randx && m_samples[2 * j + 1] == randy
            && m_samplestheta[j] == angle) {
          log("we already have this point.");
          continue;
        }
      }
      m_lgm->index2WorldCoords(randx, randy, xW, yW);

      if ((*m_lgm)(randx, randy) == 0 && !(*m_pdf_total)(randx, randy).isSeen) {
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
            m_samplestheta[i] = angle;
            i++;
          }
          else {
            //  log("there's no path to here.");
          }
          /*if reachable*/
        }

      }
      else {
        //log("point either non free space or seen.");
      }
    }

    /* Display Samples in PB BEGIN */
    double color[3] =
      { 1.0, 0, 0 };
    double xW1, yW1;
    peekabot::PointCloudProxy samples;
    samples.add(m_PeekabotClient, "root.samples", peekabot::REPLACE_ON_CONFLICT);
    samples.set_color(color[0], color[1], color[2]);
    //samples.set_opacity(0.2);

    for (int i = 0; i < m_samplesize; i++) {

      m_lgm->index2WorldCoords(m_samples[i * 2], m_samples[i * 2 + 1], xW1, yW1);
      //log("sample coord: %f, %f, %f", xW1,yW1,m_samplestheta[i]);
      samples.add_vertex(xW1, yW1, 2);
    }
    /* Display Samples in PB END */

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
      a.theta = m_samplestheta[y];
      tpoints = GetInsideViewCone(a, true);
      ViewConePts.push_back(tpoints);
      candidatePose.setX(a.x);
      candidatePose.setY(a.y);
      candidatePose.setTheta(m_samplestheta[y]);
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

  void
  AdvObjectSearch::PostNavCommand(Cure::Pose3D position) {
    SpatialData::NavCommandPtr cmd = newNavCommand();
    cmd->prio = SpatialData::URGENT;
    cmd->cmd = SpatialData::GOTOPOSITION;
    cmd->pose.resize(3);
    cmd->pose[0] = position.getX();
    cmd->pose[1] = position.getY();
    cmd->pose[2] = position.getTheta();
    cmd->tolerance.resize(1);
    cmd->tolerance[0] = 0.1;
    cmd->status = SpatialData::NONE;
    cmd->comp = SpatialData::COMMANDPENDING;

    addToWorkingMemory(newDataID(), cmd);
  }

  SpatialData::NavCommandPtr
  AdvObjectSearch::newNavCommand() {
    SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
    cmd->prio = SpatialData::NORMAL;
    cmd->cmd = SpatialData::STOP;
    cmd->status = SpatialData::NONE;
    cmd->comp = SpatialData::COMMANDPENDING;
    return cmd;
  }

}
