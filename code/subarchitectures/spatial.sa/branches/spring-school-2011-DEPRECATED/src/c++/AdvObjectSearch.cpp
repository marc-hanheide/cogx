/*
 * AdvObjectSearch.cpp
 *
 *  Created on: Feb 15, 2010
 *      Author: aydemir
 */

#include "AdvObjectSearch.hpp"
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

    if (cfg.getSensorPose(1, m_LaserPoseR)) {
      println("configure(...) Failed to get sensor pose");
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
    m_gridsize = 400;
    m_cellsize = 0.1;
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

    /* if ((it = _config.find("--objects")) != _config.end()) {
       istringstream istr(it->second);
       string label;
       while (istr >> label) {
       m_objectlist.push_back(label);
       }
       }
       log("Loaded objects.");*/

    m_objectlist.push_back("rice");
    m_objectlist.push_back("printer");
    m_objectlist.push_back("squaretable");

    gotDistribution = false;
    PDFData def;
    def.prob = 0;
    def.isSeen = false;
    def.isChecked = false;

    m_lgm = new Cure::LocalGridMap<unsigned int>(m_gridsize, m_cellsize, 2,
	Cure::LocalGridMap<unsigned int>::MAP1);

    m_pdf = new Cure::LocalGridMap<PDFData>(m_gridsize, m_cellsize, def,
	Cure::LocalGridMap<unsigned int>::MAP1);

    m_Dlgm = new Cure::X11DispLocalGridMap<unsigned int>(*m_lgm);
    m_Glrt = new Cure::ObjGridLineRayTracer<unsigned int>(*m_lgm);

    m_ProbGivenObjectIsPresent = 0.7;

    m_pPlaneGivenObj = 0.7;
    m_pFreeGivenObj = 0.05;
    m_pObsGivenObj = 0.25;

    m_pPlaneGivenNotObj = 0.05;
    m_pFreeGivenNotObj = 0.8;
    m_pObsGivenNotObj = 0.15;

    isWaitingForDetection = false;

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

    addChangeFilter(
        createLocalTypeFilter<FrontierInterface::ObjectPriorRequest> (
            cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::owtGridMapDouble));

    addChangeFilter(createLocalTypeFilter<
        FrontierInterface::ObjectTiltAngleRequest> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
            &AdvObjectSearch::owtTiltAngleRequest));

    addChangeFilter(createLocalTypeFilter<
            SpatialData::NavCommand> (cdl::OVERWRITE),
            new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
                &AdvObjectSearch::owtNavCommand));

    addChangeFilter(createGlobalTypeFilter<
               VisionData::Recognizer3DCommand> (cdl::OVERWRITE),
               new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
                   &AdvObjectSearch::owtRecognizer3DCommand));

    addChangeFilter(createGlobalTypeFilter<
               VisionData::VisualObject> (cdl::OVERWRITE),
               new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
                   &AdvObjectSearch::newVisualObject));

    addChangeFilter(createGlobalTypeFilter<
               VisionData::VisualObject> (cdl::ADD),
               new MemberFunctionChangeReceiver<AdvObjectSearch> (this,
                   &AdvObjectSearch::newVisualObject));

  }


  void
   AdvObjectSearch::newVisualObject(const cast::cdl::WorkingMemoryChange &objID) {

    if (isWaitingForDetection){
    try{
      log("new visual object");
       VisionData::VisualObjectPtr visualobject(getMemoryEntry<
           VisionData::VisualObject> (objID.address));

       if (visualobject->label == m_CurrentTarget){
         isWaitingForDetection = false;
         DetectionComplete(true);
       }

    }
    catch (const CASTException &e) {
          //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
        }
    }
  }


  void
   AdvObjectSearch::owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID) {
    if(isWaitingForDetection){
      try{
        log("got recognizer3D overwrite command");
        VisionData::Recognizer3DCommandPtr cmd(getMemoryEntry<
            VisionData::Recognizer3DCommand> (objID.address));
        if (cmd->label == m_CurrentTarget){
          isWaitingForDetection = false;
          DetectionComplete(false);
        }

    }
      catch (const CASTException &e) {
        //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
          }
    }
  }

void AdvObjectSearch::DetectionComplete(bool isDetected){
  m_gotDetectionResult = true;
  if (isDetected){
    log("object detected");
    m_isDetected = true;
    // if we are doing indirect search then ask & initialize next object
    if (m_CurrentTarget == "rice"){
      log("Object Detected, Mission Completed.");
      m_command = IDLE;
    }
    else if (m_SearchMode == "indirect" && m_CurrentTarget != "rice"){
      log("detected, changing current target to rice, asking for distribution.");
      m_CurrentTarget = "rice";
      m_command = ASK_FOR_DISTRIBUTION;
    }
    else if (m_SearchMode == "uniform" && m_CurrentTarget == "printer"){
      log("detected, changing current target to rice, asking for distribution.");
      m_CurrentTarget = "rice";
      m_command = ASK_FOR_DISTRIBUTION;
    }

    }
  else{
    // if we are not yet finished Go to NBV
    log("object not detected");
    MeasurementUpdate(false);
    m_command = EXECUTENEXT;
  }
}

  void
   AdvObjectSearch::owtNavCommand(const cast::cdl::WorkingMemoryChange &objID) {

    try{
      log("nav command overwritten");
    SpatialData::NavCommandPtr cmd(getMemoryEntry<
        SpatialData::NavCommand> (objID.address));
    if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
         log("NavCommand succeeded.");
	 if (m_SearchMode == "uniform" && m_CurrentTarget == "printer")
	   m_totalViewPoints = m_totalViewPoints + 3;
	 else
	   m_totalViewPoints++;

	 log("reached %i th viewpoint", m_totalViewPoints);
         log("Modifying seen map");
         std::pair<int, int> vp;
         vp.first = m_samples[2 * m_CurrentViewPointIndex];
         vp.second = m_samples[2 * m_CurrentViewPointIndex + 1];
         VisitedVPs.insert(vp);
         int x,y;
         for (unsigned int j = 0; j < m_VCones[m_CurrentViewPointIndex].size() / 2; j++) {
           x = m_VCones[m_CurrentViewPointIndex][2 * j];
           y = m_VCones[m_CurrentViewPointIndex][2 * j + 1];
           if (!(*m_pdf)(x, y).isSeen)
             (*m_pdf)(x, y).isSeen = true;
         }

         m_command = RECOGNIZE;
       }
    else if (cmd->comp == SpatialData::COMMANDFAILED){
      log("NavCommand failed.Getting next view.");
      m_command=EXECUTENEXT;
    }
    }
    catch (const CASTException &e) {
       //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());

     }
  }

  void
  AdvObjectSearch::owtTiltAngleRequest(
      const cast::cdl::WorkingMemoryChange &objID) {
    try {
      log("got tilt angles");
      FrontierInterface::ObjectTiltAngleRequestPtr tiltreq = getMemoryEntry<
          FrontierInterface::ObjectTiltAngleRequest> (objID.address);
      log("size: %d", tiltreq->tiltAngles.size());
      for (unsigned int i = 0; i < tiltreq->tiltAngles.size(); i++) {
        //log("tilt angle: %f", tiltreq->tiltAngles[i]);
        VPDistribution.push_back(tiltreq->tiltAngles[i]);
      }
      gotTiltAngles = true;
      log("out of tilt angles");
    }
    catch (DoesNotExistOnWMException excp) {
      log("Error!  GridMapDouble does not exist on WM!");
      return;
    }

  }

  void
  AdvObjectSearch::owtGridMapDouble(const cast::cdl::WorkingMemoryChange &objID) {
//    try {
//      log("GridMapDouble overwrite!");
//      Cure::LocalGridMap<double>* distribution =
//	new Cure::LocalGridMap<double>(m_gridsize, m_cellsize, 0.0,
//	    Cure::LocalGridMap<unsigned int>::MAP1);
//
//      // get the computed distribution.
//      FrontierInterface::ObjectPriorRequestPtr objreq = getMemoryEntry<
//	FrontierInterface::ObjectPriorRequest> (objID.address);
//      convertToCureMap(objreq->outMap, *distribution);
//
//      // Normalise over the cells which we are keeping (explored)
//      double sumin = 0.0;
//      for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
//	for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
//	  if ((*m_lgm)(x, y) == 2)
//	    continue;
//	  sumin += (*distribution)(x, y);
//	}
//      }
//
//      log("Distribution sums to: %f", sumin);
//      if (sumin != 0.0) {
//	double normFactor = 0.5*pIn/sumin;
//	double normalizedSum = 0.0;
//	//sum with m_pdf
//	for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
//	  for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
//	    if ((*m_lgm)(x, y) == 2)
//	      continue;
//	    (*m_pdf)(x, y).prob = (*m_pdf)(x, y).prob + 
//	      normFactor*(*distribution)(x, y);
//	    normalizedSum += 
//	      normFactor*(*distribution)(x, y);
//	    //if ( (*distribution)(x,y) != 0 )
//	    //    log("pdf:%f dist:%f",(*m_pdf)(x,y).prob, (*distribution)(x,y));
//	  }
//	}
//	log("Normalised probability sum added: %f", normalizedSum);
//
//	/* Display PDF in PB as Line Cloud BEGIN */
//
//	double xW2, yW2;
//	peekabot::PointCloudProxy linecloudp;
//
//	linecloudp.add(m_PeekabotClient, "root.distribution",
//	    peekabot::REPLACE_ON_CONFLICT);
//	linecloudp.clear_vertices();
//	linecloudp.set_color(0.5, 0, 0.5);
//
//	for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
//	  for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
//	    if ((*distribution)(x, y) == 0)
//	      continue;
//	    m_lgm->index2WorldCoords(x, y, xW2, yW2);
//	    linecloudp.add_vertex(xW2, yW2, (*distribution)(x, y)*2);
//	  }
//	}
//	/* Display pdfIn in as line cloud PB END */
//
//	//     DisplayPDFinPB(0,0,"root.distribution");
//	log("Displayed PDF");
//	// Once we get a distribution initiate search.
//	if (objreq->objects.back() == "squaretable"){
//	  gotSquareTable = true;
//	}
//	if (objreq->objects.back() == "desk" || objreq->objects.back() == "printer" || (m_SearchMode == "uniform" && m_CurrentTarget == "rice")){
//	  m_command = EXECUTENEXT;
//	}
//      }
//
//    }
//    catch (DoesNotExistOnWMException excp) {
//      log("Error!  GridMapDouble does not exist on WM!");
//      return;
//    }
//
  }

  void AdvObjectSearch::ResetSeenMap(){
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
            for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
                (*m_pdf)(x, y).isSeen = false;
            }
            }
  }
  void
  AdvObjectSearch::runComponent() {
    setupPushScan2d(*this, 0.1);
    setupPushOdometry(*this);
    m_command = IDLE;

    try {
      m_PeekabotClient.connect("localhost", 5050, true);
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
      //lockComponent();
      m_Dlgm->updatePlaneDisplay(&m_SlamRobotPose);

      int cvkey = cvWaitKey(100);
      char key = char(cvkey);
      if (cvkey != -1){
      if (key == 's') {
        m_table_phase = true;
        log("Saving plane map!");
        SavePlaneMap();
        cvReleaseImage(&img);
      }
      else if (key == 't') { // t
        log("Table mode!");

        m_table_phase = true;

      }
      else if (key == 'g'){ // g
        log("sampling grid!");
        SampleGrid();

      }
      else if (key == 'p') { // p
        if (gotDistribution) {
          log("Getting next view");
          GoToNBV();
        }
        else {
          log("spatial relation distribution not yet acquired.");

        }
      }
      else if (key == 'v') { // v
        addRecognizer3DCommand(VisionData::RECOGNIZE,"rice","");
      }
      else if (key == 'w'){ // w
        addRecognizer3DCommand(VisionData::RECOGNIZE,"printer","");
      }
      else if (key == 'u') { // u
        ResetSeenMap();
	m_totalViewPoints = 0;
        m_SearchMode = "uniform";
	m_CurrentTarget = "printer";
        VisitedVPs.clear();
        m_table_phase = false;
        log("Uniform search!");
        log("Reading plane map!");
        ReadPlaneMap();
        m_command = ASK_FOR_DISTRIBUTION;
      }
      else if (key == 'd') { // d
        m_SearchMode = "direct";
        ResetSeenMap();
	m_totalViewPoints = 0;
        VisitedVPs.clear();
	pIn= 0.8;
	pOut = 0.2;
        log("Direct search");
        log("Reading plane map!");
        m_CurrentTarget = "rice";
        ReadPlaneMap();
        m_command = ASK_FOR_DISTRIBUTION;
      }
      else if (key == 'i') { // i
	pIn = 0.8;
	pOut = 0.2;
        ResetSeenMap();
        VisitedVPs.clear();
	m_totalViewPoints = 0;
        m_SearchMode = "indirect";
        log("Indirect search");
        log("Reading plane map!");
        m_CurrentTarget = "printer";
        ReadPlaneMap();
        m_command = ASK_FOR_DISTRIBUTION;
      }
      else {
		log("Unknown key %c",char(key));
      	}
      }
      InterpretCommand();
      sleepComponent(100);

    }
  }

  void AdvObjectSearch::InterpretCommand(){
    switch (m_command)
        {
      case STOP: {
        log("Stopped.");
        m_command = IDLE;
        m_status = STOPPED;
        log("Command: STOP");
        Cure::Pose3D pos;
        PostNavCommand(pos, SpatialData::STOP);
        break;
      }
      case RECOGNIZE:{
        m_command=IDLE;
        Recognize();
	break;
      }
      case EXECUTENEXT:{
        m_command=IDLE;
        if (isStopSearch(0.7)){
          log("Search failed.");
        }
        else{
          GoToNBV();
        }
	break;
      }
      case ASK_FOR_DISTRIBUTION: {
        log("asking for distribution");
	m_command=IDLE;
        if (m_SearchMode =="direct"){
	  log("asking prior: direct");
          AskForDistribution(m_objectlist,pIn);
	}
        else if (m_SearchMode =="indirect" && m_CurrentTarget == "rice"){
          log("asking prior: indirect, rice");
          std::vector<std::string> objects;
          objects.push_back("rice");
          objects.push_back("printer");
          AskForDistribution(objects,pIn);
        }
        else if (m_SearchMode == "indirect" && m_CurrentTarget == "printer"){
          log("asking prior: indirect,printer");
	  std::vector<std::string> objects;
          objects.push_back("printer");
          objects.push_back("squaretable");
          AskForDistribution(objects,pIn);
        }
        else if (m_SearchMode == "uniform" && m_CurrentTarget == "printer"){
          int numberoffree = 0;
          int numberofobs = 0;
          for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
                    for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
                      if ((*m_lgm)(x,y) == 2)
                        continue;
                      if ((*m_lgm)(x,y) == 0){
                        numberoffree++;
                      }
                      else  if ((*m_lgm)(x,y) == 1){
                        numberofobs++;
                      }



                    }
          }
          double pFree = pIn*0.3/numberoffree;
          double pObs  = pIn*0.7/numberofobs;
          for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
                           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
                               if ((*m_pdf)(x,y).prob == 2)
                                 continue;
                               if ((*m_lgm)(x,y) == 0){
                                 (*m_pdf)(x,y).prob = pFree;
                               }
                               else  if ((*m_lgm)(x,y) == 1){
                                 (*m_pdf)(x,y).prob = pObs;
                                                }
                 }
          }
          m_command=EXECUTENEXT;
        }
        else if (m_SearchMode == "uniform" && m_CurrentTarget == "rice"){
	  ResetSeenMap();
          std::vector<std::string> objects;
          objects.push_back("rice");
          objects.push_back("printer");
          AskForDistribution(objects,pIn);

        }
	break;
      }
      case IDLE: {
        //log("Idling");
	break;
      }

  }
  }

  void AdvObjectSearch::Recognize(){
    m_gotDetectionResult = false;
    m_isDetected = false;
    if (m_SearchMode == "uniform" && m_CurrentTarget == "printer"){
      MovePanTilt(0, -0.5236, 0.08);
      addRecognizer3DCommand(VisionData::RECOGNIZE,m_CurrentTarget,"");
      while(!m_gotDetectionResult)
        sleepComponent(500);
      if(m_isDetected){
	if (m_CurrentTarget == "printer")
	  m_command = IDLE;
	return;
      }
      m_gotDetectionResult = false;
      MovePanTilt(0, 0, 0.08);
      addRecognizer3DCommand(VisionData::RECOGNIZE,m_CurrentTarget,"");
      while(!m_gotDetectionResult)
	sleepComponent(500);
      if(m_isDetected){
	if (m_CurrentTarget == "printer")
	  m_command = IDLE;
	return;
      }
      m_gotDetectionResult = false;
      MovePanTilt(0,0.5236, 0.08);
      addRecognizer3DCommand(VisionData::RECOGNIZE,m_CurrentTarget,"");
      return;
    }
    
    log("waiting for tilt angles");
    while(!gotTiltAngles){
      sleepComponent(500);
    }

    //Calculate necessary tilt angle
    double tolerance = 0.08; // 5 degrees in rad.
    double d1,d2,tilt;
    cogx::Math::Vector3 p1;
    p1.x = lastRobotPose->x;
    p1.y = lastRobotPose->y;
    p1.z = 1.4; //camera is 1.4 high
    tilt = 0;
    for (unsigned int i=0; i < VPDistribution.size();i++){
      //log("tilt dist: %f,%f,%f",VPDistribution[i].x,VPDistribution[i].y,VPDistribution[i].z); 
      d1 = p1.z  - VPDistribution[i].z;
      d2 = sqrt(pow(p1.x - VPDistribution[i].x,2) + pow(p1.y - VPDistribution[i].y,2));
      tilt += atan(d1/d2);
    }
    tilt = tilt / VPDistribution.size();
    log("tilt angle is: %f",tilt);
    MovePanTilt(0, -tilt, tolerance);
    addRecognizer3DCommand(VisionData::RECOGNIZE,m_CurrentTarget,"");
  }

  void
  AdvObjectSearch::MovePanTilt(double pan, double tilt, double tolerance) {
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
        //m_PTUServer->setPose(p);
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
          if (actualtilt < (tilt + tolerance) && actualtilt > (tilt - tolerance)) {
            run = false;
          }
          else {
            //if the previous check fails, loop again
            run = true;
          }
        }

        usleep(10000);
      }
      log("Moved.");
      sleep(1);
    }
  }

  bool AdvObjectSearch::isStopSearch(double threshold){
    log("pOut: %f", pOut);
    if (pOut > threshold)
      return true;
    else
      return false;
  }
  void
  AdvObjectSearch::AskForDistribution(std::vector<std::string> objectlist, double probSum) {
    log("asking for distribution: %s , %s",m_SearchMode.c_str(), m_CurrentTarget.c_str());
    gotSquareTable = false;
    // initialize m_lgm to zero
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
           for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
             (*m_pdf)(x, y).prob = 0;
           }
    }

    vector<FrontierInterface::ObjectRelation> relations;
    relations.resize(objectlist.size()-1, FrontierInterface::ON);

    FrontierInterface::WeightedPointCloudPtr queryCloud
      = new FrontierInterface::WeightedPointCloud;

    //write lgm to WM
    FrontierInterface::ObjectPriorRequestPtr objreq =
        new FrontierInterface::ObjectPriorRequest;
    objreq->relationTypes = relations;
    objreq->objects = objectlist;
    objreq->cellSize = m_cellsize;
    objreq->outCloud = queryCloud;
//    objreq->probSum = probSum/2; // FIXME
//    objreq->outMap = convertFromCureMap(*tobefilled);
    addToWorkingMemory(newDataID(), objreq);


    if (!(m_SearchMode == "indirect" && m_CurrentTarget == "rice") && m_SearchMode != "uniform"){
      log("waiting for squaretable");
      while(!gotSquareTable)
      sleepComponent(300);

      log("got squaretable distrib, now switching to desk");
      objectlist.back() = "desk";
      objreq->relationTypes = relations;
      objreq->objects = objectlist;
      objreq->cellSize = m_cellsize;
      objreq->outCloud = queryCloud;
//      objreq->probSum = probSum/2;
//      objreq->outMap = convertFromCureMap(*tobefilled);
      addToWorkingMemory(newDataID(), objreq);
    }
  }
  void
  AdvObjectSearch::ReadPlaneMap() {
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
  }
  void
  AdvObjectSearch::GoToNBV() {
    gotDistribution = false;
    int nbv;
    log("Getting NextBestView");
    nbv = NextBestView();
    m_CurrentViewPointIndex = nbv;
    double Wx, Wy;
    m_lgm->index2WorldCoords(m_samples[2 * nbv], m_samples[2 * nbv + 1], Wx, Wy);
    log("Best view coords: %f, %f, %f", Wx, Wy, m_samplestheta[nbv]);

if (!(m_SearchMode == "uniform" && m_CurrentTarget == "printer")){
    /******* Get tilt angles  to calculate desired tilt value*/
  log("asking for tilt angles");
    std::vector<std::string> objects;
    if (m_SearchMode =="direct"){
      log("asking prior: direct");
      objects = m_objectlist;
    }
    else if (m_SearchMode =="indirect" && m_CurrentTarget == "rice"){
      log("asking prior: indirect, rice");
      objects.push_back("rice");
      objects.push_back("printer");
    }
    else if (m_SearchMode == "indirect" && m_CurrentTarget == "printer"){
      log("asking prior: indirect,printer");
      objects.push_back("printer");
      objects.push_back("squaretable");
    }
    else if (m_SearchMode=="uniform" && m_CurrentTarget == "rice"){
      objects.push_back("rice");
      objects.push_back("printer");
    }

    gotTiltAngles = false;
    VPDistribution.clear();
    XVector3D a, b, c;
    a.x = Wx;
    a.y = Wy;
    CalculateViewCone(a, a.theta, m_CamRange, m_fov, b, c);

    cogx::Math::Vector2 point;
    vector<cogx::Math::Vector2> triangle;
    point.x = a.x;
    point.y = a.y;
    triangle.push_back(point);
    point.x = b.x;
    point.y = b.y;
    triangle.push_back(point);
    point.x = c.x;
    point.y = c.y;
    triangle.push_back(point);

    FrontierInterface::ObjectTiltAngleRequestPtr req =
        new FrontierInterface::ObjectTiltAngleRequest;

    req->relationType = FrontierInterface::ON;
    req->objects = objects;
    req->triangle = triangle;
    addToWorkingMemory(newDataID(), req);
    //////////////////////////////////////
}
    Cure::Pose3D pos;
    pos.setX(Wx);
    pos.setY(Wy);
    pos.setTheta(m_samplestheta[nbv]);
    log("posting nav command");
    PostNavCommand(pos, SpatialData::GOTOPOSITION);

    /* Add plan to PB BEGIN */
        NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
        cogx::Math::Vector3 viewpoint;
        viewpoint.x = Wx;
        viewpoint.y = Wy;
        viewpoint.z = m_samplestheta[nbv];
        obs->planlist.push_back(viewpoint);
        addToWorkingMemory(newDataID(), obs);
     /* Add plan to PB END */

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
      //log("got new robotpose");
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

    if (m_CurrentTarget == "rice")
        m_CamRange = 1.0;
      else
        m_CamRange = 4.0;

   // std::vector<std::vector<int> > VCones;
    m_VCones.clear();
    log("sampling grid");
    SampleGrid();
    log("getting view cones");
    m_VCones = GetViewCones();
    double highest_sum = -10000.0;
    double sum;
    int highest_VC_index = 0;
    int x, y;
    for (unsigned int i = 0; i < m_VCones.size(); i++) {
      sum = 0;
      for (unsigned int j = 0; j < m_VCones[i].size() / 2; j++) {
        x = m_VCones[i][2 * j];
        y = m_VCones[i][2 * j + 1];
        sum += (*m_pdf)(x, y).prob;
      }
      if (sum > highest_sum) {
        highest_sum = sum;
        highest_VC_index = i;
      }
    }


    /* Display SeenMap in PB BEGIN */
    double color[3] =
      { 0.5, 0.5, 0.5 };
    double xW, yW;
    m_ProxySeenMap.clear_vertices();
    m_ProxySeenMap.set_color(color[0], color[1], color[2]);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_pdf)(x, y).isSeen == true || (*m_lgm)(x, y) == 2)
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
        if ((*m_lgm)(CoordPair.first, CoordPair.second) != 3 && !(*m_pdf)(
            CoordPair.first, CoordPair.second).isChecked) {
          NewPlanePoints.insert(CoordPair);
        }

        (*m_lgm)(CoordPair.first, CoordPair.second) = 3;

        // TODO: do not initialize PDF with fixed value if on the fly adding of planes is on.
      }
      if (NewPlanePoints.size() > 0)
        PlaneObservationUpdate(NewPlanePoints);
    }
    catch (DoesNotExistOnWMException) {
      log("Error! plane point cloud disappeared from WM.");
    }
  }

  void
  AdvObjectSearch::DisplayPDFinPB(double xoffset, double yoffset,
      std::string name) {

    //    double uUnit = pIn / pow(double((2 * m_lgm->getSize() + 1)), 2);

    /* Display PDF in PB as Line Cloud BEGIN */

    double multiplier1 = 500.0;
    double xW2, yW2, xW3, yW3;
    peekabot::LineCloudProxy linecloudp;

    linecloudp.add(m_PeekabotClient, name, peekabot::REPLACE_ON_CONFLICT);
    linecloudp.clear_vertices();
    linecloudp.set_line_width(2);
    linecloudp.set_color(0.9, 0, 0);

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y < m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2 || y == m_lgm->getSize())
          continue;
        m_lgm->index2WorldCoords(x, y, xW2, yW2);
        m_lgm->index2WorldCoords(x, y + 1, xW3, yW3);
        linecloudp.add_line(xW2 + xoffset, yW2 + yoffset, (*m_pdf)(x, y).prob
            * multiplier1, xW3 + xoffset, yW3 + yoffset,
            (*m_pdf)(x, y + 1).prob * multiplier1);
      }
    }

    for (int x = -m_lgm->getSize(); x < m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2 || x == m_lgm->getSize())
          continue;
        m_lgm->index2WorldCoords(x, y, xW2, yW2);
        m_lgm->index2WorldCoords(x + 1, y, xW3, yW3);
        linecloudp.add_line(xW2 + xoffset, yW2 + yoffset, (*m_pdf)(x, y).prob
            * multiplier1, xW3 + xoffset, yW3 + yoffset,
            (*m_pdf)(x + 1, y).prob * multiplier1);
      }
    }
    /* Display pdfIn in as line cloud PB END */

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
        //log("ActionProbcell returned %f", 0.7);
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

    //DisplayPDFinPB(6, 0, "root.before");

    /* DEBUG */
    double sumin = 0.0;
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2)
          continue;
        sumin += (*m_pdf)(x, y).prob;
      }
    }
    log("pdfIn sums to: %f", sumin);
    log("pdfIn + Cout sums to: %f", sumin + pOut);

    /* DEBUG */
    log("current vp index %d , cones size %d", m_CurrentViewPointIndex,m_VCones.size());
    double denomsum = 0.0;
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2)
          continue;
        denomsum += (*m_pdf)(x, y).prob * (1 - ActionProbabilityPerCell(x, y,
            m_VCones[m_CurrentViewPointIndex]));
      }
    }
    denomsum += pOut;

    log("pOut is: %f", pOut);
    log("denomsum is: %f", denomsum);
    // For everything inside meaning: pIn
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2)
          continue;
        //if ((*m_lgm)(x,y) == 0)
        //log("old prob for fs: %f", (*m_pdf)(x,y).prob);
        (*m_pdf)(x, y).prob = ((*m_pdf)(x, y).prob * (1
            - ActionProbabilityPerCell(x, y, m_VCones[m_CurrentViewPointIndex])))
            / denomsum;
        //if ((*m_lgm)(x,y) == 0)
        //log("new prob for fs: %f", (*m_pdf)(x,y).prob);
      }
    }

    pOut = pOut / denomsum;
    log("pOut has become: %f", pOut);

    /* DEBUG */
    sumin = 0.0;
    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
        if ((*m_lgm)(x, y) == 2)
          continue;
        sumin += (*m_pdf)(x, y).prob;
      }
    }
    log("pdfIn sums to: %f", sumin);
    log("pdfIn + Cout sums to: %f", sumin + pOut);
    /* DEBUG */
    DisplayPDFinPB(0, -8, "root.after");

  }

  void
  AdvObjectSearch::PlaneObservationUpdate(
      std::set<std::pair<int, int> > NewPlanePoints) {

    /* there are two terms which are now constants
     * one is p(plane|z,c_i)  given an object is at c_i probability
     * of there being a plane. and p(plane|z,c_i^-1) given there isn't
     * any object at c_i probability of there being a table. Plus another term
     * for the probability of there being a plane only from plane observations (pPlane).
     * so this is
     * p(c_i | z,plane) = p(c_i|z)p(plane|ci)
     *                 ---------------------
     *                 p(plane|c_i)p(c_i|z) + p(plane |z, not(c_i))p(not(c_i) | z) for i = 0...N
     */
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
        if (((*m_lgm)(x, y) == 2))
          continue;

        tmp.first = x;
        tmp.second = y;
        if (NewPlanePoints.find(tmp) != end) { // this is a new plane point
          (*m_pdf)(x, y).isChecked = true;
          (*m_pdf)(x, y).prob = m_pPlaneGivenObj * (*m_pdf)(x, y).prob
              / (m_pPlaneGivenObj * (*m_pdf)(x, y).prob + m_pPlaneGivenNotObj
                  * (*m_pdf)(x, y).prob);
        }
        else {
          if ((*m_lgm)(x, y) == 0 && !(*m_pdf)(x, y).isChecked) {
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

  }

bool 
isCircleFree(const Cure::LocalGridMap<unsigned int> &map, double xW, double yW, double rad)
{
  int xiC, yiC;
  if (map.worldCoords2Index(xW, yW, xiC, yiC) != 0) {
    CureCERR(30) << "Querying area outside the map (xW="
                 << xW << ", yW=" << yW << ")\n";
    return true;
  }

  double w = rad / map.getCellSize();
  int wi = int(w + 0.5);
  int size = map.getSize();

  for (int x = xiC-wi; x <= xiC+wi; x++) {
    for (int y = yiC-wi; y <= yiC+wi; y++) {
      if (x >= -size && x <= size && y >= -size && y <= size) {
        if (hypot(x-xiC,y-yiC) < w) {
          if (map(x,y) != 0) return false;
        }
      }
    }
  }

  return true;
}
  void
  AdvObjectSearch::SampleGrid() {
    srand (
    time(NULL));
    std::vector<double> angles;

    log("Sampling Grid");
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
    //log("for sample size");
    bool haspoint;
    while (i < m_samplesize) {
      haspoint = false;
      randx = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
      randy = (rand() % (2 * m_lgm->getSize())) - m_lgm->getSize();
      int the = (int) (rand() % angles.size());
      angle = angles[the];
      //if we have that point already, skip.
      for (int j = 0; j < i; j++) {
        if (m_samples[2 * j] == randx && m_samples[2 * j + 1] == randy
            && m_samplestheta[j] == angle) {
//          log("we already have this point.");
          haspoint = true;
          break;
        }
      }
      m_lgm->index2WorldCoords(randx, randy, xW, yW);
      std::pair<int,int> sample;
      sample.first = randx;
      sample.second = randy;

      if (VisitedVPs.find(sample) != VisitedVPs.end() && VisitedVPs.size() != 0){
//        log("we already checked this viewpoint");
        continue;
      }

      if (!haspoint && (*m_lgm)(randx, randy) == 0 && isCircleFree(*m_lgm, xW, yW, 0.4)) {
        /*if reachable*/
        // Get the indices of the destination coordinates
        //	log("point reachable");
        int rS, cS, rE, cE;
        //log("robotpose: %f,%f", lastRobotPose->x,lastRobotPose->y);
        //log("1");
        if (m_lgm->worldCoords2Index(lastRobotPose->x, lastRobotPose->y, rS, cS)
            == 0 && m_lgm->worldCoords2Index(xW, yW, rE, cE) == 0) {
          //log("check if point is reachable");
          // Compensate for the fact that the PathGrid is just a normal matrix where the cells are numbers from the corner
          cS += m_lgm->getSize();
          rS += m_lgm->getSize();
          cE += m_lgm->getSize();

          rE += m_lgm->getSize();

          Cure::ShortMatrix path;
          double d = (m_PathGrid.path(rS, cS, rE, cE, path, 20
              * m_lgm->getSize()) * m_lgm->getCellSize());

          if (d > 0) {
            // There is a path to this destination
            //log("there's a path to this destination");
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
    //log("Displaying VP samples in PB");

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
  AdvObjectSearch::PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype) {
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

    addToWorkingMemory(newDataID(), cmd);
    log("posted nav command");
  }

  void 
  AdvObjectSearch::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, 
      std::string label, std::string visualObjectID){
    log("posting recognizer command.");
    VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
    rec_cmd->cmd = cmd;
    rec_cmd->label = label;
    rec_cmd->visualObjectID = visualObjectID;
    addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
    isWaitingForDetection = true;
  }
}
