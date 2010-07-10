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

namespace spatial
{   
  using namespace cast;
  using namespace spatial;
  using namespace std;
  using namespace boost;
  using namespace SpatialGridMap;
  extern "C"
  { 
    cast::CASTComponentPtr
      newComponent() {
	return new VisualObjectSearch();
      }
  } 
  VisualObjectSearch::VisualObjectSearch() {
    // TODO Auto-generated constructor stub
    // If we're not building the map it means we're using an already built one. Hence, read it.		         
  }
  VisualObjectSearch::~VisualObjectSearch() {
    // TODO Auto-generated destructor stub
    log("Destructor called.");
  }


  spatial::VisualObjectSearch* AVSComponentPtr;
  void VisualObjectSearch::configure(const std::map<std::string, std::string>& _config){

    isSearchFinished = false;
    AVSComponentPtr = this;
    map<string,string>::const_iterator it = _config.find("-c");
    if (it== _config.end()) {
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

    if (cfg->getSensorPose(1, m_LaserPoseR)) {
      println("configure(...) Failed to get sensor pose for laser");
      std::abort();
    }

    m_mapceiling= 3.0;
    it = _config.find("--mapceiling");
    if (it != _config.end()) {
      m_mapceiling = (atof(it->second.c_str()));
      log("Map ceiling set to: %d", m_mapceiling);
    }

    m_samplesize = 100;
    it = _config.find("--samplesize");
    if (it != _config.end()) {
      m_samplesize = (atof(it->second.c_str()));
      log("Samplesize set to: %d", m_samplesize);
    }

    m_samples = new int[2 * m_samplesize];
    m_samplestheta = new double[m_samplesize];
    
    
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


    m_horizangle = M_PI/4;
    it = _config.find("--cam-horizangle");
    if (it != _config.end()) {
      m_horizangle = (atof(it->second.c_str()))*M_PI/180.0;
      log("Camera FoV horizontal angle set to: %f", m_horizangle);
    }


    m_vertangle = M_PI/4;
    it = _config.find("--cam-vertangle");
    if (it != _config.end()) {
      m_vertangle = (atof(it->second.c_str()));
      log("Camera FoV vertical angle set to: %f", m_vertangle);
    }


    m_conedepth = 2.0;
    it = _config.find("--cam-conedepth");
    if (it != _config.end()) {
      m_conedepth = (atof(it->second.c_str()));
      log("Camera view cone depth set to: %f", m_conedepth);
    }
    m_minDistance = m_conedepth / 4.0;
    if (m_minDistance < 0.5) m_minDistance = 0.5;

    m_savemapmode = false;
    it = _config.find("--savemap");
    if (it != _config.end()) {
      m_savemapmode = true; 
      if(m_savemapmode)
	log("Save map mode : on");
      else
	log("Save map mode : off");
    }
    m_usePTZ = false;
    if (_config.find("--ctrl-ptu") != _config.end()) {
      m_usePTZ = true;
      log("will use ptu");
    }
    string filename;
    if ((it = _config.find("--relations-file")) != _config.end()) {
      istringstream istr(it->second);
      istr >> filename;
    }
    LoadSpatialRelations(filename);

    m_curemapfile = "";
    if ((it = _config.find("--curemap-file")) != _config.end()) {
      istringstream istr(it->second);
      istr >> m_curemapfile;
    }

    m_pout = 0.3;
    log("Cure map file: %s", m_curemapfile.c_str());
    GridMapData def;
    def.occupancy = UNKNOWN;
    //std::vector< pair<std::string,double> > objectprobability;
    //objectprobability.push_back(make_pair("ricebox",0));
    //def.objprob = objectprobability;
    def.pdf = 0;
    m_map = new SpatialGridMap::GridMap<GridMapData>(m_gridsize, m_gridsize, m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
    m_tracer = new LaserRayTracer<GridMapData>(m_map,1.0);
    pbVis = new VisualPB_Bloxel(m_PbHost,5050,m_gridsize,m_gridsize,m_cellsize,1,true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time

     m_lgm = new Cure::LocalGridMap<unsigned char>(m_gridsize/2, m_cellsize, '2', Cure::LocalGridMap<unsigned char>::MAP1);
    m_lgmpdf = new Cure::LocalGridMap<double>(m_gridsize/2, m_cellsize,0, Cure::LocalGridMap<unsigned char>::MAP1);
    m_Glrt  = new Cure::ObjGridLineRayTracer<unsigned char>(*m_lgm);
    pbVis->connectPeekabot();

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
  void VisualObjectSearch::LoadSpatialRelations(std::string filename){
    //open file
    ifstream file(filename.c_str());
    if (!file.good()){
      log("Could not open file, returning without doing anything.");
      return;
    }
    string line,word,objectname;

    ObjectRelations objrel;

    while(getline(file,line)){
      ObjectPairRelation pairrel;
      istringstream iss(line, istringstream::in);
      iss >> word;
      if (word == "END"){
	log("reached end breaking");
	objectData.push_back(objrel);
	break;
      }
      SetCurrentTarget(word);
      if (objrel.object != word){ //if this is a new object
	if ( objrel.relations.size() != 0)
	  objectData.push_back(objrel);
	objrel.relations.clear();
      }
      objrel.object = word;
      pairrel.primaryobject = word;
      iss >> word;
      pairrel.relation = word == "ON" ? FrontierInterface::ON : FrontierInterface::IN;
      iss >> pairrel.secobject;
      iss >> word;
      pairrel.prob = atof(word.c_str());
      objrel.relations.push_back(pairrel);
    }
    log("relations loaded %d objects",objectData.size());
    for (unsigned int i = 0; i < objectData.size(); i++){
      for (unsigned int j = 0; j < objectData[i].relations.size(); j++){
	cout<< objectData[i].object << " " << objectData[i].relations[j].relation << " " << objectData[i].relations[j].secobject << " " << objectData[i].relations[j].prob << endl;
      }
    }
  }
  void VisualObjectSearch::start() {
    log("I have started");
    setupPushScan2d(*this, 0.1);
    setupPushOdometry(*this);

    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
	new MemberFunctionChangeReceiver<VisualObjectSearch>(this,
	  &VisualObjectSearch::newRobotPose));

    addChangeFilter(createLocalTypeFilter<SpatialData::SpatialObject>(cdl::ADD),
	new MemberFunctionChangeReceiver<VisualObjectSearch>(this,
	  &VisualObjectSearch::newSpatialObject));

    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<VisualObjectSearch>(this,
	  &VisualObjectSearch::newRobotPose));

       addChangeFilter(createLocalTypeFilter<
       SpatialData::NavCommand> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtNavCommand));


    /* addChangeFilter(createLocalTypeFilter<
       FrontierInterface::ObjectTiltAngleRequest> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtTiltAngleRequest));
*/
       addChangeFilter(createGlobalTypeFilter<
       VisionData::Recognizer3DCommand> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtRecognizer3DCommand));
     
       addChangeFilter(createGlobalTypeFilter<
       FrontierInterface::ObjectPriorRequest> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtWeightedPointCloud));

    addChangeFilter(createGlobalTypeFilter<
	VisionData::VisualObject> (cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
	  &VisualObjectSearch::newVisualObject));


    addChangeFilter(createGlobalTypeFilter<
	VisionData::VisualObject> (cdl::ADD),
	new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
	  &VisualObjectSearch::newVisualObject));

  }


  void 
  VisualObjectSearch::newSpatialObject(const cast::cdl::WorkingMemoryChange &objID)
  {
    try {
      SpatialData::SpatialObjectPtr newObj = 
	getMemoryEntry<SpatialData::SpatialObject>(objID.address);

      spatial::Object *model = generateNewObjectModel(newObj->label);

      model->pose = newObj->pose;

      putObjectInMap(*m_map, model);

      pbVis->DisplayMap(*m_map);

      delete model;

      if (!waitingForDetection.empty() || !waitingForObjects.empty()) {
	if (newObj->label == currentTarget) {
	  DetectionComplete(true);
	}
	else {
	  waitingForObjects.erase(newObj->label);
	  waitingForDetection.erase(newObj->label);

	  string logString("Waiting list: ");
	  for (std::set<string>::iterator it = waitingForObjects.begin(); it != waitingForObjects.end(); it++) {
	    logString += *it + " ";
	  }
	  for (std::set<string>::iterator it = waitingForDetection.begin(); it != waitingForDetection.end(); it++) {
	    logString += *it + " ";
	  }
	  log(logString.c_str());

	  if (waitingForDetection.empty() && waitingForObjects.empty()) {
	    DetectionComplete(false);
	  }
	}
      }
    }
    catch (DoesNotExistOnWMException e) {
      log("Error! SpatialObject disappeared from WM!");
    }
  }

  void
  VisualObjectSearch::putObjectInMap(GridMap<GridMapData> &map, spatial::Object *object)
  {
    cogx::Math::Pose3 &pose = object->pose;
    switch (object->type) {
      case OBJECT_BOX:
	{
	  BoxObject &box = *(BoxObject*)object;
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
	  if (direction < 0) direction += 2*M_PI;
	  GDMakeObstacle makeObstacle;
	  map.boxModifier(box.pose.pos.x, box.pose.pos.y, box.pose.pos.z, 2*radius1, 2*radius2, 2*radius3,
	      direction, makeObstacle);
	}
	break;
      case OBJECT_HOLLOW_BOX:
	{
	  HollowBoxObject &box = *(HollowBoxObject*)object;
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
	  if (direction < 0) direction += 2*M_PI;
	  GDMakeObstacle makeObstacle;
	  double h = box.thickness/2;
	  double cd = cos(direction);
	  double sd = sin(direction);

	  if (openSide != 0) 
	    map.boxModifier(box.pose.pos.x, box.pose.pos.y, 
		box.pose.pos.z + radius3-h, radius1*2, radius2*2, 2*h,
	      direction, makeObstacle);

	  if (openSide != 1) 
	    map.boxModifier(box.pose.pos.x+cd*(radius1-h), 
		box.pose.pos.y+sd*(radius1-h),
		box.pose.pos.z, 2*h, radius2*2, radius3*2,
	      direction, makeObstacle);

	  map.boxModifier(box.pose.pos.x-cd*(radius1-h), 
	      box.pose.pos.y-sd*(radius1-h),
	      box.pose.pos.z, 2*h, radius2*2, radius3*2,
	      direction, makeObstacle);

	  map.boxModifier(box.pose.pos.x-sd*(radius2-h), 
	      box.pose.pos.y+cd*(radius2-h),
	      box.pose.pos.z, radius1*2, 2*h, radius3*2,
	      direction, makeObstacle);

	  map.boxModifier(box.pose.pos.x+sd*(radius2-h), 
	      box.pose.pos.y-cd*(radius2-h),
	      box.pose.pos.z, radius1*2, 2*h, radius3*2,
	      direction, makeObstacle);

	  if (openSide != 5) 
	    map.boxModifier(box.pose.pos.x, box.pose.pos.y, 
		box.pose.pos.z - radius3+h, radius1*2, radius2*2, 2*h,
	      direction, makeObstacle);
	}
	break;
      default:
	log("Error! Unsupported object type in puObjectInMap!");
	return;
    }
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

  void VisualObjectSearch::InitializePDF(double initprob){
    GDProbInit initfunctor(initprob/(m_map->getZBounds().second - m_map->getZBounds().first));

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
	int bloxelX = x + m_lgm->getSize();
	int bloxelY = y + m_lgm->getSize();
	if ((*m_lgm)(x,y) == '1'){
	  // For each obstacle cell, assign a uniform probability density to it
	  // (free/unknown space only), and to its immediate neighbors
	  // (Only neighbors which are not unknown in the Cure map. Unknown 3D
	  // space is still assigned, though)
	  for(int i = -1; i <= 1; i++){
	    for (int j=-1; j <= 1; j++){
	      if((*m_lgm)(x+i,y+j) != '2' && (bloxelX+i <= m_gridsize && bloxelX+i > 0 ) 
		  && (bloxelY+i <= m_gridsize && bloxelY+i > 0 ))
		m_map->boxSubColumnModifier(bloxelX+i,bloxelY+j, m_mapceiling/2, m_mapceiling,initfunctor);
	    }
	  }
	}
      }
    }
    double massAfterInit = initfunctor.getTotal();
//    double normalizeTo = (initfunctor.getTotal()*m_pout)/(1 - m_pout);
    normalizePDF(*m_map,initprob,massAfterInit);
 for (int x = -m_lgmpdf->getSize(); x <= m_lgmpdf->getSize(); x++) {
    for (int y = -m_lgmpdf->getSize(); y <= m_lgmpdf->getSize(); y++) {
      int bloxelX = x + m_lgmpdf->getSize();
      int bloxelY = y + m_lgmpdf->getSize();
      if ((*m_lgm)(x,y) != '2'){
	double bloxel_floor = 0;
	for (unsigned int i = 0; i < (*m_map)(bloxelX,bloxelY).size();i++){
	  (*m_lgmpdf)(x,y) += (*m_map)(bloxelX, bloxelY)[i].data.pdf * ((*m_map)(bloxelX,bloxelY)[i].celing - bloxel_floor);
	  bloxel_floor = (*m_map)(bloxelX,bloxelY)[i].celing;
	}	
      }
    }
  }
  }
  void VisualObjectSearch::ReadCureMapFromFile() {
    log("Reading cure map");

    ifstream file(m_curemapfile.c_str());
    if (!file.good()){
      log("Could not open file, returning without doing anything.");
      return;
    }
    GDMakeObstacle makeobstacle;
    std::string line;

    for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
      getline(file,line);
      if (line != ""){
	int count = 0;
	for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
	  char c = line[count];
	  if (c == '3'){ //if this is a wall
	    (*m_lgm)(x, y) = '1';
	    m_map->boxSubColumnModifier(x+m_lgm->getSize(),y + m_lgm->getSize(), m_mapceiling/2, m_mapceiling,makeobstacle);
	  }
	  else if  (c  == '1'){ // a normal obstacle
	    (*m_lgm)(x, y) = c;
	    m_map->boxSubColumnModifier(x+m_lgm->getSize(),y + m_lgm->getSize(), m_LaserPoseR.getZ(), m_minbloxel*2,makeobstacle);
	  }
	  else{
	    (*m_lgm)(x, y) = c;
	  }
	  count++;
	}
      }
      line = "";
    }

    vector< pair<double,double> > thresholds;
    thresholds.push_back(make_pair(1e-4,1));
    InitializePDF(1-m_pout);
    pbVis->DisplayMap(*m_map);
    pbVis->AddPDF(*m_map, thresholds);
     pbVis->Display2DCureMap(m_lgm);
    m_maploaded = true;
  }

  void VisualObjectSearch::savemap( GtkWidget *widget,gpointer data )
  {
    AVSComponentPtr->SaveCureMapToFile();
  }
  void VisualObjectSearch::readmap( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->ReadCureMapFromFile();
  }
  void VisualObjectSearch::selectdu( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->LookforObjectWithStrategy(DIRECT_UNINFORMED);
  }
  void VisualObjectSearch::selectdi( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->LookforObjectWithStrategy(DIRECT_INFORMED);
  }
  void VisualObjectSearch::selectind( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->searchChainPos = 0;
    AVSComponentPtr->LookforObjectWithStrategy(INDIRECT);
  }



  void VisualObjectSearch::runComponent(){
    m_command = IDLE;
    if(m_savemapmode){
      int argc= 0;
      char** argv = NULL;

      gtk_init (&argc, &argv);

      // add a window
      window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
      gtk_window_set_title (GTK_WINDOW (window), "map controls");

      //window cannot hold more than 1 button, thus ad a box
      hbox = gtk_hbox_new (FALSE, 5);
      savebutton = gtk_button_new_with_label ("Save Map");
      readbutton = gtk_button_new_with_label ("Load Map");
      direct_uninformed = gtk_button_new_with_label ("Direct Uninformed");
      direct_informed = gtk_button_new_with_label ("Direct Informed");
      indirect = gtk_button_new_with_label ("Indirect");




      //parent: window, child: hbox
      gtk_container_add (GTK_CONTAINER (window), hbox);
      gtk_container_set_border_width (GTK_CONTAINER (window), 5);
      gtk_widget_show (hbox);

      //callbacks

      g_signal_connect (G_OBJECT (savebutton), "clicked",
	  G_CALLBACK (&spatial::VisualObjectSearch::savemap), NULL);
      g_signal_connect (G_OBJECT (readbutton), "clicked",
	  G_CALLBACK (&spatial::VisualObjectSearch::readmap), NULL);


      g_signal_connect (G_OBJECT (direct_uninformed), "clicked",
	  G_CALLBACK (&spatial::VisualObjectSearch::selectdu), NULL);
      g_signal_connect (G_OBJECT (direct_informed), "clicked",
	  G_CALLBACK (&spatial::VisualObjectSearch::selectdi), NULL);
      g_signal_connect (G_OBJECT (indirect), "clicked",
	  G_CALLBACK (&spatial::VisualObjectSearch::selectind), NULL);



      //add buttons to box
      gtk_box_pack_start (GTK_BOX (hbox), readbutton, TRUE, TRUE, 0);
      gtk_box_pack_start (GTK_BOX (hbox), savebutton, TRUE, TRUE, 0);

      gtk_box_pack_start (GTK_BOX (hbox), direct_uninformed, TRUE, TRUE, 0);
      gtk_box_pack_start (GTK_BOX (hbox), direct_informed, TRUE, TRUE, 0);
      gtk_box_pack_start (GTK_BOX (hbox), indirect, TRUE, TRUE, 0);

      gtk_widget_show (direct_uninformed);
      gtk_widget_show (direct_informed);
      gtk_widget_show (indirect);
      gtk_widget_show (readbutton);
      gtk_widget_show (savebutton);
      gtk_widget_show (window);
    }

    while(true){ 
      m_Mutex.lock();
//      pbVis->DisplayMap(*m_map);
//      pbVis->Display2DCureMap(m_lgm);
      while(gtk_events_pending())
	gtk_main_iteration();
      InterpretCommand();
      m_Mutex.unlock();
      sleep(1);
    }
  }
  //void
  //VisualObjectSearch::AskForDistribution(const vector<string> &objectLabels,
  //    const vector<SpatialData::ObjectRelationPtr> &relations)
  //{
  //    FrontierInterface::WeightedPointCloudPtr queryCloud
  //      = new FrontierInterface::WeightedPointCloud;
  //
  //    //write lgm to WM
  //    FrontierInterface::ObjectPriorRequestPtr objreq =
  //        new FrontierInterface::ObjectPriorRequest;
  //    objreq->relationTypes = relations; // ON or IN or whatnot
  //    objreq->objects = objectLabels;	// Names of objects, starting with the query object
  //    objreq->cellSize = m_cellsize;	// Cell size of map (affects spacing of samples)
  //    objreq->outCloud = queryCloud;	// Data struct to receive output
  //    addToWorkingMemory(newDataID(), objreq);
  //}

  //void
  //VisualObjectSearch::owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID) {
  //  try {
  //    FrontierInterface::WeightedPointCloudPtr cloud =
  //      getMemoryEntry<FrontierInterface::WeightedPointCloud>(objID.address);
  //
  //    m_sampler.kernelDensityEstimation3D(m_map, cloud->center,
  //	cloud->interval,
  //	cloud->xExtent,
  //	cloud->yExtent,
  //	cloud->zExtent,
  //	cloud->values);
  //  }
  //  catch (DoesNotExistOnWMException excp) {
  //    log("Error!  WeightedPointCloud does not exist on WM!");
  //    return;
  //  }
  //}

  VisualObjectSearch::ObjectPairRelation VisualObjectSearch::GetSecondaryObject(string name){
    ObjectPairRelation obj;
    obj.primaryobject = "";
    //NOTE: WE ASSUME THAT EACH OBJECT HAS EXACTLY ONE SPATIAL RELATION WITH ANOTHER OBJECT!
    // I.E. NO SUCH: RICE ON TABLE, RICE IN ROVIO AT THE SAME TIME
    for(unsigned int i = 0; i < objectData.size(); i++){
      if (objectData[i].object == name && objectData[i].relations.size() != 0){
	return objectData[i].relations[0];
      }
    }
    return obj;
  }
  void VisualObjectSearch::InterpretCommand(){
    switch (m_command)
        {
      case STOP: {
        log("Stopped.");
        m_command = IDLE;
        log("Command: STOP");
        Cure::Pose3D pos;
        PostNavCommand(pos, SpatialData::STOP);
        break;
      }
      case ASK_FOR_DISTRIBUTION:{
	log("Command: ASK_FOR_DIST");
        m_command = IDLE;
        AskForDistribution();
	break;
      }
      case NEXT_NBV:{
        //TODO isStopSearch then STOP	
	log("Command: NEXT_NBV");
        m_command = IDLE; //Wait to get in position
	GoToNBV();
	break;
      }
      case RECOGNIZE:{
	log("Command: Recognize");
        m_command = IDLE; //Wait for recognition
	Recognize();
	break;
      }
      case IDLE:{
	log("Idling..");
	break;
      }
    }
  }
  void VisualObjectSearch::GoToNBV(){
    for (int x = -m_lgmpdf->getSize(); x <= m_lgmpdf->getSize(); x++) {
      for (int y = -m_lgmpdf->getSize(); y <= m_lgmpdf->getSize(); y++) {
	int bloxelX = x + m_lgmpdf->getSize();
	int bloxelY = y + m_lgmpdf->getSize();
	if ((*m_lgm)(x,y) != '2'){
	  double bloxel_floor = 0;
	  (*m_lgmpdf)(x,y) = 0;
	  for (unsigned int i = 0; i < (*m_map)(bloxelX,bloxelY).size();i++){
	    (*m_lgmpdf)(x,y) += (*m_map)(bloxelX, bloxelY)[i].data.pdf * ((*m_map)(bloxelX,bloxelY)[i].celing - bloxel_floor);
	    bloxel_floor = (*m_map)(bloxelX,bloxelY)[i].celing;
	  }	
	}
      }
    }

    m_nbv = SampleAndSelect();
    /* Add plan to PB BEGIN */
    NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
    NavData::ViewPoint viewpoint;
    viewpoint.pos.x = m_nbv.pos[0];
    viewpoint.pos.y = m_nbv.pos[1];
    viewpoint.pos.z = m_nbv.pos[2];
	viewpoint.pan = m_nbv.pan;
	viewpoint.tilt = m_nbv.tilt;
        obs->planlist.push_back(viewpoint);
        addToWorkingMemory(newDataID(), obs);
     /* Add plan to PB END */

cout << "selected cone" << viewpoint.pos.x << " " << viewpoint.pos.y << " " <<viewpoint.pos.z << "" << viewpoint.pan << " " << viewpoint.tilt << endl;
    // Send move command
    Cure::Pose3D pos;
    pos.setX(m_nbv.pos[0]);
    pos.setY(m_nbv.pos[1]);
    pos.setTheta(m_nbv.pan); 
    log("posting nav command");
    //m_command = RECOGNIZE; // FIXME this line is added and below is commented out only for easy testing
    PostNavCommand(pos, SpatialData::GOTOPOSITION);
  }

void VisualObjectSearch::owtNavCommand(const cast::cdl::WorkingMemoryChange &objID){
    try{
      if(isSearchFinished)
	return;
      log("nav command overwritten");
      SpatialData::NavCommandPtr cmd(getMemoryEntry<
	  SpatialData::NavCommand> (objID.address));
      if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
	log("NavCommand succeeded.");

	// Move pan tilt to correct position
	MovePanTilt(0, m_nbv.tilt, 0.08);

	m_command = RECOGNIZE;
      }
      else if (cmd->comp == SpatialData::COMMANDFAILED){
	log("NavCommand failed.Getting next view.");
	m_command=NEXT_NBV;
      }
    }
    catch (const CASTException &e) {
      //log("failed to delete SpatialDataCommand: %s", e.message.c_str());
      log("CASTException in VisualObjectSearch::owtNavCommand");
    }
  }

  void VisualObjectSearch::LookforObjectWithStrategy(SearchMode mode){
    //ask   
    if (mode == DIRECT_UNINFORMED){
      currentSearchMode = DIRECT_UNINFORMED;
      log("Will look for %s", currentTarget.c_str());
      m_command = NEXT_NBV;
    }
    else if (mode == DIRECT_INFORMED){
      currentSearchMode = DIRECT_INFORMED;
      log("Will look for %s", currentTarget.c_str());
      m_command = ASK_FOR_DISTRIBUTION;
    }
    else if(mode == INDIRECT){
      currentSearchMode = INDIRECT;
      ObjectPairRelation rel;
      while(true){
	rel = GetSecondaryObject(currentTarget);
	if(rel.primaryobject == "")
	  break;
	searchChain.push_back(rel);
	SetCurrentTarget(rel.secobject);
      }
      for(unsigned int i = 0; i < searchChain.size(); i++){
	cout << searchChain[i].primaryobject << " " << searchChain[i].secobject << " " << searchChain[i].prob << endl;
      }
      m_command = NEXT_NBV;
    }
  }

  void VisualObjectSearch::AskForDistribution(){
    log("Asking for distribution");
    m_command = IDLE;

    //if informed direct, get search chain, construct distribution parameters and ask for it
    double queryProbabilityMass = 1.0;

    if (currentSearchMode == DIRECT_INFORMED){
      vector<FrontierInterface::ObjectRelation> relations;
      vector<string> labels;
      ObjectPairRelation rel;
      string name = currentTarget;
      while(true){
	rel = GetSecondaryObject(name);
	if(rel.primaryobject == "")
	  break;
	searchChain.push_back(rel); 
	name = rel.secobject;
	queryProbabilityMass *= rel.prob;
      }
      cout << queryProbabilityMass << endl;
      labels.push_back(currentTarget);
      for(unsigned int i = 0; i < searchChain.size(); i++){
	relations.push_back(searchChain[i].relation);
	labels.push_back(searchChain[i].secobject);
      }
      FrontierInterface::WeightedPointCloudPtr queryCloud = new FrontierInterface::WeightedPointCloud;
      //write lgm to WM
      FrontierInterface::ObjectPriorRequestPtr objreq =
	new FrontierInterface::ObjectPriorRequest;
      objreq->relationTypes = relations; // ON or IN or whatnot
      objreq->objects = labels;	// Names of objects, starting with the query object
      objreq->cellSize = m_cellsize;	// Cell size of map (affects spacing of samples)
      objreq->outCloud = queryCloud;	// Data struct to receive output
      objreq->totalMass = queryProbabilityMass;
      addToWorkingMemory(newDataID(), objreq);

      cout << labels[0] << endl;
      for(unsigned int i = 0; i < relations.size(); i++){
	cout << relations[i]<< " " << labels[i + 1]  << endl;
      }

    }
    //if indirect look where we are in the chain ask for it's distribution
    else if (currentSearchMode == INDIRECT){
      if(currentTarget == searchChain.back().secobject){
	// This is the first target, no relation exists
	// Just use the predefined uniform PDF
	return;
      }

      vector<FrontierInterface::ObjectRelation> relations;
      vector<string> labels;

      unsigned int i;
      for(i=0; i<searchChain.size(); i++){
	if(searchChain[i].primaryobject == currentTarget){
	  break;
	}
      }

      labels.push_back(currentTarget);
      relations.push_back(searchChain[i].relation);
      labels.push_back(searchChain[i].secobject);

      FrontierInterface::WeightedPointCloudPtr queryCloud = new FrontierInterface::WeightedPointCloud;
      //write lgm to WM
      FrontierInterface::ObjectPriorRequestPtr objreq =
	new FrontierInterface::ObjectPriorRequest;
      objreq->relationTypes = relations; // ON or IN or whatnot
      objreq->objects = labels;	// Names of objects, starting with the query object
      objreq->cellSize = m_cellsize;	// Cell size of map (affects spacing of samples)
      objreq->outCloud = queryCloud;	// Data struct to receive output
      addToWorkingMemory(newDataID(), objreq);
    }
  }

  void
    VisualObjectSearch::owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID) {
      try {
	log("got weighted PC");
	FrontierInterface::ObjectPriorRequestPtr req =
	  getMemoryEntry<FrontierInterface::ObjectPriorRequest>(objID.address);
	FrontierInterface::WeightedPointCloudPtr cloud =
	  req->outCloud;

	vector<Vector3> centers;
	centers.push_back(cloud->center);

	//req->totalMass represents the probability of the relation
	//GIVEN that the object is in the room.
	//We want m_pout to remain unchanged, and the probability mass
	//already in the map diminished so that the relative probability
	//inside the room coming from the cloud is req->totalMass.
	//I.e., after normalization its mass should be (1-pout)*req->totalMass.
	//Before normalization, it will need to be 1/(1-req->totalMass) times larger
	double weightToAdd;
	if (req->totalMass > 0.99999999999) {
	  GDProbScale reset(0.0);
	  m_map->universalQuery(reset, true);
	  weightToAdd = (1-m_pout);
	}
	else {
	  weightToAdd = (1-m_pout)*req->totalMass/(1-req->totalMass);
	}

	cout << req->totalMass << endl;
	if(cloud->isBaseObjectKnown) {
	  log("Got distribution around known object pose");
	  m_sampler.kernelDensityEstimation3D(*m_map,
	      centers,
	      cloud->interval,
	      cloud->xExtent,
	      cloud->yExtent,
	      cloud->zExtent,
	      cloud->values,
	      1.0,
	      weightToAdd,
	      m_lgm
	      );
	  normalizePDF(*m_map,1.0-m_pout);
	}
	else {
	  log("Got distribution around unknown object pose");
	  vector<pair<int, int> >obstacleCells;
	  const unsigned int nTargetKernelSamples = 25;
	  const int step = 10;

	  while (obstacleCells.size() < nTargetKernelSamples) {
	    int yOffset = rand()%step;
	    for (int x = -m_lgm->getSize() + rand()%step; x <= m_lgm->getSize(); x+=step) {
	      for (int y = -m_lgm->getSize() + yOffset; y <= m_lgm->getSize(); y+=step) {
		int bloxelX = x + m_lgm->getSize();
		int bloxelY = y + m_lgm->getSize();
		if ((*m_lgm)(x,y) == '1') {
		  obstacleCells.push_back(make_pair<int>(bloxelX, bloxelY));
		}
	      }
	    }
	  }

	  vector<Vector3> centers;
	  for(unsigned int i = 0; i < obstacleCells.size(); i++) {
	    // place a KDE cloud around it, with the center given
	    // by the relation manager (which includes a z-coordinate
	    // for tables, desks etc)
	    int bloxelX = obstacleCells[i].first;
	    int bloxelY = obstacleCells[i].second;
	    pair<double, double> kernelCoords = 
	      m_map->gridToWorldCoords(bloxelX, bloxelY);
	    cloud->center.x = kernelCoords.first;
	    cloud->center.y = kernelCoords.second;
	    centers.push_back(cloud->center);
	  }

	  m_sampler.kernelDensityEstimation3D(*m_map, 
	      centers,
	      cloud->interval,
	      cloud->xExtent,
	      cloud->yExtent,
	      cloud->zExtent,
	      cloud->values,
	      1.0/(m_lgm->getSize()), //Just an ad-hoc guess
	      weightToAdd,
	      m_lgm);
	  normalizePDF(*m_map,1.0-m_pout);
	}
	m_command = NEXT_NBV;

	vector< pair<double,double> > thresholds;
	thresholds.push_back(make_pair(1e-4,1e-3));
	thresholds.push_back(make_pair(1e-3,5e-3));
	thresholds.push_back(make_pair(5e-3,1e-2));
	thresholds.push_back(make_pair(1e-2,5e-2));
	thresholds.push_back(make_pair(5e-2,1e2));
	pbVis->AddPDF(*m_map, thresholds);
	}
      catch (DoesNotExistOnWMException excp) {
	log("Error!  WeightedPointCloud does not exist on WM!");
	return;
      }
//      else {
//	// The base object pose is unknown. The returned cloud->center
//	// represents just a sample offset; we must populate the grid map with
//	// a pdf based on 
//      }
    }
  bool VisualObjectSearch::isCircleFree(double xW, double yW, double rad){
    int xiC,yiC;
    if (m_lgm->worldCoords2Index(xW,yW,xiC,yiC)!= 0)
      return false;

    double w = rad / m_lgm->getCellSize();
    int wi = int(w + 0.5);

    for (int x = xiC-wi; x <= xiC+wi; x++) {
      for (int y = yiC-wi; y <= yiC+wi; y++) {
	if (x >= -m_lgm->getSize() && x <= m_lgm->getSize() && y >= -m_lgm->getSize() && y <= m_lgm->getSize()) {
	  if (hypot(x-xiC,y-yiC) < w) {
	    if ((*m_lgm)(x,y) == '1' or (*m_lgm)(x,y) == '2') return false;
	  }
	}
      }
    }
    return true;
  }
  
bool isCircleFree2D(const Cure::LocalGridMap<unsigned char> &map, double xW, double yW, double rad)
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
          if (map(x,y) != '0') return false;
        }
      }
    }
  }

  return true;
}void VisualObjectSearch::Sample2DGrid() {
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
        if ((*m_lgm)(x, y) == '1') { // occupied OR Plane
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
        if ((*m_lgm)(x, y) == '2') {
          m_PathGrid.setBit(x + m_lgm->getSize(), y + m_lgm->getSize(), true);
        }
      }
    }

    /*Checking if a point in x,y is reachable */

    for (double rad = 0; rad < M_PI * 2; rad = rad + M_PI / 13) {
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

      if (!haspoint && (*m_lgm)(randx, randy) == '0' && isCircleFree2D(*m_lgm, xW, yW, 0.2)) {
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
    //samples.set_opacity(0.2);

    vector < vector<double> > sampled2Dpoints;
    double xW1,yW1;
    for(int i= 0; i < m_samplesize; i++){
      vector<double> point;
      m_lgm->index2WorldCoords(m_samples[i * 2], m_samples[i * 2 + 1], xW1, yW1);
      point.push_back(xW1);
      point.push_back(yW1);
      point.push_back(1.4);
      sampled2Dpoints.push_back(point);
    }
    pbVis->Add3DPointCloud(sampled2Dpoints);
    /* Display Samples in PB END */

    /*Sampling free space END*/
  }


  std::vector<std::vector<int> >
  VisualObjectSearch::GetViewCones() {
    log("Calculating view cones for generated samples");
    Cure::Pose3D candidatePose;
    XVector3D a;
    std::vector<int> tpoints;
    std::vector<std::vector<int> > ViewConePts;

    for (int y = 0; y < m_samplesize; y++) { //calc. view cone for each sample

      m_lgm->index2WorldCoords(m_samples[y * 2], m_samples[2 * y + 1], a.x, a.y);
      a.theta = m_samplestheta[y];
      tpoints = GetInsideViewCone(a, true);
    //  cout << "view cone got " << tpoints.size() << endl;
  
 /*   vector < vector<double> > sampled2Dpoints;
    double xW1,yW1;
    for(int i= 0; i < tpoints.size() / 2; i++){
      vector<double> point;
      m_lgm->index2WorldCoords(tpoints[i * 2], tpoints[i * 2 + 1], xW1, yW1);
      point.push_back(xW1);
      point.push_back(yW1);
      point.push_back(1.4);
      sampled2Dpoints.push_back(point);
    }
    pbVis->Add3DPointCloud(sampled2Dpoints);
*/

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
VisualObjectSearch::GetInsideViewCone(XVector3D &a, bool addall) {
  std::vector<int> tpoints;
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
VisualObjectSearch::CalculateViewCone(XVector3D a, double direction,
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
VisualObjectSearch::FindBoundingRectangle(XVector3D a, XVector3D b, XVector3D c,
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
VisualObjectSearch::isPointInsideTriangle(XVector3D p, XVector3D a, XVector3D b,
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
VisualObjectSearch::isPointSameSide(XVector3D p1, XVector3D p2, XVector3D a,
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


VisualObjectSearch::SensingAction VisualObjectSearch::SampleAndSelect(){

  double cameraheight = 1.4;
  debug("Sampling Grid.");

  double lgmpdfsum = 0;
  for (int x = -m_lgmpdf->getSize(); x <= m_lgmpdf->getSize(); x++) {
    for (int y = -m_lgmpdf->getSize(); y <= m_lgmpdf->getSize(); y++) {
      lgmpdfsum += (*m_lgmpdf)(x,y);
    }
  }
  cout<< "lgmpdf map sums to: " << lgmpdfsum << endl;
  std::vector<std::vector<int> > VCones;
  VCones.clear();
  log("sampling grid");
  Sample2DGrid();
  log("getting view cones");
  VCones = GetViewCones();
  double highest_sum = -10000.0;
  double sum;
  int highest_VC_index = 0;
  int x, y;
  for (unsigned int i = 0; i < VCones.size(); i++) {
    sum = 0;
    for (unsigned int j = 0; j < VCones[i].size() / 2; j++) {
      x = VCones[i][2 * j];
      y = VCones[i][2 * j + 1];
      sum += (*m_lgmpdf)(x, y);
    }
    if (sum > highest_sum) {
      highest_sum = sum;
      highest_VC_index = i;
    }
  }

  std::pair<double,double> best2DconeW;
  double pan = m_samplestheta[highest_VC_index];
  m_lgm->index2WorldCoords(m_samples[2*highest_VC_index],m_samples[2*highest_VC_index +1], best2DconeW.first, best2DconeW.second); 

  cout << "highest sum is: " << highest_sum << " " << "for view cone at: " << best2DconeW.first << "," << best2DconeW.second << endl ;

  pair<int,int> tiltRange(20,-20);
  std::vector< pair<int,int> > freespace;
  std::vector< std::vector<double> >  visualizationpoints;
  SensingAction sample;
  std::vector <SensingAction> samplepoints;

    std::vector<double> angles;
  for (double rad = 0; rad < M_PI * 2; rad = rad + M_PI / 3) {
    angles.push_back(rad);
  }

  int i = 0;
  while (i< 5){
    double randomTilt = ((rand() % (tiltRange.first + abs(tiltRange.second))) - abs(tiltRange.second))*M_PI/180;
    double randomPan = (rand() % 360 - 180)*M_PI/180;
    randomPan = pan; 
    std::vector<double> coord;
    coord.push_back(best2DconeW.first);
    coord.push_back(best2DconeW.second);
    coord.push_back(cameraheight);
    visualizationpoints.push_back(coord);
    SensingAction sample;
    sample.pos = coord;
    sample.pan = randomPan;
    sample.tilt= randomTilt;
    samplepoints.push_back(sample);
    i++;
    // add this for visualization
  }
  pbVis->Add3DPointCloud(visualizationpoints);
  int bestConeIndex = GetViewConeSums(samplepoints);
  return samplepoints[bestConeIndex];

}

//Get X number of samples

/*    srand(time(NULL));
      int i = 0;
      while (i< m_samplesize){
      int randomPos = rand() % freespace.size();
      double randomTilt = ((rand() % (tiltRange)) - tiltRange/2.0)*M_PI/180;
      double randomPan = (rand() % 360 - 180)*M_PI/180;

      pair<int,int> samplepoint(freespace[randomPos].first,freespace[randomPos].second);
//  pair<int,int> samplepoint(rand() % xGrid,rand() % yGrid);
// Convert cure coordinates to Bloxel Grid coordinates
int bloxelX = samplepoint.first + m_gridsize/2;
int bloxelY = samplepoint.second + m_gridsize/2;
pair<double,double> worldCoords = m_map->gridToWorldCoords(bloxelX,bloxelY);

if (isCircleFree(worldCoords.first, worldCoords.second, 0.5) && (*m_lgm)(samplepoint.first,samplepoint.second) == '0'){
//cout << "sample point: " << samplepoint.first << "," << samplepoint.second << endl;
//check if the sample point too close to an obstacle by a box query 
//cout << "bloxel world coords : " << worldCoords.first << "," << worldCoords.second << endl;
double xW,yW;           
m_lgm->index2WorldCoords(samplepoint.first,samplepoint.second,xW,yW);
std::vector<double> coord;
//cout << "cure world coords : " << xW << "," << yW << endl;
coord.push_back(xW);
coord.push_back(yW);
coord.push_back(cameraheight);
visualizationpoints.push_back(coord);
SensingAction sample;
sample.pos = coord;
sample.pan = randomPan;
sample.tilt= randomTilt;

samplepoints.push_back(sample);
i++;
// add this for visualization
}
}
pbVis->Add3DPointCloud(visualizationpoints);

NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
for(unsigned int i = 0; i < samplepoints.size(); i++){

SensingAction nbv = samplepoints[i];
Add plan to PB BEGIN */
//	NavData::ViewPoint viewpoint;
//        viewpoint.pos.x = nbv.pos[0];
//        viewpoint.pos.y = nbv.pos[1];
//        viewpoint.pos.z = nbv.pos[2];
//	viewpoint.pan = nbv.pan;
//	viewpoint.tilt = nbv.tilt;
//        obs->planlist.push_back(viewpoint);
/* Add plan to PB END 
   }*/


//   addToWorkingMemory(newDataID(), obs);

int VisualObjectSearch::GetViewConeSums(std::vector <SensingAction> samplepoints){
  debug("Querying cones");
  debug("Cone range is %f to %f", m_minDistance, m_conedepth);

  GDProbSum sumcells;
  GDIsObstacle isobstacle;
  double maxpdf = -1000.0;
  int maxindex = -1;

  try{ 
    for (unsigned int i =0; i < samplepoints.size(); i++){
      SensingAction viewpoint = samplepoints[i];
      /*m_map->coneModifier(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan,samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, makeobstacle,makeobstacle);*/
      m_map->coneQuery(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan, samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, sumcells,sumcells, m_minDistance);
      cout << "cone #" << i  << " " << viewpoint.pos[0] << " " << viewpoint.pos[1] << " " <<viewpoint.pos[2] << "" << viewpoint.pan << " " << viewpoint.tilt << " pdfsum: " << sumcells.getResult() << endl;
      if (sumcells.getResult() > maxpdf){
	maxpdf = sumcells.getResult();
	maxindex = i;
      }
      sumcells.reset();
    }
  }
  catch(std::exception &e) {
    printf("Caught exception %s: \n", e.what());
  }
  return maxindex;
}


void
VisualObjectSearch::owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID) {
  try{
    log("got recognizer3D overwrite command");
    VisionData::Recognizer3DCommandPtr cmd(getMemoryEntry<
	VisionData::Recognizer3DCommand> (objID.address));
    if(waitingForDetection.find(cmd->label) != waitingForDetection.end()){
      //We were waiting for detection results on this object
      waitingForDetection.erase(cmd->label);

      string logString("Waiting list: ");
      for (std::set<string>::iterator it = waitingForObjects.begin(); it != waitingForObjects.end(); it++) {
	logString += *it + " ";
      }
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
      //	m_command = NEXT_NBV;
    }
  }
  catch (const CASTException &e) {
    log("failed to delete SpatialDataCommand: %s", e.message.c_str());
  }
}


void
VisualObjectSearch::MovePanTilt(double pan, double tilt, double tolerance) {
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

  void VisualObjectSearch::Recognize() {
    log("Recognize called");
    waitingForDetection.insert(currentTarget);
//    isWaitingForDetection = true;
    //DetectionComplete(false);
    // FIXME Below is commented out just for easy testing
    addRecognizer3DCommand(VisionData::RECOGNIZE,"rovio","");
    addRecognizer3DCommand(VisionData::RECOGNIZE,"bookcase_lg","");
    addRecognizer3DCommand(VisionData::RECOGNIZE,"bookcase_sm","");
    addRecognizer3DCommand(VisionData::RECOGNIZE,"rice","");
  }


  void VisualObjectSearch::DetectionComplete(bool isDetected){
    waitingForDetection.clear();
    waitingForObjects.clear();

    cout << "Detection complete" << endl;
    if (isDetected){
      log("Object Detected.");
      // check for success
      bool greatSuccess = false;
      if(currentSearchMode == DIRECT_UNINFORMED || currentSearchMode == DIRECT_INFORMED){
	greatSuccess = true;
      }
      else if(currentSearchMode == INDIRECT && searchChain[0].primaryobject == currentTarget){
	greatSuccess = true;
      }
      if(greatSuccess){
	isSearchFinished = true;
	m_command = STOP;
	log("Object Detected, Mission Completed.");
	return;
      }

      // if we are doing indirect search then ask & initialize next object
      if(currentSearchMode == INDIRECT){
	for(unsigned int i=0; i<searchChain.size(); i++){
	  if(searchChain[i].secobject == currentTarget){
	    SetCurrentTarget(searchChain[i].primaryobject);
	  }
	  log("Object Detected, new target is %s", currentTarget.c_str());
	  m_command = ASK_FOR_DISTRIBUTION;
	}
      }
    } else {
      // if we are not yet finished Go to NBV
      log("Object not detected");
      UnsuccessfulDetection(m_nbv);  
      m_command = NEXT_NBV;
    }
  }

  void
    VisualObjectSearch::SetCurrentTarget(const string &label) {
      currentTarget = label;
      double objectSize = 0.5;
      if (label == "table") {
	objectSize = 1.1;
      }
      else if (label == "bookcase_sm") {
	objectSize = 1.5;
      }
      else if (label == "bookcase_lg") {
	objectSize = 1.93;
      }
      else if (label == "desk") {
	objectSize = 2.0;
      }
      else if (label == "rice") {
	objectSize = 0.19;
      }
      else if (label == "rovio") {
	objectSize = 0.35;
      }
      m_minDistance = 0.5*objectSize/tan(0.5*m_vertangle);
      if (m_minDistance < 0.5) m_minDistance = 0.5;
      m_conedepth = m_minDistance * 4;
      if (m_conedepth > 5.0) m_conedepth = 5.0;
    }

  void
  VisualObjectSearch::newVisualObject(const cast::cdl::WorkingMemoryChange &objID) {

    try{
      debug("new visual object");
      VisionData::VisualObjectPtr visualobject(getMemoryEntry<
	  VisionData::VisualObject> (objID.address));
      if (waitingForDetection.find(visualobject->label) != waitingForDetection.end()) {
	//This was an object we were looking for
	waitingForDetection.erase(visualobject->label);
	waitingForObjects.insert(visualobject->label);

	string logString("Waiting list: ");
	for (std::set<string>::iterator it = waitingForObjects.begin(); it != waitingForObjects.end(); it++) {
	  logString += *it + " ";
	}
	for (std::set<string>::iterator it = waitingForDetection.begin(); it != waitingForDetection.end(); it++) {
	  logString += *it + " ";
	}
	log(logString.c_str());


      }
    }
    catch (const CASTException &e) {
      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
    }
  }

  void VisualObjectSearch::UnsuccessfulDetection(SensingAction viewcone){
    double sensingProb = 1.0;
    GDProbSum sumcells;
    GDIsObstacle isobstacle;
    /* DEBUG */
    GDProbSum conesum;
    m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
	viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, conesum, conesum, m_minDistance);
    cout << "cone sums to" << conesum.getResult() << endl;
    
    /* DEBUG */
     
     cout << "m_pout before update  is:" <<m_pout << endl;
 //to get the denominator first sum all cells
m_map->universalQuery(sumcells); 

double mapsum = 0;
mapsum = sumcells.getResult();
cout << "whole map PDF sums to: " << mapsum << endl;
// then deal with those bloxels that belongs to this cone
    GDMeasUpdateGetDenominator getnormalizer(sensingProb,mapsum);
    m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
	viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, getnormalizer,getnormalizer, m_minDistance);
    double normalizer = getnormalizer.getResult() + m_pout;
  cout << "normalizer is: " << normalizer << endl;

GDProbScale scalefunctor(1.0/normalizer);
m_map->universalQuery(scalefunctor,false);

GDUnsuccessfulMeasUpdate measupdate(normalizer,sensingProb); 
    m_map->coneModifier(viewcone.pos[0], viewcone.pos[1],viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, measupdate,measupdate, m_minDistance);
 
m_pout = m_pout / normalizer;
m_map->universalQuery(sumcells);    
cout << "m_pout after update  is:" <<m_pout << endl;
cout << "map sums to: " << sumcells.getResult() << endl;

normalizePDF(*m_map,(1- m_pout));

vector< pair<double,double> > thresholds;
thresholds.push_back(make_pair(1e-4,1e-3));
thresholds.push_back(make_pair(1e-3,5e-3));
thresholds.push_back(make_pair(5e-3,1e-2));
thresholds.push_back(make_pair(1e-2,5e-2));
thresholds.push_back(make_pair(5e-2,1e2));

pbVis->AddPDF(*m_map,thresholds);
/* //to get the denominator first sum all cells
   m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
   viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, sumcells,sumcells, m_minDistance);
    double probsum = sumcells.getResult();
    // then deal with those bloxels that belongs to this cone
    GDMeasUpdateGetDenominator getnormalizer(m_pout, sensingProb,probsum);
    m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
	viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, getnormalizer,getnormalizer, m_minDistance);
    double normalizer = getnormalizer.getResult();
     cout << "probsum is:" << probsum << endl;
    m_pout = m_pout / probsum;
    cout << "normalizer: " << normalizer << endl;
    //finally set those bloxels that belongs to this cone
    GDUnsuccessfulMeasUpdate measupdate(normalizer,sensingProb); 
    m_map->coneModifier(viewcone.pos[0], viewcone.pos[1],viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, measupdate,measupdate, m_minDistance);
    normalizePDF(*m_map,m_pout); */

  }
  void VisualObjectSearch::newRobotPose(const cdl::WorkingMemoryChange &objID) 
  {
    try {
      lastRobotPose =
	getMemoryEntry<NavData::RobotPose2d>(objID.address);
      m_SlamRobotPose.setX(lastRobotPose->x);
      m_SlamRobotPose.setY(lastRobotPose->y);
      m_SlamRobotPose.setTheta(lastRobotPose->theta);

      Cure::Pose3D cp = m_SlamRobotPose;
      m_TOPP.defineTransform(cp);


    }
    catch (DoesNotExistOnWMException e) {
      log("Error! robotPose missing on WM!");
      return;
    }

  }

  void VisualObjectSearch::receiveOdometry(const Robotbase::Odometry &castOdom)
  {
    lockComponent(); //Don't allow any interface calls while processing a callback
    Cure::Pose3D cureOdom;
    CureHWUtils::convOdomToCure(castOdom, cureOdom);

    debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
	cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
	cureOdom.getTime().getDouble());
    m_TOPP.addOdometry(cureOdom);
    unlockComponent();
  }


  void VisualObjectSearch::receiveScan2d(const Laser::Scan2d &castScan)
  {
    lockComponent();
    debug("Got scan with n=%d and t=%ld.%06ld",
	castScan.ranges.size(), 
	(long)castScan.time.s, (long)castScan.time.us);

    GDIsObstacle obstacle;
    GDMakeObstacle makeobstacle;
    GDMakeFree makefree;
    Cure::LaserScan2d cureScan;
    CureHWUtils::convScan2dToCure(castScan, cureScan);
    if (m_TOPP.isTransformDefined()) {
      Cure::Pose3D scanPose;
      if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
	Cure::Pose3D lpW;
	lpW.add(scanPose, m_LaserPoseR);
	//add tracer
	vector<double> LaserPose;
	LaserPose.push_back(lpW.getX()); //lpW.getY(), 0.4 , lpW.getTheta()};
	LaserPose.push_back(lpW.getY());
	LaserPose.push_back(lpW.getZ());
	LaserPose.push_back(lpW.getTheta());

	debug("Adding scan..");
	m_Mutex.lock();
	m_tracer->addScanStationarySensor(castScan,LaserPose,obstacle,makefree,makeobstacle);
	m_Glrt->addScan(cureScan, lpW,1.0);
	m_Mutex.unlock();
    }
  }
  unlockComponent();
}

void VisualObjectSearch::PostNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype) {
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

void VisualObjectSearch::addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd, 
    std::string label, std::string visualObjectID){
  log("posting recognizer command.");
  VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
  rec_cmd->cmd = cmd;
  rec_cmd->label = label;
  rec_cmd->visualObjectID = visualObjectID;
  addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
}
}
