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

    AVSComponentPtr = this;
    map<string,string>::const_iterator it = _config.find("-c");
    if (it== _config.end()) {
      println("configure(...) Need config file (use -c option)\n");
      std::abort();
    }
    std::string configfile = it->second;


    Cure::ConfigFileReader cfg;
    if (cfg.init(configfile)) {
      println("configure(...) Failed to open with \"%s\"\n",
	  configfile.c_str());
      std::abort();
    }

    if (cfg.getSensorPose(1, m_LaserPoseR)) {
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
    log("Cure map file: %s", m_curemapfile.c_str());
    GridMapData def;
    def.occupancy = UNKNOWN;
    //std::vector< pair<std::string,double> > objectprobability;
    //objectprobability.push_back(make_pair("ricebox",0));
    //def.objprob = objectprobability;
    def.pdf = 0;
    m_map = new SpatialGridMap::GridMap<GridMapData>(m_gridsize, m_gridsize, m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
    m_tracer = new LaserRayTracer<GridMapData>(m_map,1.0);
    pbVis = new VisualPB_Bloxel("localhost",5050,m_gridsize,m_gridsize,m_cellsize,1,true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time

    m_lgm = new Cure::LocalGridMap<unsigned char>(m_gridsize/2, m_cellsize, '2', Cure::LocalGridMap<unsigned char>::MAP1);
    m_Glrt  = new Cure::ObjGridLineRayTracer<unsigned char>(*m_lgm);
    pbVis->connectPeekabot();

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

       addChangeFilter(createLocalTypeFilter<
       SpatialData::NavCommand> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtNavCommand));
*/
       addChangeFilter(createGlobalTypeFilter<
       VisionData::Recognizer3DCommand> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtRecognizer3DCommand));
     
       addChangeFilter(createGlobalTypeFilter<
       FrontierInterface::ObjectPriorRequest> (cdl::OVERWRITE),
       new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
       &VisualObjectSearch::owtWeightedPointCloud));

//    addChangeFilter(createGlobalTypeFilter<
//	VisionData::VisualObject> (cdl::OVERWRITE),
//	new MemberFunctionChangeReceiver<VisualObjectSearch> (this,
//	  &VisualObjectSearch::newVisualObject));

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
	  for(int i = -1; i <= 1; i++){
	    for (int j=-1; j <= 1; j++){
	      if((*m_lgm)(x+i,y+j) != '2' && (bloxelX+i <= m_gridsize && bloxelX+i > 0 ) 
		  && (bloxelY+i <= m_gridsize && bloxelY+i > 0 ))
		m_map->boxSubColumnModifier(bloxelX+i,bloxelY+j, 0, m_mapceiling,initfunctor);
	    }
	  }
	}
      }
    }
    //normalizePDF(*m_map, initfunctor.getTotal());
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
	    m_map->boxSubColumnModifier(x+m_lgm->getSize(),y + m_lgm->getSize(), 0, m_mapceiling,makeobstacle);
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
    InitializePDF(1);
    pbVis->DisplayMap(*m_map);
    log("myline %i", __LINE__);
    pbVis->AddPDF(*m_map, thresholds);
    log("myline %i", __LINE__);
     pbVis->Display2DCureMap(m_lgm);
    log("myline %i", __LINE__);
  //  m_maploaded = true;
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
    AVSComponentPtr->LookforObjectWithStrategy("rice",DIRECT_UNINFORMED);
  }
  void VisualObjectSearch::selectdi( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->LookforObjectWithStrategy("rice",DIRECT_INFORMED);
  }
  void VisualObjectSearch::selectind( GtkWidget *widget, gpointer data )
  {
    AVSComponentPtr->searchChainPos = 0;
    AVSComponentPtr->LookforObjectWithStrategy("rice",INDIRECT);
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
      log("I am running!"); 
      m_Mutex.lock();
//      pbVis->DisplayMap(*m_map);
//      pbVis->Display2DCureMap(m_lgm);
      while(gtk_events_pending())
	gtk_main_iteration();

    log("myline %i", __LINE__);
      InterpretCommand();
    log("myline %i", __LINE__);
      if(m_maploaded)
	SampleAndSelect();
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
      case GOTO_NEXT_NBV:{
			   m_command = IDLE;
			   GoToNextBestView();
			 }
      case ASK_FOR_DISTRIBUTION:{
				  m_command=IDLE;
				  AskForDistribution();
				}
      case RECOGNIZE:{
		       m_command = IDLE;
      		addRecognizer3DCommand(VisionData::RECOGNIZE, currentTarget,"");
		     }
      case IDLE:{
		  break;
		}
    }
  }

  void VisualObjectSearch::owtNavCommand(const cast::cdl::WorkingMemoryChange &objID){
    log("I do naaathing.");
  
    try{
      log("nav command overwritten");
    SpatialData::NavCommandPtr cmd(getMemoryEntry<
        SpatialData::NavCommand> (objID.address));
    if (cmd->comp == SpatialData::COMMANDSUCCEEDED) {
      MovePanTilt(0,m_currentVP.tilt,0.1);
      m_command = RECOGNIZE;
    }
  
  }
    catch(std::exception e) { }
  }
  void VisualObjectSearch::LookforObjectWithStrategy(std::string name, SearchMode mode){
    //ask   
    if (mode == DIRECT_UNINFORMED){
      currentSearchMode = DIRECT_UNINFORMED;
      currentTarget  = name;
      m_command = GOTO_NEXT_NBV;
    }
    else if (mode == DIRECT_INFORMED){
      currentSearchMode = DIRECT_INFORMED;
      currentTarget = name;
      m_command = ASK_FOR_DISTRIBUTION;
    }
    else if(mode == INDIRECT){
      currentSearchMode = INDIRECT;
      ObjectPairRelation rel;
      while(true){
	rel = GetSecondaryObject(name);
	if(rel.primaryobject == "")
	  break;
	searchChain.push_back(rel);
	name = rel.secobject;
      }
      for(unsigned int i = 0; i < searchChain.size(); i++){
	cout << searchChain[i].primaryobject << " " << searchChain[i].secobject << " " << searchChain[i].prob << endl;
      }
      m_command = ASK_FOR_DISTRIBUTION;
    }
  }

  void VisualObjectSearch::AskForDistribution(){
    log("Asking for distribution");
    m_command = IDLE;
    //if informed direct, get search chain, construct distribution parameters and ask for it
    if (currentSearchMode == DIRECT_INFORMED){
      vector<FrontierInterface::ObjectRelation> relations;
      vector<string> labels;
      ObjectPairRelation rel;
      string name = currentTarget;
      while(true){
	//finally set those bloxels that belongs to this conee(true){
	rel = GetSecondaryObject(name);
	if(rel.primaryobject == "")
	  break;
	searchChain.push_back(rel);
	name = rel.secobject;
      }
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
      addToWorkingMemory(newDataID(), objreq);

      cout << labels[0] << endl;
      for(unsigned int i = 0; i < relations.size(); i++){
	cout << relations[i]<< " " << labels[i + 1]  << endl;
      }

    }

    else if (currentSearchMode == INDIRECT){
    }
    //if indirect look where we are in the chain ask for it's distribution
  }

  void
    VisualObjectSearch::owtWeightedPointCloud(const cast::cdl::WorkingMemoryChange &objID) {
      try {
	log("got weighted PC");
	FrontierInterface::WeightedPointCloudPtr cloud =
	  getMemoryEntry<FrontierInterface::ObjectPriorRequest>(objID.address)->outCloud;

	if(cloud->isBaseObjectKnown) {
	  log("Got distribution around known object pose");
	  m_sampler.kernelDensityEstimation3D(*m_map, cloud->center,
	      cloud->interval,
	      cloud->xExtent,
	      cloud->yExtent,
	      cloud->zExtent,
	      cloud->values,
	      1.0,
	      1.0
	      );
	  normalizePDF(*m_map);
	}
	else {
	  log("Got distribution around unknown object pose");
//	if(currentSearchMode == DIRECT_INFORMED){
//	  log("got direct point cloud");
	  // the cloud is centered around 0,0, TODO: center the cloud 
	  // on possible locations of the most supportive object, hint use Sample and Select
	  // for this, ask Alper if you are not already him.

	}
//	else if(currentSearchMode == INDIRECT){
//
//	}

	vector< pair<double,double> > thresholds;
	thresholds.push_back(make_pair(1e-4,1e-3));
	thresholds.push_back(make_pair(1e-3,5e-3));
	thresholds.push_back(make_pair(1e-2,5e-2));
	thresholds.push_back(make_pair(5e-2,1));
	pbVis->AddPDF(*m_map, thresholds);
	}
      catch (DoesNotExistOnWMException excp) {
	log("Error!  WeightedPointCloud does not exist on WM!");
	return;
      }
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

  void VisualObjectSearch::GoToNextBestView(){

    m_currentVP = SampleAndSelect();
    Cure::Pose3D pos;
    pos.setX(m_currentVP.pos[0]);
    pos.setY(m_currentVP.pos[1]);
    pos.setTheta(m_currentVP.pan);
    PostNavCommand(pos, SpatialData::GOTOPOSITION);
  }
  VisualObjectSearch::SensingAction VisualObjectSearch::SampleAndSelect(){

    double cameraheight = 1.4;
    debug("Sampling Grid.");

    double  tiltRange;
    tiltRange= 20;
    std::vector< pair<int,int> > freespace;
    std::vector< std::vector<double> >  visualizationpoints;
    SensingAction sample;
    std::vector <SensingAction> samplepoints;
    isRegionFree<GridMapData> isfree;
    //Get X number of samples

    for (int x = -m_lgm->getSize(); x < m_lgm->getSize(); x++){
      for (int y = -m_lgm->getSize(); y< m_lgm->getSize(); y++){
	if ((*m_lgm)(x,y) == '0'){
	  pair<int,int> coord(x,y);
	  freespace.push_back(coord);
	}
      }
    }

    cout << "There are " << freespace.size() << " free points in cure map." << endl;
    if (freespace.size() == 0){
      SensingAction tmp;
      return tmp;
    }
    srand(time(NULL));
    int i = 0;
    while (i< m_samplesize){
      int randomPos = rand() % freespace.size();
      double randomTilt = (rand() % 2*tiltRange - tiltRange)*M_PI/180;
      double randomPan = (rand() % 360 - 180)*M_PI/180;

      pair<int,int> samplepoint(freespace[randomPos].first,freespace[randomPos].second);
      //  pair<int,int> samplepoint(rand() % xGrid,rand() % yGrid);
      // Convert cure coordinates to Bloxel Grid coordinates
      int bloxelX = samplepoint.first + m_gridsize/2;
      int bloxelY = samplepoint.second + m_gridsize/2;
      pair<double,double> worldCoords = m_map->gridToWorldCoords(bloxelX,bloxelY);

      if (isCircleFree(worldCoords.first, worldCoords.second, 0.1) && (*m_lgm)(samplepoint.first,samplepoint.second) == '0'){
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

    //TODO: Add tilting and panning
    int bestConeIndex = GetViewConeSums(samplepoints);
    return samplepoints[bestConeIndex];
  }


  int VisualObjectSearch::GetViewConeSums(std::vector <SensingAction> samplepoints){
    debug("Querying cones");

    GDProbSum sumcells;
    GDIsObstacle isobstacle;
    double maxpdf = -1000.0;
    int maxindex = -1;

    try{ 
      for (unsigned int i =0; i < samplepoints.size(); i++){
	cout << "cone #" << i << " at: " << samplepoints[i].pos[0] << "," << samplepoints[i].pos[1] << "," << samplepoints[i].pos[2] << endl; 

	/*m_map->coneModifier(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan,samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, makeobstacle,makeobstacle);*/
	m_map->coneQuery(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan, samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 0, 0, isobstacle, sumcells,sumcells);
	if (sumcells.getResult() > maxpdf){
	  maxpdf = sumcells.getResult();
	  maxindex = i;
	}
      }
    }
    catch(std::exception &e) {
      printf("Caught exception %s: \n", e.what());
    }
      return maxindex;
  }


  void
   VisualObjectSearch::owtRecognizer3DCommand(const cast::cdl::WorkingMemoryChange &objID) {
    if(isWaitingForDetection){
      try{
        log("got recognizer3D overwrite command");
        VisionData::Recognizer3DCommandPtr cmd(getMemoryEntry<
            VisionData::Recognizer3DCommand> (objID.address));
        if (cmd->label == currentTarget){
          isWaitingForDetection = false;
          DetectionComplete(false);
        }

    }
      catch (const CASTException &e) {
        //      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
          }
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

 void VisualObjectSearch::DetectionComplete(bool isDetected){
   if(true){
     m_command = IDLE;
     log("Object found.");
   }
   else
   {
     UnsuccessfulDetection(m_currentVP);
     m_command = GOTO_NEXT_NBV;
    }

}
  void
    VisualObjectSearch::newVisualObject(const cast::cdl::WorkingMemoryChange &objID) {

      if (isWaitingForDetection){
      try{
	log("new visual object");
	VisionData::VisualObjectPtr visualobject(getMemoryEntry<
	    VisionData::VisualObject> (objID.address));

	if (visualobject->label == currentTarget){
	isWaitingForDetection = false;
	DetectionComplete(true);
	 }

      }
      catch (const CASTException &e) {
	      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
      }
      }
    }

  void VisualObjectSearch::UnsuccessfulDetection(SensingAction viewcone){
    double pOut = 0.3;
    double sensingProb = 0.7;
    GDProbSum sumcells;
    GDIsObstacle isobstacle;
    //to get the denominator first sum all cells
    m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
	viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, sumcells,sumcells);
    double probsum = sumcells.getResult();
    // then deal with those bloxels that belongs to this cone
    GDMeasUpdateGetDenominator getnormalizer(pOut, sensingProb,probsum);
    m_map->coneQuery(viewcone.pos[0],viewcone.pos[1],
	viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, getnormalizer,getnormalizer);
    double normalizer = getnormalizer.getResult();
    //finally set those bloxels that belongs to this cone
    GDUnsuccessfulMeasUpdate measupdate(normalizer,sensingProb); 
    m_map->coneModifier(viewcone.pos[0], viewcone.pos[1],viewcone.pos[2], viewcone.pan, viewcone.tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, measupdate,measupdate);

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
  isWaitingForDetection = true;
}
}
