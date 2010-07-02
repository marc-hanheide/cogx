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

      m_minbloxel = 0.1;
      it = _config.find("--minbloxel");
      if (it != _config.end()) {
	m_minbloxel = (atof(it->second.c_str()));
	log("Min bloxel height set to: %f", m_minbloxel);
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

      GridMapData def;
      def.occupancy = UNKNOWN;
      //std::vector< pair<std::string,double> > objectprobability;
      //objectprobability.push_back(make_pair("ricebox",0));
      //def.objprob = objectprobability;
      def.pdf = 0;
      m_map = new GridMap<GridMapData>(m_gridsize, m_gridsize, m_cellsize, m_minbloxel, 0, m_mapceiling, 0, 0, 0, def);
      m_tracer = new LaserRayTracer<GridMapData>(m_map,1.0);
      p = new VisualPB_Bloxel("localhost",5050,m_gridsize,m_gridsize,m_cellsize,1,true);//host,port,xsize,ysize,cellsize,scale, redraw whole map every time

      m_lgm = new Cure::LocalGridMap<unsigned char>(m_gridsize/2, m_cellsize, '2', Cure::LocalGridMap<unsigned char>::MAP1);
      m_Glrt  = new Cure::ObjGridLineRayTracer<unsigned char>(*m_lgm);
      p->connectPeekabot();


    }
    void
      VisualObjectSearch::start() {
	log("I am started");

	log("I have started");
	setupPushScan2d(*this, 0.1);
	setupPushOdometry(*this);

	addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
	    new MemberFunctionChangeReceiver<VisualObjectSearch>(this,
	      &VisualObjectSearch::newRobotPose));

	addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
	    new MemberFunctionChangeReceiver<VisualObjectSearch>(this,
	      &VisualObjectSearch::newRobotPose));

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


 void VisualObjectSearch::ReadCureMapFromFile() {
   log("Reading cure map");
   int length;
   char * buffer;
   ifstream file("curemap.txt");
   if (!file.good()){
   log("Could not open file, returning without doing anything.");
   return;
   }
   file.seekg(0, ios::end);
   length = file.tellg();
   file.seekg(0, ios::beg);
   buffer = new char[length];
   file.read(buffer, length);
   int index = 0;
   for (int x = -m_lgm->getSize(); x <= m_lgm->getSize(); x++) {
     for (int y = -m_lgm->getSize(); y <= m_lgm->getSize(); y++) {
       char c = buffer[index];
       (*m_lgm)(x, y) = c;
       index++;
     }
   }
 }

 void VisualObjectSearch::savemap( GtkWidget *widget,gpointer data )
 {
   AVSComponentPtr->SaveCureMapToFile();
 }
 void VisualObjectSearch::readmap( GtkWidget *widget, gpointer data )
 {
   AVSComponentPtr->ReadCureMapFromFile();
 }


 void VisualObjectSearch::runComponent(){

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


	//parent: window, child: hbox
	gtk_container_add (GTK_CONTAINER (window), hbox);
	gtk_container_set_border_width (GTK_CONTAINER (window), 5);
	gtk_widget_show (hbox);

	//callbacks

	g_signal_connect (G_OBJECT (savebutton), "clicked",
	    G_CALLBACK (&spatial::VisualObjectSearch::savemap), NULL);
	g_signal_connect (G_OBJECT (readbutton), "clicked",
	    G_CALLBACK (&spatial::VisualObjectSearch::readmap), NULL);
	//add buttons to box
	gtk_box_pack_start (GTK_BOX (hbox), readbutton, TRUE, TRUE, 0);
	gtk_box_pack_start (GTK_BOX (hbox), savebutton, TRUE, TRUE, 0);

	gtk_widget_show (readbutton);
	gtk_widget_show (savebutton);
	gtk_widget_show (window);
      }

      while(true){
	log("I am running!");
	m_Mutex.lock();
	//SampleAndSelect();
	//p->DisplayMap(*m_map);
	p->Display2DCureMap(m_lgm);
	while(gtk_events_pending())
		gtk_main_iteration();
	m_Mutex.unlock();
	sleep(1);
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

 int VisualObjectSearch::GetFreeSpace(){
   int count = 0;

   for (int x = -m_lgm->getSize(); x < m_lgm->getSize(); x++){
     for (int y = -m_lgm->getSize(); y< m_lgm->getSize(); y++){
       if ((*m_lgm)(x,y) == '0'){
        count++;
       }
     }
   }

   return count;
 }
 void VisualObjectSearch::SampleAndSelect(){

   double cameraheight = 1.4;
   int numSamples = 1;
   int xGrid = 200;
   int yGrid = 200;
   debug("Sampling Grid.");

   pair<double,double> panRange, tiltRange;
   panRange.first = -20;
   panRange.second = 20;
   tiltRange.first = 0;
   tiltRange.first = 0;
   std::vector< pair<int,int> > samples,freespace;
   std::vector < std::vector<double> > visualizationpoints;
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
     return;
   }
   //srand(time(NULL));
   int i = 0;
   GDMakeObstacle makeobstacle;
   while (i< numSamples){
     int randomPos = rand() % freespace.size();
     pair<int,int> samplepoint(freespace[randomPos].first,freespace[randomPos].second);
     //  pair<int,int> samplepoint(rand() % xGrid,rand() % yGrid);
     // Convert cure coordinates to Bloxel Grid coordinates

     int bloxelX = samplepoint.first + 100;
     int bloxelY = samplepoint.second + 100;
     pair<double,double> worldCoords = m_map->gridToWorldCoords(bloxelX,bloxelY);

     if (find(samples.begin(), samples.end(),samplepoint) == samples.end()  && isCircleFree(worldCoords.first, worldCoords.second, 0.4) 
	 && (*m_lgm)(samplepoint.first,samplepoint.second) == '0'){
       //cout << "sample point: " << samplepoint.first << "," << samplepoint.second << endl;
       //check if the sample point too close to an obstacle by a box query 
       //cout << "bloxel world coords : " << worldCoords.first << "," << worldCoords.second << endl;
       double xW,yW;           
       m_lgm->index2WorldCoords(samplepoint.first,samplepoint.second,xW,yW);
       std::vector<double> coord;
       //cout << "cure world coords : " << xW << "," << yW << endl;
       coord.push_back(xW);
       coord.push_back(yW);
       coord.push_back(1.4);
       visualizationpoints.push_back(coord);
       i++;
       // add this for visualization
     }
   }
   p->Add3DPointCloud(visualizationpoints);

   //TODO: Add tilting and panning
   GetViewConeSums(visualizationpoints);
 }


 std::vector<double> VisualObjectSearch::GetViewConeSums(std::vector < std::vector<double> > samplepoints){
  debug("Querying cones");
  
  GDProbSum sumcells;
  GDIsObstacle isobstacle;
  GDMakeObstacle makeobstacle;
  GDProbAdd addval(1/ (GetFreeSpace()*m_mapceiling));
  double maxpdf = -1000.0;
  int maxindex = -1;

  try{ 
    for (unsigned int i =0; i < samplepoints.size(); i++){
      cout << "cone for: " << samplepoints[i][0] << "," << samplepoints[i][1] << "," << samplepoints[i][2] << endl; 

      /* m_map->coneModifier(samplepoints[i][0],samplepoints[i][1],samplepoints[i][2], 0, 0 , 
	 m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, addval,addval);*/
      m_map->coneQuery(samplepoints[i][0],samplepoints[i][1],samplepoints[i][2], 0, 0 , 
	  m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, sumcells,sumcells);
      if (sumcells.getResult() > maxpdf){
	maxpdf = sumcells.getResult();
	maxindex = i;
      }
    }
  }
  catch(std::exception &e) {
    printf("Caught exception %s: \n", e.what());
  }

  return std::vector<double>();
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

}
