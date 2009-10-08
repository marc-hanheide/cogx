#include "ObjectSearch.hpp"
#include <NavData.hpp>
#include <VisionData.hpp>
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <Navigation/GridContainer.hh>
#include "ObjPdf.hpp"
#include "XVector3D.h"
#include <FrontierInterface.hpp>
#include <float.h>

using namespace cast;
using namespace std;
using namespace boost;
//using namespace Cure;
extern "C" {
    cast::CASTComponentPtr
    newComponent() {
        return new ObjectSearch();
    }
}

ObjectSearch::ObjectSearch() {}

ObjectSearch::~ObjectSearch() {}
void ObjectSearch::start() {
    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                    new MemberFunctionChangeReceiver<ObjectSearch>(this,
                            &ObjectSearch::newRobotPose));

    addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                    new MemberFunctionChangeReceiver<ObjectSearch>(this,
                            &ObjectSearch::newRobotPose));
    addChangeFilter(createLocalTypeFilter<NavData::NavCommand>(cdl::ADD),
                    new MemberFunctionChangeReceiver<ObjectSearch>(this,
                            &ObjectSearch::owtNavCommand));

    addChangeFilter(createLocalTypeFilter<NavData::NavCommand>(cdl::OVERWRITE),
                 new MemberFunctionChangeReceiver<ObjectSearch>(this,
                            &ObjectSearch::owtNavCommand));

    addChangeFilter(createChangeFilter<VisionData::VisualObject>
		    (cdl::ADD,
		     "",
		     "",
		     "vision.sa",
		     cdl::ALLSA),
		    new MemberFunctionChangeReceiver<ObjectSearch>(this,
								   &ObjectSearch::ObjectDetected));
    
    addChangeFilter(createChangeFilter<VisionData::VisualObject>
		    (cdl::OVERWRITE,
		     "",
		     "",
		     "vision.sa",
		     cdl::ALLSA),
		    new MemberFunctionChangeReceiver<ObjectSearch>(this,
								   &ObjectSearch::ObjectDetected));
    
    addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::ADD),
		    new MemberFunctionChangeReceiver<ObjectSearch>(this,
								   &ObjectSearch::newNavGraphNode));  
    addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::OVERWRITE),
		    new MemberFunctionChangeReceiver<ObjectSearch>(this,
								   &ObjectSearch::newNavGraphNode)); 
                                        
     
	addChangeFilter(createLocalTypeFilter<SpatialData::AVSCommand>(cdl::ADD),
                  new MemberFunctionChangeReceiver<ObjectSearch>(this,
                                        &ObjectSearch::newAVSCommand));  
                                        
  addChangeFilter(createLocalTypeFilter<SpatialData::AVSCommand>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<ObjectSearch>(this,
                                        &ObjectSearch::newAVSCommand)); 
}

void ObjectSearch::newAVSCommand(const cdl::WorkingMemoryChange &objID){
	  
	  shared_ptr<CASTData<SpatialData::AVSCommand> > oobj =
    	getWorkingMemoryEntry<SpatialData::AVSCommand>(objID.address);
    	
    	if (oobj->getData()->cmd == SpatialData::PLAN){
    	
    	    placestosearch = oobj->getData()->placestosearch;
	  		m_command = PLAN;
	}
	else if (oobj->getData()->cmd == SpatialData::STOPAVS)
			m_command = STOP;
}
void ObjectSearch::newNavGraphNode(const cdl::WorkingMemoryChange &objID)
{
  debug("new NavGraphNode");

  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  NavData::FNodePtr fnode = oobj->getData();
  fnodeseq.push_back(fnode);
  
}
void ObjectSearch::configure(const map<string,string>& _config) {
    log("Configuring ObjectSearch");
    map<string,string>::const_iterator it = _config.find("-c");

    if (it== _config.end()) {
        log("configure(...) Need config file (use -c option)\n");
        std::abort();
    }
    std::string configfile = it->second;
    Cure::ConfigFileReader cfg;
    if (cfg.init(configfile)) {
        log("configure(...) Failed to open with %s\n",
            configfile.c_str());
        std::abort();
    }
    //Exploration range
    m_MaxExplorationRange = 1;
    it = _config.find("--explore-range");
    if (it != _config.end()) {
        m_MaxExplorationRange = (atof(it->second.c_str()));
    }

	m_fov = M_PI/4;
    it = _config.find("--cam-fov");
    if (it != _config.end()) {
        m_fov = (atof(it->second.c_str()))*M_PI/180;
    }
	

	m_ptustep = M_PI/6;
    it = _config.find("--cam-step");
    if (it != _config.end()) {
        m_ptustep = (atof(it->second.c_str()))*M_PI/180;
    }
    //Coverage percent treshold
    m_covthresh = 70.0;
    it = _config.find("--coverage-threshold");
    if (it != _config.end()) {
        m_covthresh = (atof(it->second.c_str()));
    }
    log("Coverage threshold set to: %f",m_covthresh);


    m_vpthreshold = 10.0;
    it = _config.find("--vp-threshold");
    if (it != _config.end()) {
        m_vpthreshold = (atof(it->second.c_str()));
    }
    log("Viewpoint threshold set to: %f",m_vpthreshold);

    m_CamRange = 1;
    it = _config.find("--cam-range");
    if (it != _config.end()) {
        m_CamRange = (atof(it->second.c_str()));
    }
    log("Camera range set to: %f",m_CamRange);


    m_awayfromobstacles = 1.0;
    it = _config.find("--away-from-obstacles");
    if (it != _config.end()) {
        m_awayfromobstacles = (atof(it->second.c_str()));
    }
    log("Away from obstacle set to: %f",m_awayfromobstacles);

    //Laser pose
    if (cfg.getSensorPose(1, m_LaserPoseR)) {
        log("configure(...) Failed to get sensor pose");
        std::abort();
    }


    m_gridsize = 400;
    m_cellsize = 0.1;
    it = _config.find("--gridsize");
    if (it != _config.end()) {

        m_gridsize = (atof(it->second.c_str()));
        log("Gridsize set to: %f",m_CamRange);
    }


    it = _config.find("--cellsize");
    if (it != _config.end()) {
        m_cellsize = (atof(it->second.c_str()));
        log("Cellize set to: %f",m_cellsize);
    }

    int magnification = 1;
    it = _config.find("--disp-magn");
    if (it != _config.end()) {
        magnification = (atoi(it->second.c_str()));
        log("Display magnification set to: %f", magnification);
    }

    m_CtrlPTU = (_config.find("--ctrl-ptu") != _config.end());

    m_samplesize = 100;
    it = _config.find("--samplesize");
    if (it != _config.end()) {
        m_samplesize = (atof(it->second.c_str()));
        log("Samplesize set to: %d",m_samplesize);
    }
    m_samples = new int[2*m_samplesize];
    m_samplestheta = new double[m_samplesize];
    m_lgm = new Cure::LocalGridMap<double>(m_gridsize/2, m_cellsize, 256, Cure::LocalGridMap<float>::MAP1);
    m_Glrt  = new Cure::ObjGridLineRayTracer<double>(*m_lgm);
    //m_lmap = new LocalMap(*m_lgm);
    // X window setting
    coveragemap = new Cure::LocalGridMap<unsigned int>(m_gridsize/2, m_cellsize, 0, Cure::LocalGridMap<unsigned int>::MAP1);
	pdf = new Cure::LocalGridMap<float>(m_gridsize/2, m_cellsize, 0, Cure::LocalGridMap<float>::MAP1);
    if (_config.find("--no-x-window") == _config.end()) {

      m_Displaylgm = new Cure::X11DispLocalGridMap<double>(*m_lgm,magnification);
        log("Will use X window to show the exploration map");
    } else {
        m_Displaylgm = 0;
        log("Will NOT use X window to show the exploration map");
    }

    if (_config.find("--display-coverage") != _config.end()) {
      m_Displaycoverage = new Cure::X11DispLocalGridMap<unsigned int>(*coveragemap,magnification);
    } else {
        m_Displaycoverage = 0;
    }

    displayOn = true;
    //Objects
    if((it = _config.find("--objects")) != _config.end()) {
        istringstream istr(it->second);
        string label;
        Object* obj;

        while(istr >> label) {
            obj = new Object();
            obj->ObjID = label;
            m_objectlist.push_back(obj);
        }
    }
    log("Loaded objects.");
	
    cmp = NavData::SUCCEEDED;
    
	m_coveragetotal = -1;
    m_covered = 0;
    m_status = STOPPED;
    whereinplan = -1;

    if (m_CtrlPTU)  {
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
      m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);
    }

    log("Configured ObjectSearch");
}

void ObjectSearch::runComponent() {
    log("ObjectSearch running ");

    setupPushScan2d(*this, -1);
    setupPushOdometry(*this, -1);
	MovePanTilt(20 , 5*M_PI/180);

    //clock_t start_time,elapsed;
    //double elapsed_time;
    m_command = IDLE; //TURN;
	log("hey I run");
    while(isRunning()) {
		
        Update_PDF_with_GridMap();
        Update_CoverageMap_with_GridMap();
        InterpretCommand ();
        UpdateDisplays();
        sleep(1);
    }
}
void ObjectSearch::MovePanTilt(double pan,double tolerance){
		if (m_CtrlPTU)
		{
		log(" Moving pantilt to: %f with %f tolerance", pan, tolerance);
		ptz::PTZPose p;
		ptz::PTZReading ptuPose;
		p.pan = pan ;
		p.tilt = 0.0;
		p.zoom = 0;
		m_PTUServer->setPose(p);
		bool run = true;
		ptuPose = m_PTUServer->getPose();
		double actualpose = ptuPose.pose.pan;
		while(run){
			m_PTUServer->setPose(p);
			ptuPose = m_PTUServer->getPose();
			actualpose = ptuPose.pose.pan;
			log("actualpose is: %f", actualpose);
			if (pan > actualpose){
				if (actualpose > abs(pan) - tolerance){
					log("false actualpose is: %f, %f", actualpose, abs(pan) + tolerance);
					run = false;
				}
					}
			if (actualpose > pan){
				if (actualpose < abs(pan) + tolerance)
					run = false;
					}
		    if(pan == actualpose)
				run = false;
			
			usleep(10000);
		}
		log("Moved.");
		sleep(1);
		}
}
NavData::ObjectSearchPlanPtr ObjectSearch::ConvertPlantoIce()
{ 
	NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
	cogx::Math::Vector3 a;
	for (unsigned int i = 0; i < m_plan.plan.size(); i++){
		a.x = m_plan.plan[i].getX(); a.y = m_plan.plan[i].getY(); a.z = m_plan.plan[i].getTheta();
		obs->planlist.push_back(a);
	}
	return obs;
}
void ObjectSearch::Plan () {
	
	Cure::LocalGridMap<unsigned int> fcm(*coveragemap);
    GenViewPoints();
    m_plan = GeneratePlan(m_covthresh, ScorebyCoverage(fcm));
	addToWorkingMemory(newDataID(), ConvertPlantoIce());
    log("Plan generated %i view points with %f coverage",m_plan.plan.size(),m_plan.totalcoverage);
	
    if (true) {
      m_command = EXECUTE;
    } else {
        m_command = PLAN;
    }
}
void ObjectSearch::InterpretCommand () {
    switch(m_command) {
    case STOP: {
            m_command = IDLE;
            log("Command: STOP");
            NavData::NavCommandPtr cmd = new NavData::NavCommand;
            cmd->prio = NavData::URGENT;
            cmd->cmd = NavData::STOP;
            addToWorkingMemory<NavData::NavCommand>(newDataID(), cmd);
            break;
        }
    case TURN: {
            log("Command: TURN");
            m_command = IDLE;
            NavData::NavCommandPtr cmd = new NavData::NavCommand;
            cmd->prio = NavData::URGENT;
            cmd->cmd = NavData::TURN;
            cmd->angle.resize(1);
            cmd->angle[0] = M_PI;
            addToWorkingMemory<NavData::NavCommand>(newDataID(), cmd);
            break;
        }
    case PLAN:
        log("Command: PLAN");
        m_command = IDLE;
		m_status = PLANNING;
        Plan();
        break;
    case EXECUTE:
        log("Command: EXECUTE");
        m_command = IDLE;
		m_status = EXECUTINGPLAN;
        ExecutePlan();
        break;
	case RECOGNIZE:
	log("Command: RECOGNIZE");
		m_command = IDLE;
		m_status = RECOGNITIONINPROGRESS;
		Recognize();
    case IDLE:
		if(m_status == RECOGNITIONINCOMPLETE )
		{
			m_command = EXECUTE;
			log("Recognition complete. Execute next in plan.");
		}
//        log("Command: IDLE");
        break;
    default:
        log("Command: Default.");
        break;
    }
}
void ObjectSearch::ExecuteNextInPlan () {
	whereinplan++;
    log("Plan size %i, where in plan: %i.",m_plan.plan.size(),whereinplan);
	if (whereinplan >= m_plan.plan.size()){
        log("Plan finished.");
        m_command = IDLE;
        m_status = STOPPED;
        return;
		}
    if (m_plan.plan.size() == 0) {
        log("Nothing to execute.");
        m_command = IDLE;
        m_status = STOPPED;
        return;
    }
    if (m_status != NAVCOMMANDINPROGRESS) {
        log("Posting NavCommand");
        PostNavCommand(m_plan.plan[whereinplan]);
        m_status = NAVCOMMANDINPROGRESS;
        
    } else if (m_status == NAVCOMMANDINPROGRESS) {
        log("NavCommand in progress.");
    }
}
void ObjectSearch::PostNavCommand(Cure::Pose3D position) {
    NavData::NavCommandPtr cmd = new NavData::NavCommand;
    cmd->prio = NavData::URGENT;
    cmd->cmd = NavData::GOTOPOSITION;
    cmd->pose.resize(2);
    cmd->pose[0] = position.getX();
    cmd->pose[1] = position.getY();
	//cmd->pose[2] = position.getTheta();
    cmd->tolerance.resize(1);
    cmd->tolerance[0] = 0.1;
    string id = newDataID();
    log("ID post: %s",id.c_str());
    addToWorkingMemory<NavData::NavCommand>(id, cmd);
}

void ObjectSearch::ExecutePlan () {
		ExecuteNextInPlan ();
}


void ObjectSearch::GenViewPoints() {
	srand ( time(NULL) );
    //log("Generating %i random samples", m_samplesize);
    ViewConePts.clear();
    int randx,randy;
    double xW,yW;
    int i=0;
    std::vector<double> angles;
    log("creating placeinterface proxy");
    FrontierInterface::PlaceInterfacePrx agg(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));
    
    std::vector< boost::shared_ptr<CASTData<NavData::FNode> > > obj;
    while (obj.empty()) {
      getWorkingMemoryEntries<NavData::FNode>(20, obj);	
      usleep(1000);
    }
    NavData::FNodeSequence fnodeseq;
    for (unsigned int i = 0; i < obj.size() ; i++)
    {
    	fnodeseq.push_back(obj[i]->getData());
    }
    
    for (double rad= 0 ; rad < M_PI*2 ; rad = rad + M_PI/90)
        angles.push_back(rad);

    while (i < m_samplesize) {

        randx = rand();
        randy = rand();
        randx = (randx % (m_gridsize)) - m_gridsize/2;
        randy = (randy % (m_gridsize)) - m_gridsize/2;
        m_lgm->index2WorldCoords(randx,randy,xW,yW);
        
        if ((*m_lgm)(randx,randy) == 0) {
            if (m_lgm->isCircleObstacleFree(xW,yW, m_awayfromobstacles) &&
            	m_LMap.goalReachable(xW,yW, 1,0.3)) { //if random point is free space
            	long nodeid = GetClosestFNode(xW,yW);
        		SpatialData::PlacePtr place = agg->getPlaceFromNodeID(nodeid);
        		long id = -1;
        		if (place != NULL)
        			id = place->id;
        	    bool isincluded = false;
        		for (unsigned int q= 0; q < placestosearch.size(); q++){
        			if (placestosearch[q] == id){
        				isincluded = true;
        				break;
        			}
        		}
            	if (isincluded) { //if sample is in a place we were asked to search
                	m_samples[2*i] = randx;
                	m_samples[2*i+1] = randy;
                	int the = (int)(rand() % angles.size());
                	m_samplestheta[i] = angles[the];
                	i++;
            	}
            }
        }
    }

    //log("Calculating view cones for generated samples");
    Cure::Pose3D candidatePose;
    XVector3D a;
    
    for (int y=0; y < m_samplesize; y++) { //calc. view cone for each sample
      
        m_lgm->index2WorldCoords(m_samples[y*2],m_samples[2*y+1],a.x,a.y);
        a.theta =  m_samplestheta[y];
        tpoints = GetInsideViewCone(a, false);
        ViewConePts.push_back(tpoints);
        candidatePose.setTheta(m_samplestheta[y]);
        candidatePose.setX(a.x);
        candidatePose.setY(a.y);
        //log("CurrentPose.Theta : %f", candidatePose.getTheta());
        candidatePoses.push_back(candidatePose);
    }
    //log("View Cones calculated.");
    displayOn = true;
}
long ObjectSearch::GetClosestFNode(double xW, double yW){
	long nodeid;
    double hdist = DBL_MAX;
    for (unsigned int i = 0; i < fnodeseq.size() ; i++){
        double dist = sqrt( pow((xW - fnodeseq[i]->x),2) + pow((yW - fnodeseq[i]->y),2) );
        //log("node pose: %.2f,%.2f dist: %f", fnodeseq[i]->x,fnodeseq[i]->y, dist); 
        if ( dist < hdist){
        	hdist = dist;
        	nodeid = fnodeseq[i]->nodeId;
        }
    }
    //log("closest node id : for point %f,%f is %i",xW,yW,nodeid);
    return nodeid;
}
std::vector<int> ObjectSearch::GetInsideViewCone(XVector3D &a, bool addall) {
	tpoints.clear();
	XVector3D b,c,p;
    XVector3D m_a,m_b,m_c;
    int* rectangle = new int[4];
    int h,k;
        //currentPose = m_TOPP.getPose();
        CalculateViewCone(a,a.theta,m_CamRange,m_fov,b,c);

        m_lgm->worldCoords2Index(a.x,a.y,h,k);
        m_a.x = h;
        m_a.y = k;
        m_lgm->worldCoords2Index(b.x,b.y,h,k);
        m_b.x = h;
        m_b.y = k;
        m_lgm->worldCoords2Index(c.x,c.y,h,k);
        m_c.x = h;
        m_c.y = k;
        //  log("Got Map triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", m_a.x,m_a.y,m_b.x,m_b.y,m_c.x,m_c.y);

        FindBoundingRectangle(m_a,m_b,m_c,rectangle);
        //log("XRectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",rectangle[0],rectangle[2]
        //,rectangle[1], rectangle[3]);
        for (int x=rectangle[0]; x < rectangle[1] ; x++) // rectangle bounding triangle
        {
            for (int y=rectangle[2]; y < rectangle[3]; y++) {
                p.x = x;
                p.y = y;
                if (addall){
                    if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
                        tpoints.push_back(x);
                        tpoints.push_back(y);
                    }
                }
                else {
                if ((*coveragemap)(x,y) == 1) {
                    if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
                        tpoints.push_back(x);
                        tpoints.push_back(y);
                    }
                }
                }
            }
        }
        vector<int>::iterator theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.y);
        theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.x);
        return tpoints;
	
}

std::vector<double> ObjectSearch::ScorebyCoverage(Cure::LocalGridMap<unsigned int> fcm ) {
	std::vector<double> CoverageSum;
	int covered = m_covered;
    for (unsigned int i = 0; i < ViewConePts.size(); i++) {
		double score = GetExtraCoverage(ViewConePts[i], covered, fcm);
        CoverageSum.push_back(score);
        //log("Coverage Sum for view cone %i is: %f",i, score);
	}
	return CoverageSum;
}

std::vector<double> ObjectSearch::ScorebyPDF() {
    //log("Selecting best view cone out of %i random samples.",m_samplesize);
    std::vector<double> PDFsum;
    int x,y;
    double singlesum;
    for (unsigned int i = 0; i < ViewConePts.size(); i++) {
        singlesum = 0;
        for (unsigned int j = 0; j < ViewConePts[i].size()/2; j++) {
            x = ViewConePts[i][2*j];
            y =ViewConePts[i][2*j + 1];
            if((*coveragemap)(x,y) == 1) {
                //println("obs: %f", SuperObject->pdf->pdfgrid[x+lsize][y+lsize]);
                //singlesum = singlesum + SuperObject->pdf->pdfgrid[x+lsize][y+lsize];
            }
        }
        PDFsum.push_back(singlesum);
        debug("PDF Sum for view cone %i is: %f",i, singlesum);
    }
    return PDFsum;
}
bool comp (double i,double j) {
    return (i>j);
}
ObjectSearch::SearchPlan ObjectSearch::GeneratePlan(double covpercent,std::vector<double> PDFsum) {
    SearchPlan searchplan;
    log("GeneratePlan");
    searchplan.plan.clear();
    searchplan.totalcoverage = 0;
    searchplan.indexarray.clear();
    std::vector<double> copy = PDFsum;

    Cure::LocalGridMap<unsigned int> fcm(*coveragemap);
    int covered = m_covered;
    sort(copy.begin(),copy.end(),comp);
    while(copy.size() > 0) {
        double val = copy[0];
        unsigned int j = 0;
        debug("val is: %f",val);
        for (;j < PDFsum.size(); j++) {
            if (PDFsum[j] == val) { //find the index with value val in PDFsum
                //log("found at %ith view point",j);
                break;
            }
        }

        copy.erase(copy.begin()); //chop off the first element so next time in loop copy[0] is the next element
		std::vector<int> rollback;
		double extracoverage = GetExtraCoverage(ViewConePts[j],covered, fcm, true, rollback);
		//println("Extracoverage: %f",extracoverage);
        if (extracoverage > m_vpthreshold) { // if worthy of adding to the plan
        	//just to display the selected viewcone
			
        	XVector3D a;
        	m_lgm->index2WorldCoords(m_samples[j*2],m_samples[2*j+1],a.x,a.y);
        	a.theta =  m_samplestheta[j];
        	ViewConePts[j] = GetInsideViewCone(a, true); 
			searchplan.extracoverage.push_back(extracoverage);
            searchplan.plan.push_back(candidatePoses[j]);
            searchplan.indexarray.push_back(j);
            searchplan.totalcoverage += extracoverage;
            //println("Total Coverage %f",  searchplan.totalcoverage);
            //println("Added to plan");
            if (searchplan.totalcoverage >= m_covthresh) {
              //  println("Found plan covers %f of the area", searchplan.totalcoverage);
			//	println("%i out of  %i points", covered, m_coveragetotal);
				return searchplan;
            }
        } else {
			int x,y;
			for (unsigned int j = 0; j < rollback.size()/2; j++) {
				x = rollback[2*j];
				y = rollback[2*j + 1];
					fcm(x,y) = 1;
			}
        }


    }
    if (searchplan.totalcoverage < m_covthresh) {
        //println("Plan does not cover %f only %f", covpercent, searchplan.totalcoverage);
		//println("%i out of  %i points", covered, m_coveragetotal);
    }
    	return searchplan;
}

double ObjectSearch::GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned int> &fcm) {
	std::vector<int> rollback;
return GetExtraCoverage(tpoints, covered, fcm, false, rollback);
}
double ObjectSearch::GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned int> &fcm, bool changefcm,
										std::vector<int> &rollback ) {
    double oldcov = (double)100*covered/m_coveragetotal;
    int h = 0;
    int x,y;
    for (unsigned int j = 0; j < tpoints.size()/2; j++) {
        x = tpoints[2*j];
        y = tpoints[2*j + 1];
        if ((fcm)(x,y) == 1) {
            h++;
			if(changefcm)
				fcm(x,y) = 2;
			rollback.push_back(x);
			rollback.push_back(y);
        }
    }
    double cov = (double)100*(covered+h)/m_coveragetotal;
    //log("Difference in cov: %f", (cov-oldcov));
	return (cov-oldcov);
}

double ObjectSearch::ModifyCoverageMap(std::vector<int> tpoints) {
    //log("ModifyCoverageMap");
    int x,y;
    for (unsigned int j = 0; j < tpoints.size()/2; j++) {
        x = tpoints[2*j];
        y = tpoints[2*j + 1];
        if ((*m_lgm)(x,y) == 1 && (*coveragemap)(x,y) != 2) {
            m_covered++;
            (*coveragemap)(x,y) = 2; //obstacle covered
        }
    }
    CoveragePercentage = (double)100*m_covered/m_coveragetotal;
    return CoveragePercentage;
}
void ObjectSearch::Update_CoverageMap_with_GridMap() {
    int msize = m_lgm->getSize();

    for (int i=-msize; i < msize; i++) {
        for (int j=-msize; j < msize; j++) {
            if ((*m_lgm)(i,j) == 1 && (*coveragemap)(i,j) == 0) {
                m_coveragetotal++;
                (*coveragemap)(i,j) = 1; //obstacle not yet covered
            }
        }
    }
    CoveragePercentage = (double)100*m_covered/m_coveragetotal;
}

void ObjectSearch::Update_PDF_with_GridMap() {
	int msize = m_lgm->getSize();
    for (int i=-msize; i < msize; i++) {
        for (int j=-msize; j < msize; j++) {
            if ((*m_lgm)(i,j) == 1 && (*pdf)(i,j) == 0) {
                (*pdf)(i,j) = 1;
            }
        }
    }
}
void ObjectSearch::CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c) {
    //log("Direction: %f, FOV:%f, range: %f", direction, fov, range);
    float angle1 = direction + fov/2;
    float angle2 = direction - fov/2;
    b.x = cos(angle1)*range + a.x;
    c.x = cos(angle2)*range + a.x;
    b.y = sin(angle1)*range + a.y;
    c.y = sin(angle2)*range + a.y;
    //log("Got triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", a.x,a.y,b.x,b.y,c.x,c.y);
}
void ObjectSearch::FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c, int* rectangle) {
    int maxx,maxy,minx,miny;
    maxy = max(max(a.y,b.y),c.y);
    maxx = max(max(a.x,b.x),c.x);
    miny = min(min(a.y,b.y),c.y);
    minx = min(min(a.x,b.x),c.x);
    rectangle[0] = minx;
    rectangle[1] = maxx;
    rectangle[2] = miny;
    rectangle[3] = maxy;
    //log("Rectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",minx,miny,maxx, maxy);
}
bool ObjectSearch::isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c) { //the first one is the point the rest triangle

    if (isPointSameSide(p,a, b,c) && isPointSameSide(p,b, a,c)
            && isPointSameSide(p,c, a,b)) {
        return true;
    } else {
        return false;
    }


}
bool ObjectSearch::isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b) {
    XVector3D cp1 = (b - a).crossVector3D((p1-a));
    XVector3D cp2 = (b - a).crossVector3D((p2-a));
    if (cp1.dotVector3D(cp2) >= 0 ) {
        return true;
    } else {
        return false;
    }

}
void ObjectSearch::owtNavCommand(const cdl::WorkingMemoryChange & objID) {
    log("NavCommand Overwritten: %s", objID.address.id.c_str());
    boost::shared_ptr<CASTData<NavData::NavCommand> > oobj =
        getWorkingMemoryEntry<NavData::NavCommand>(objID.address);

    if (oobj != 0) {
        switch(oobj->getData()->comp) {
        case NavData::SUCCEEDED:
            m_status = NAVCOMMANDCOMPLETED;
            if (m_plan.plan.size() == 0) {
                m_command = PLAN;
            } else {
				
				Cure::Pose3D currpos = m_TOPP.getPose();
				double plantheta = m_plan.plan[whereinplan].getTheta();
				double anglediff = Cure::HelpFunctions::angleDiffRad(plantheta,currpos.getTheta());
				if ( fabs(anglediff) > M_PI_2 )
				{
					log("First turning the robot a little.");
					double turnangle;
					if (anglediff < 0) {
						turnangle = anglediff + M_PI_2 - 0.2;
					} else {
						turnangle = anglediff - M_PI_2 + 0.2;
					}
					NavData::NavCommandPtr cmd = new NavData::NavCommand;
					cmd->prio = NavData::URGENT;
					cmd->cmd = NavData::TURN;
					cmd->angle.resize(1);
					cmd->angle[0] = turnangle;
					cmd->tolerance.resize(1);
					cmd->tolerance[0] = 0.1;
					string id = newDataID();
					log("ID post: %s",id.c_str());
					addToWorkingMemory<NavData::NavCommand>(id, cmd);
					break;
				}
				
				log("command set to recognize");
                m_command = RECOGNIZE;
            }
            log("Task accomplished!");
            break;
        case NavData::FAILED:
            m_status = NAVCOMMANDCOMPLETED;
            m_command = EXECUTE;
            log("Task failed");
            break;
        case NavData::ABORTED:
            log("Task aborted :(");
            break;
        case NavData::INPROGRESS:
            log("Task in progress...");
            break;
        case NavData::PENDING:
            log("Task pending...");
            break;
        }
    }
}
void ObjectSearch::receiveScan2d(const Laser::Scan2d &castScan) {
	
    debug("Received Scan");
    Cure::LaserScan2d cureScan;
    CureHWUtils::convScan2dToCure(castScan, cureScan);
    if (m_TOPP.isTransformDefined()) {
        Cure::Pose3D scanPose;
        if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
        	m_Mutex.lock();
            m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
            m_Mutex.unlock();
            
            Cure::Pose3D lpW;
            m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
                                        0.5,
                                        0);
            lpW.add(scanPose, m_LaserPoseR);
            m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);
            
        }
    }

}
void ObjectSearch::ObjectDetected(const cast::cdl::WorkingMemoryChange &objID) {
	 shared_ptr<CASTData<VisionData::VisualObject> > oobj =
        getWorkingMemoryEntry<VisionData::VisualObject>(objID.address);
		
  VisionData::VisualObjectPtr obj = oobj->getData();
  if(obj->detectionConfidence >= 0.5){
	log("ok, detected '%s'", obj->label.c_str());

	NavData::ObjObsPtr obs = new NavData::ObjObs;
	obs->category = obj->label;
	obs->time = obj->time;

	static long objid = 0;
	obs->id.resize(1);
	obs->id[0] = objid++;

	if (m_CtrlPTU) {
	  ptz::PTZReading ptz = m_PTUServer->getPose();
	  obs->angles.resize(2);
	  obs->angles[0] = ptz.pose.pan;
	  obs->angles[1] = ptz.pose.tilt;
	}

	// FIXME Use the position of the object in the image as well
	// to get a better estimate of the direction to the object

	// FIXME Estimate the distance as well based on eg bounding box size
	
	addToWorkingMemory(newDataID(), obs);
  }
  else{
	log("nah, did not detect '%s'", obj->label.c_str());	  
  }
	if (obj->label == m_objectlist[m_objectlist.size()-1]->ObjID) {
		log("got the last object. recognition complete.");
		m_status = RECOGNITIONINCOMPLETE;  
	}
	
}
void ObjectSearch::Recognize(){
	ptz::PTZReading ptz;
	ptz.pose.pan = 0;
	if (m_CtrlPTU)
		ptz::PTZReading ptz = m_PTUServer->getPose();
		
	Cure::Pose3D currpos = m_TOPP.getPose();
	double plantheta = m_plan.plan[whereinplan].getTheta();
	double anglediff = Cure::HelpFunctions::angleDiffRad(plantheta,currpos.getTheta());
	
	log("plantheta : %f, currtheta, %f", plantheta, currpos.getTheta());
	log("anglediff is: %f", anglediff);
	log("ptz reading: %f", ptz.pose.pan);
	MovePanTilt(anglediff);
	m_status = 	RECOGNITIONINPROGRESS;
	PostRecognitionCommand();
	while(m_status != RECOGNITIONINCOMPLETE);
	log("now moving extras");
	int n = 1;
	while(anglediff + n*m_ptustep < M_PI/2){
		m_status = 	RECOGNITIONINPROGRESS;
		MovePanTilt(anglediff + n*m_ptustep);
		PostRecognitionCommand();
		while(m_status != RECOGNITIONINCOMPLETE);
		n++;
	}
	n= 1;
	while(anglediff - n*m_ptustep > -M_PI/2){
		m_status = 	RECOGNITIONINPROGRESS;
		MovePanTilt(anglediff - n*m_ptustep);
		PostRecognitionCommand();
		while(m_status != RECOGNITIONINCOMPLETE);
		n++;
}
}
void ObjectSearch::PostRecognitionCommand(){
	log("Posting Recog. Command now");
	 VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
	 vector<string> obj_labels;
	 for (unsigned int i= 0; i < m_objectlist.size(); i++)
	 {
		 obj_labels.push_back((m_objectlist[i]->ObjID));
	 }
	cmd->labels = obj_labels;
	addToWorkingMemory(newDataID(), "vision.sa", cmd);
	log("DetectionCommand added.");
}
void ObjectSearch::newRobotPose(const cast::cdl::WorkingMemoryChange &objID) {
    shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
        getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);

    m_SlamRobotPose.setX(oobj->getData()->x);
    m_SlamRobotPose.setY(oobj->getData()->y);
    m_SlamRobotPose.setTheta(oobj->getData()->theta);

    Cure::Pose3D cp = m_SlamRobotPose;
    m_TOPP.defineTransform(cp);
}

void ObjectSearch::receiveOdometry(const Robotbase::Odometry &castOdom) {
    Cure::Pose3D cureOdom;
    CureHWUtils::convOdomToCure(castOdom, cureOdom);

    debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
          cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
          cureOdom.getTime().getDouble());

    m_TOPP.addOdometry(cureOdom);
   Cure::Pose3D CurrPose = m_TOPP.getPose();
   m_Mutex.lock();
   m_LMap.moveRobot(CurrPose);
   m_Mutex.unlock();
}
void ObjectSearch::UpdateDisplays () {
    if (m_Displaylgm && displayOn) {
        Cure::Pose3D currentPose = m_TOPP.getPose();
        m_Displaylgm->updateDisplay(&currentPose,0,0,m_samplesize, m_samples,tpoints,ViewConePts,
                                    1,m_plan.plan,m_plan.indexarray);
    }
    if(m_Displaycoverage) {
        //log("Updating coverage map display");
        m_Displaycoverage->updateCoverageDisplay();
    }
    usleep(100);
}
