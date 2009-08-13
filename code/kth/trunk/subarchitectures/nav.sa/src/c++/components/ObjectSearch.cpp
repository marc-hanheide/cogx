#include "ObjectSearch.hpp"
#include <NavData.hpp>
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <Navigation/GridContainer.hh>
#include "ObjPdf.hpp"
#include <NavData.hpp>
#include "XVector3D.h"
using namespace cast;
using namespace std;
using namespace boost;
using namespace Cure;
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
        log("configure(...) Failed to open with \"%s\"\n",
            configfile.c_str());
        std::abort();
    }

    //Exploration range
    m_MaxExplorationRange = 1;
    it = _config.find("--explore-range");
    if (it != _config.end()) {
        m_MaxExplorationRange = (atof(it->second.c_str()));
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
            obj->pdf = new ObjPdf::ObjPdf(m_gridsize);
            m_objectlist.push_back(obj);
        }
    }
    log("Loaded objects.");



    SuperObject = new Object();
    SuperObject->pdf = new ObjPdf(m_gridsize);
    firstscanreceived = false;

    srand(time(NULL));
    cmp = NavData::SUCCEEDED;
    for (int i=0; i < m_gridsize; i++) {
        for (int j=0; j < m_gridsize; j++) {
            SuperObject->pdf->pdfgrid[i][j] = 0;
        }
    }

    m_coveragetotal = -1;
    m_covered = 0;
    runObjectSearch = true;
    m_status = STOPPED;
    whereinplan = 0;
    log("Configured ObjectSearch");
}

void ObjectSearch::runComponent() {
    log("ObjectSearch running ");

    setupPushScan2d(*this, -1);
    setupPushOdometry(*this, -1);

    //clock_t start_time,elapsed;
    //double elapsed_time;
    m_command = PLAN;
    while(runObjectSearch) {
        InterpretCommand ();
        UpdateDisplays();
        sleep(1);
        //start_time = clock();
        //elapsed = clock()-start_time;
        //elapsed_time = elapsed / ((double) CLOCKS_PER_SEC)*1000;
        //log("Time elapsed: %f",elapsed_time);

    }
}

void ObjectSearch::Plan () {
    GenViewPoints();
    GeneratePlan(m_covthresh, IntegrateProb());
    log("Plan generated %i view points with %f coverage",m_plan.plan.size(),m_plan.totalcoverage);
    if (true) {
        m_command = PLAN;
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
        Plan();
        m_status = PLANNING;
        break;
    case EXECUTE:
        log("Command: EXECUTE");
        m_command = IDLE;
        ExecutePlan();
        m_status = EXECUTINGPLAN;
        break;
    case IDLE:
//        log("Command: IDLE");
        break;
    default:
        log("Command: Default.");
        break;
    }
}
void ObjectSearch::ExecuteNextInPlan () {
    log("Plan size %i, where in plan: %i.",m_plan.plan.size(),whereinplan);
    if (whereinplan > m_plan.plan.size()-1 ) {
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
        whereinplan++;
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
    cmd->tolerance.resize(1);
    cmd->tolerance[0] = 0.1;
    string id = newDataID();
    log("ID post: %s",id.c_str());
    addToWorkingMemory<NavData::NavCommand>(id, cmd);
}
void ObjectSearch::ExecutePlan () {
    ExecuteNextInPlan();
}


void ObjectSearch::GenViewPoints() {
    //log("Generating %i random samples", m_samplesize);
    ViewConePts.clear();
    int randx,randy;
    double xW,yW;
    int i=0;
    std::vector<double> angles;
    for (double rad= 0 ; rad < M_PI*2 ; rad = rad + M_PI/12)
        angles.push_back(rad);

    while (i < m_samplesize) {

        randx = rand();
        randy = rand();
        randx = (randx % (m_gridsize)) - m_gridsize/2;
        randy = (randy % (m_gridsize)) - m_gridsize/2;
        m_lgm->index2WorldCoords(randx,randy,xW,yW);
        if ((*m_lgm)(randx,randy) == 0) {
            if (m_lgm->isCircleObstacleFree(xW,yW, m_awayfromobstacles) &&
            	m_LMap.goalReachable(xW,yW, 1,1)) { //if random point is free space
                m_samples[2*i] = randx;
                m_samples[2*i+1] = randy;
                int the = (int)(rand() % angles.size());
                m_samplestheta[i] = angles[the];
                i++;
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
       /* m_lgm->index2WorldCoords(m_samples[y*2],m_samples[2*y+1],a.x,a.y);
        //currentPose = m_TOPP.getPose();
        CalculateViewCone(a,m_samplestheta[y],m_CamRange,M_PI/3,b,c);

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
                if ((*coveragemap)(x,y) == 1) {
                    if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
                        //log("in here with size: %i.", (int)tpoints.size());
                        tpoints.push_back(x);
                        tpoints.push_back(y);
                    }
                }
            }
        }
        vector<int>::iterator theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.y);
        theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.x);*/
        
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
std::vector<int> ObjectSearch::GetInsideViewCone(XVector3D &a, bool addall) {
	tpoints.clear();
	XVector3D b,c,p;
    XVector3D m_a,m_b,m_c;
    int* rectangle = new int[4];
    int h,k;
        //currentPose = m_TOPP.getPose();
        CalculateViewCone(a,a.theta,m_CamRange,M_PI/3,b,c);

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
                        //log("in here with size: %i.", (int)tpoints.size());
                        tpoints.push_back(x);
                        tpoints.push_back(y);
                    }
                }
                else {
                if ((*coveragemap)(x,y) == 1) {
                    if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
                        //log("in here with size: %i.", (int)tpoints.size());
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
std::vector<double> ObjectSearch::IntegrateProb() {
    //log("Selecting best view cone out of %i random samples.",m_samplesize);
    std::vector<double> PDFsum;
    int x,y;
    int lsize = m_gridsize/2;
    double singlesum;
    for (unsigned int i = 0; i < ViewConePts.size(); i++) {
        singlesum = 0;
        for (unsigned int j = 0; j < ViewConePts[i].size()/2; j++) {
            x = ViewConePts[i][2*j];
            y =ViewConePts[i][2*j + 1];
            if((*coveragemap)(x,y) == 1) {
                //println("obs: %f", SuperObject->pdf->pdfgrid[x+lsize][y+lsize]);
                singlesum = singlesum + SuperObject->pdf->pdfgrid[x+lsize][y+lsize];
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
bool ObjectSearch::GeneratePlan(double covpercent,std::vector<double> PDFsum) {
    SearchPlan searchplan;
    log("GeneratePln");
    //runObjectSearch = false
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
                log("found at %ith view point",j);
                break;
            }
        }

        copy.erase(copy.begin()); //chop off the first element so next time in loop copy[0] is the next element
        if (CalculateCoverage(ViewConePts[j],covered, m_vpthreshold,fcm)) { // if worthy of adding to the plan
        	//just to display the selected viewcone
        	XVector3D a;
        	m_lgm->index2WorldCoords(m_samples[j*2],m_samples[2*j+1],a.x,a.y);
        	a.theta =  m_samplestheta[j];
        	ViewConePts[j] = GetInsideViewCone(a, true); 
            searchplan.plan.push_back(candidatePoses[j]);
            searchplan.indexarray.push_back(j);
            searchplan.totalcoverage = (double)100*(covered)/m_coveragetotal;
            println("Total Coverage %f", searchplan.totalcoverage);
            println("Added to plan");
            if (searchplan.totalcoverage >= m_covthresh) {
                println("Found plan covers %f of the area", searchplan.totalcoverage);
                m_plan =  searchplan;
                return true;
            }
        } else {
            //println("not worthy of adding to the plan.");
        }


    }
    if (searchplan.totalcoverage < m_covthresh) {
        println("Plan does not cover %f only %f", covpercent, searchplan.totalcoverage);
        m_plan =  searchplan;
        return false;
        //m_plan=  searchplan;
    }
    else{
    	return true;
    }
    
}

bool ObjectSearch::CalculateCoverage(std::vector<int> tpoints, int &covered, double treshold,Cure::LocalGridMap<unsigned int> &fcm) {
    double oldcov = (double)100*covered/m_coveragetotal;
    int h = 0;
    int x,y;
    std::vector<int> tmp;
    for (unsigned int j = 0; j < tpoints.size()/2; j++) {
        x = tpoints[2*j];
        y = tpoints[2*j + 1];
        if ((*m_lgm)(x,y) == 1 && (fcm)(x,y) != 2) {
            h++;
            fcm(x,y) = 2;
            tmp.push_back(x);
            tmp.push_back(y);
        }
    }
    double cov = (double)100*(covered+h)/m_coveragetotal;
    debug("Difference in cov: %f", (cov-oldcov));
    if ((cov - oldcov) > treshold) {
        covered = covered + h;
        return true;
    } else { //roll back
        for (unsigned int j = 0; j < tmp.size()/2; j++) {
            x = tmp[2*j];
            y = tmp[2*j + 1];
            fcm(x,y) = 1;
        }
    }
    return false;
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
void ObjectSearch::UpdateCoverageMap() {
    int msize = m_lgm->getSize();

    for (int i=-msize; i < msize; i++) {
        for (int j=-msize; j < msize; j++) {
            if ((*m_lgm)(i,j) == 1 && (*coveragemap)(i,j) != 1
                    && (*coveragemap)(i,j) != 2) {
                m_coveragetotal++;
                (*coveragemap)(i,j) = 1; //obstacle not yet covered
            }
        }
    }
    CoveragePercentage = (double)100*m_covered/m_coveragetotal;
}
void ObjectSearch::UpdateGlobalPDF(int index) {
    //log("InitializeGlobalPDF");
    int lsize = m_gridsize/2;
    LoadPriors();
    if (index == -1) {
        for (unsigned int  m = 0 ; m < m_objectlist.size() ; m++) {
            for (int i=0; i < m_gridsize; i++) {
                for (int j=0; j < m_gridsize; j++) {
                    if ((*coveragemap)(i-lsize,j-lsize) == 0) {
                        SuperObject->pdf->pdfgrid[i][j] =
                            SuperObject->pdf->pdfgrid[i][j] + m_objectlist[m]->pdf->pdfgrid[i][j];
                    }
                }
            }
        }
    }
}

void ObjectSearch::ModifyGlobalPDF() {}

void ObjectSearch::InitializeObjPDF(Object* foo) {
    int lsize = m_gridsize/2;
    for (int i=0; i < foo->pdf->m_gridsize; i++) {
        for (int j=0; j < foo->pdf->m_gridsize; j++) {
            //log("at: %i,%i",i-lsize,j-lsize);
            if ((*m_lgm)(i-lsize,j-lsize) == 1 && (*coveragemap)(i-lsize,j-lsize) == 0 ) { // an obstacle
                //log("AN OBSTACLE!!!");
                foo->pdf->pdfgrid[i][j] = 1;
            } else {
                foo->pdf->pdfgrid[i][j] = 0;
            }
        }
    }
}
void ObjectSearch::LoadPriors() {
    for (unsigned int i =0; i < m_objectlist.size(); i++) {
        //log("Loading prior for %s",m_objectlist[i]->ObjID.c_str());
        InitializeObjPDF(m_objectlist[i]);
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
                m_command = EXECUTE;
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
            Cure::Pose3D lpW;
            m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
                                        0.5,
                                        '0');
            lpW.add(scanPose, m_LaserPoseR);
            m_Glrt->addScan(cureScan, lpW, m_MaxExplorationRange);
            m_Mutex.lock();
            m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
            m_Mutex.unlock();
        }
        firstscanreceived = true;
        UpdateGlobalPDF();
        UpdateCoverageMap();
    }

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
    if (m_Displaylgm && displayOn && firstscanreceived) {
        Cure::Pose3D currentPose = m_TOPP.getPose();
        m_Displaylgm->updateDisplay(&currentPose,0,0,m_samplesize, m_samples,tpoints,ViewConePts,
                                    -1,m_plan.plan,m_plan.indexarray);
    }
    if(m_Displaycoverage) {
        //log("Updating coverage map display");
        m_Displaycoverage->updateCoverageDisplay();
    }
    usleep(100);
}
