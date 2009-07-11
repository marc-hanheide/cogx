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

    m_CamRange = 1;
    it = _config.find("--cam-range");
    if (it != _config.end()) {
        m_CamRange = (atof(it->second.c_str()));
    }

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
        log("gridsize setto %i",m_gridsize);
    }
    it = _config.find("--cellsize");
    if (it != _config.end()) {
        m_cellsize = (atof(it->second.c_str()));
        log("cellsize setto %f",m_cellsize);
    }
    coveragetreshold = 50.0;
    it = _config.find("--coverage-treshold");
    if (it != _config.end()) {
        coveragetreshold = (double)(atof(it->second.c_str()));
        log("coverage treshold setto %f",coveragetreshold);
    }
    m_samplesize = 100;
    it = _config.find("--samplesize");
    if (it != _config.end()) {
        m_samplesize = (atof(it->second.c_str()));
        log("samplesize setto %d",m_samplesize);
    }
    m_samples = new int[2*m_samplesize];
    m_samplestheta = new double[m_samplesize];
    m_lgm = new Cure::XLocalGridMap<double>(m_gridsize/2, m_cellsize, 256, Cure::XLocalGridMap<float>::MAP1);
    m_Glrt  = new Cure::ObjGridLineRayTracer<double>(*m_lgm);
    //m_lmap = new LocalMap(*m_lgm);
    // X window setting
    coveragemap = new Cure::XLocalGridMap<unsigned int>(m_gridsize/2, m_cellsize, 0, Cure::XLocalGridMap<unsigned int>::MAP1);
    if (_config.find("--no-x-window") == _config.end()) {

        m_Displaylgm = new Cure::XDisplayLocalGridMap<double>(*m_lgm);
        log("Will use X window to show the exploration map");
    } else {
        m_Displaylgm = 0;
        log("Will NOT use X window to show the exploration map");
    }

    if (_config.find("--display-coverage") != _config.end()) {
        m_Displaycoverage = new Cure::XDisplayLocalGridMap<unsigned int>(*coveragemap);
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
    highestVCindex = -1;
    srand(time(NULL));
    cmp = NavData::SUCCEEDED;
    for (int i=0; i < m_gridsize; i++) {
        for (int j=0; j < m_gridsize; j++) {
            SuperObject->pdf->pdfgrid[i][j] = 0;
        }
    }

    coveragetotal = -1;
    covered = 0;
    runObjectSearch = true;
    tasktoggle = true;
    log("Configured ObjectSearch");
}

void ObjectSearch::runComponent() {
    log("ObjectSearch running ");

    setupPushScan2d(*this, -1);
    setupPushOdometry(*this, -1);

    while(runObjectSearch) {
    	UpdateGlobalPDF();
        UpdateCoverageMap();
        UpdateDisplays();
        Commander();
    }
}

void ObjectSearch::Commander () {
    debug("In commander");  
    if(!tasktoggle) {
        //log("Task being executed returning.");
        return;
    }
    ViewConePts.clear();
    GenViewPoints();
    tasktoggle = false;
    if (CoveragePercentage > coveragetreshold) {
        log("Coverage Treshold reached.Stopping.");
    }

    log("Sending command");
    log("Total coverage: %i, covered: %i, percentage: %f",coveragetotal, covered, CoveragePercentage);
    if (highestVCindex != -1) {
        log("GOTO X: %f Y:%f",candidatePoses[highestVCindex].getX(),candidatePoses[highestVCindex].getY());
        cmd = new NavData::NavCommand;
        cmd->prio = NavData::NORMAL;
        cmd->cmd = NavData::GOTOPOSITION;
        cmd->pose.resize(2);
        cmd->pose[0] = candidatePoses[highestVCindex].getX();
        cmd->pose[1] = candidatePoses[highestVCindex].getY();
        cmd->tolerance.resize(1);
        cmd->tolerance[0] = 0.1;
        string id = newDataID();
        log("ID post: %s",id.c_str());
        addToWorkingMemory<NavData::NavCommand>(id, cmd);
    }

}


void ObjectSearch::GenViewPoints() {
    //log("Generating %i random samples", m_samplesize);
    int randx,randy;
    int i=0;
    while (i < m_samplesize) {

        randx = rand();
        randy = rand();
        randx = (randx % (m_gridsize)) - m_gridsize/2;
        randy = (randy % (m_gridsize)) - m_gridsize/2;
        if ( (*m_lgm)(randx,randy) == 0) { //if random point is free space
            m_samples[2*i] = randx;
            m_samples[2*i+1] = randy;
            m_samplestheta[i] = (rand() * 3.14156592*2)/RAND_MAX;
            i++;
        }
    }

    //log("Calculating view cones for generated samples");
    Cure::Pose3D candidatePose;
    XVector3D a,b,c,p;
    XVector3D m_a,m_b,m_c;
    int h,k;
    int* rectangle = new int[4];
    for (int y=0; y < m_samplesize; y++) { //calc. view cone for each sample
        tpoints.clear();
        m_lgm->index2WorldCoords(m_samples[y*2],m_samples[2*y+1],a.x,a.y);
        //currentPose = m_TOPP.getPose();
        CalculateViewCone(a,m_samplestheta[y],m_CamRange,1,b,c);

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
                if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
                    //log("in here with size: %i.", (int)tpoints.size());
                    (*m_lgm)(x,y) == 0;
                    tpoints.push_back(x);
                    tpoints.push_back(y);
                }
            }
        }
        vector<int>::iterator theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.y);
        theIterator = tpoints.begin();
        tpoints.insert( theIterator, 1, m_a.x);
        ViewConePts.push_back(tpoints);
        candidatePose.setTheta(m_samplestheta[y]);
        candidatePose.setX(a.x);
        candidatePose.setY(a.y);
        //log("CurrentPose.Theta : %f", candidatePose.getTheta());
        candidatePoses.push_back(candidatePose);
    }
    //log("View Cones calculated.");
    SelectBestView();
    displayOn = true;
}

void ObjectSearch::SelectBestView() {
    //log("Selecting best view cone out of %i random samples.",m_samplesize);
    std::vector<int> PDFsum;
    int x,y;
    int singlesum;
    int lsize = m_gridsize/2;
    int highest = -999;
    for (unsigned int i = 0; i < ViewConePts.size(); i++) {
        singlesum = 0;
        for (unsigned int j = 0; j < ViewConePts[i].size()/2; j++) {
            x = ViewConePts[i][2*j];
            y =ViewConePts[i][2*j + 1];
            //log("If empty: %i,%i",x,y);
            if((*coveragemap)(x,y) != 2) {
                singlesum = singlesum + SuperObject->pdf->pdfgrid[x+lsize][y+lsize];
            }
        }

        PDFsum.push_back(singlesum);
        if (singlesum > highest) {
            highest = singlesum;
            highestVCindex = i;
        }
        //log("PDF Sum for view cone %i is: %i",i, singlesum);
    }
    
    ModifyCoverageMap(ViewConePts[highestVCindex]);
}

double ObjectSearch::ModifyCoverageMap(std::vector<int> tpoints, bool hypothetical) {
    //log("ModifyCoverageMap");
    double percentage;
    int h = 0;
    if ( highestVCindex != -1) {
        int x,y;
        for (int j = 0; j < tpoints.size()/2; j++) {
            x = tpoints[2*j];
            y = tpoints[2*j + 1];
            if ((*m_lgm)(x,y) == 1 && (*coveragemap)(x,y) != 2) {
                covered++;
                h++;
                if (hypothetical == false)
                    (*coveragemap)(x,y) = 2; //obstacle covered
            }
        }
    }
    percentage = (double)100*covered/coveragetotal;
    if(hypothetical) {
        covered = covered - h;
        return percentage;
    } else {
        CoveragePercentage = (double)100*covered/coveragetotal;
        return CoveragePercentage;
    }
    //log("ModifyCoverageMap2");
}
void ObjectSearch::UpdateCoverageMap() {
    int msize = m_lgm->getSize();

    for (int i=-msize; i < msize; i++) {
        for (int j=-msize; j < msize; j++) {
            if ((*m_lgm)(i,j) == 1 && (*coveragemap)(i,j) != 1
                    && (*coveragemap)(i,j) != 2) {
                coveragetotal++;
                (*coveragemap)(i,j) = 1; //obstacle not yet covered
            }
        }
    }
    CoveragePercentage = (double)100*covered/coveragetotal;
}
void ObjectSearch::UpdateGlobalPDF(int index) {
    //log("InitializeGlobalPDF");
    LoadPriors();
    if (index == -1) {
        for (int  m = 0 ; m < m_objectlist.size() ; m++) {
            for (int i=0; i < m_gridsize; i++) {
                for (int j=0; j < m_gridsize; j++) {
                    SuperObject->pdf->pdfgrid[i][j] =
                        SuperObject->pdf->pdfgrid[i][j] + m_objectlist[m]->pdf->pdfgrid[i][j];
                }
            }
        }
    } else {
        for (int i=0; i < m_gridsize; i++) {
            for (int j=0; j < m_gridsize; j++) {
                SuperObject->pdf->pdfgrid[i][j] =
                    SuperObject->pdf->pdfgrid[i][j] + m_objectlist[index]->pdf->pdfgrid[i][j];
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
            if ((*m_lgm)(i-lsize,j-lsize) == 1 ) { // an obstacle
                //log("AN OBSTABLE!!!");
                foo->pdf->pdfgrid[i][j] = 100;
            } else {
                foo->pdf->pdfgrid[i][j] = 0;
            }
        }
    }
}
void ObjectSearch::LoadPriors() {
    for (int i =0; i < m_objectlist.size(); i++) {
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
        	tasktoggle= true;	
            log("Task accomplished!");
            break;
        case NavData::FAILED:
        	tasktoggle= true;
            log("Task failed");
            break;
        case NavData::ABORTED:
        	tasktoggle= true;
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
        }
        firstscanreceived = true;
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
}
void ObjectSearch::UpdateDisplays () {
    if (m_Displaylgm && displayOn && firstscanreceived) {
        Cure::Pose3D currentPose = m_TOPP.getPose();
        m_Displaylgm->updateDisplay(&currentPose,0,0,m_samplesize, m_samples,tpoints,ViewConePts,highestVCindex);
    }
    if(m_Displaycoverage) {
        //log("Updating coverage map display");
        m_Displaycoverage->updateCoverageDisplay();
    }
    usleep(100);
}