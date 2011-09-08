/**
 * @author Kai ZHOU
 * @date June 2009
 */

#ifdef __APPLE__
#include <GL/glut.h> //nah: glut is installed via MacPorts and thus preferred
#else
#include <GL/freeglut.h>
#endif


#include <cogxmath.h>
#include "PlanePopOut.h"
#include <stack>
#include <vector>
#include <VideoUtils.h>
#include <math.h>
#include <algorithm>
#include <time.h>
#include "StereoCamera.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "../VisionUtils.h"


#ifdef __APPLE__

long long gethrtime(void)
{
    timeval tv;
    int ret;
    long long v;
    ret = gettimeofday(&tv, NULL);
    if(ret!=0) return 0;  
    v=1000000000LL; /* seconds->nanonseconds */
    v*=tv.tv_sec;
    v+=(tv.tv_usec*1000); /* microseconds->nanonseconds */
    return v;
}

#else

long long gethrtime(void)
{
    struct timespec sp;
    int ret;
    long long v;
#ifdef CLOCK_MONOTONIC_HR
    ret=clock_gettime(CLOCK_MONOTONIC_HR, &sp);
#else
    ret=clock_gettime(CLOCK_MONOTONIC, &sp);
#endif
    if(ret!=0) return 0;
    v=1000000000LL; /* seconds->nanonseconds */
    v*=sp.tv_sec;
    v+=sp.tv_nsec;
    return v;
}

#endif


#define SendDensePoints  1 	//0 send sparse points ,1 send dense points (recollect them after the segmentation)
#define Treshold_Comp2SOI	0.75	//the similarity of 2 SOIs higher than this will make the system treat these 2 SOI as the sam one

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
    cast::CASTComponentPtr newComponent()
    {
	return new cast::PlanePopOut();
    }
}

namespace cast
{
using namespace std;
using namespace PointCloud;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;
//using namespace navsa;
using namespace cdl;



void PlanePopOut::configure(const map<string,string> & _config)
{
//     log("start the configuration!!!!!!!");
    // first let the base classes configure themselves
    configureServerCommunication(_config);

    map<string,string>::const_iterator it;

    useGlobalPoints = true;
    doDisplay = false;
    bWithKinect = false;
    AgonalTime = 10;
    StableTime = 2;
    Shrink_SOI = 0.9;
    Upper_BG = 1.5;
    Lower_BG = 1.1;
    mConvexHullDensity = 0.0;
    pre_mCenterOfHull.x = pre_mCenterOfHull.y = pre_mCenterOfHull.z = 0.0;
    pre_mConvexHullRadius = 0.0;
    pre_id = "";
    bSaveImage = true;
    fx = 525;
    fy = 525;
    cx = 320;
    cy = 240;
    if((it = _config.find("--globalPoints")) != _config.end())
    {
	istringstream str(it->second);
	str >> boolalpha >> useGlobalPoints;
    }
    if((it = _config.find("--display")) != _config.end())
    {
	doDisplay = true;
    }
    if((it = _config.find("--agonalTime")) != _config.end())
    {
	istringstream str(it->second);
	str >> AgonalTime;
    }
    if((it = _config.find("--stableTime")) != _config.end())
    {
	istringstream str(it->second);
	str >> StableTime;
    }
    if((it = _config.find("--useKinect")) != _config.end())
    {
	bWithKinect = true;
    }
    bWriteSoisToWm = true;
    if((it = _config.find("--generate-sois")) != _config.end())
    {
	bWriteSoisToWm = ! (it->second == "0" || it->second == "false" || it->second == "off");
    }
    println("%s write SOIs to WM", bWriteSoisToWm ? "WILL" : "WILL NOT"); 

    // startup window size
    int winWidth = 640, winHeight = 480;
    cv::Mat intrinsic = (cv::Mat_<double>(3,3) << winWidth,0,winWidth/2, 0,winWidth,winHeight/2, 0,0,1);
    cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
    cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
    cv::Vec3d rotCenter(0,0,0.4);

    if(doDisplay)
    {
	// Initialize 3D render engine 
	tgRenderer = new TGThread::TomGineThread(1280, 1024);
	tgRenderer->SetParameter(intrinsic);
	tgRenderer->SetCamera(R, t, rotCenter);
	tgRenderer->SetCoordinateFrame(0.5);
    }

    println("use global points: %d", (int)useGlobalPoints);

#ifdef FEAT_VISUALIZATION
    m_display.configureDisplayClient(_config);
#endif
}

#ifdef FEAT_VISUALIZATION
// display objects
// TODO: with multiple PPOs the names have to include the componentID.
#define ID_OBJECT_3D       "PlanePopout.3D"
#define ID_PART_3D_POINTS  "3D points"
#define ID_PART_3D_PLANE   "Plane grid"
#define ID_PART_3D_SOI     "SOI:"
#define ID_PART_3D_OVERLAY "Overlay"
#define ID_OBJECT_IMAGE    "PlanePopout.Image"

// display controls
#define IDC_POPOUT_SOIS "popout.show.sois"
#define IDC_POPOUT_IMAGE "popout.show.image"
#define IDC_POPOUT_POINTS "popout.show.points"
#define IDC_POPOUT_PLANEGRID "popout.show.planegrid"
#define IDC_POPOUT_LABEL_COLOR "popout.show.labelcolor"
#endif

void PlanePopOut::start()
{
    startPCCServerCommunication(*this);
//     log("start the component!!!!!!!!!");
#ifdef FEAT_VISUALIZATION

    m_bSendPoints = false;
    m_bSendPlaneGrid = true;
    m_bSendImage = true;
    m_bSendSois = true;
    m_bColorByLabel = true;
    m_display.connectIceClient(*this);
    m_display.setClientData(this);
    m_display.installEventReceiver();

    Visualization::ActionInfo act;
    act.id = IDC_POPOUT_LABEL_COLOR;
    act.label = "Color by label";
    act.iconLabel = "Color";
    act.iconSvg = "text:Co";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = IDC_POPOUT_POINTS;
    act.label = "Toggle Update 3D Points";
    act.iconLabel = "3D Points";
    act.iconSvg = "text:Pts";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = IDC_POPOUT_SOIS;
    act.label = "Toggle Update SOIs";
    act.iconLabel = "SOIs";
    act.iconSvg = "text:Soi";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = IDC_POPOUT_PLANEGRID;
    act.label = "Toggle Update Convex Hull of Principal Plane";
    act.iconLabel = "Plane Hull";
    act.iconSvg = "text:Hul";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = IDC_POPOUT_IMAGE;
    act.label = "Toggle Update Image";
    act.iconLabel = "Image";
    act.iconSvg = "text:Img";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_IMAGE, act);

    // Object displays (m_bXX) are set to off: we need to create dummy display objects
    // on the server so that we can activate displays through GUI
    ostringstream ss;
    ss <<  "function render()\nend\n"
	<< "setCamera('ppo.points.top', 0, 0, -0.5, 0, 0, 1, 0, -1, 0)\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_POINTS, ss.str());
    m_tmSendPoints.restart();

    //Video::Image image;
    //m_display.setImage(ID_OBJECT_IMAGE, image);

    ss.str("");
    ss  << "sois = {}\n"
	<< "showSois = " << (m_bSendSois ? "true" : "false") << "\n";

    ss  << "function render()\n"
	<<  "if not showSois then return end\n"
	<<  "glColor(0.0, 0.0, 1.0, 0.2)\n"
	<<  "for k,v in pairs(sois) do\n"
	<<   "glPushMatrix()\n"
	<<   "glTranslate(v.x, v.y, v.z)\n"
	<<   "StdModel:cylinder(v.sx, v.sy, v.sz, 12)\n"
	<<   "glPopMatrix()\n"
	<<  "end\n"
	<< "end\n";

    ss  << "function setSoi(id, x, y, z, sx, sy, sz)\n"
	<<  "sois[id] = {x=x, y=y, z=z, sx=sx, sy=sy, sz=sz}\n"
	<< "end\n";

    ss  << "function removeSoi(id)\n"
	<<  "sois[id] = nil\n"
	<< "end\n";

    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI, ss.str());
#endif
//     log("In start, finish the initialization of visualization");
    // @author: mmarko
    // we want to receive GetStableSoisCommand-s
    addChangeFilter(createLocalTypeFilter<VisionData::GetStableSoisCommand>(cdl::ADD),
	    new MemberFunctionChangeReceiver<PlanePopOut>(this,
		&PlanePopOut::onAdd_GetStableSoisCommand));
}

#ifdef FEAT_VISUALIZATION
void PlanePopOut::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
    if (!pPopout) return;
    pPopout->println("Got event: %s", event.sourceId.c_str());
    if (event.sourceId == IDC_POPOUT_POINTS) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendPoints = false;
	else pPopout->m_bSendPoints = true;
	if (!pPopout->m_bSendPoints)
	    setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_POINTS, "function render()\nend\n");
    }
    else if (event.sourceId == IDC_POPOUT_PLANEGRID) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendPlaneGrid = false;
	else pPopout->m_bSendPlaneGrid = true;
	if (!pPopout->m_bSendPlaneGrid)
	    setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_PLANE, "function render()\nend\n");
    }
    else if (event.sourceId == IDC_POPOUT_LABEL_COLOR) {
	if (event.data == "0" || event.data=="") pPopout->m_bColorByLabel = false;
	else pPopout->m_bColorByLabel = true;
    }
    else if (event.sourceId == IDC_POPOUT_IMAGE) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendImage = false;
	else pPopout->m_bSendImage = true;
    }
    else if (event.sourceId == IDC_POPOUT_SOIS) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendSois = false;
	else pPopout->m_bSendSois = true;
	pPopout->m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI, 
		pPopout->m_bSendSois ? "showSois=true" : "showSois=false");
    }
}

std::string PlanePopOut::CDisplayClient::getControlState(const std::string& ctrlId)
{
    if (!pPopout) return "";
    pPopout->println("Get control state: %s", ctrlId.c_str());
    if (ctrlId == IDC_POPOUT_POINTS) {
	if (pPopout->m_bSendPoints) return "2";
	else return "0";
    }
    if (ctrlId == IDC_POPOUT_PLANEGRID) {
	if (pPopout->m_bSendPlaneGrid) return "2";
	else return "0";
    }
    if (ctrlId == IDC_POPOUT_LABEL_COLOR) {
	if (pPopout->m_bColorByLabel) return "2";
	else return "0";
    }
    if (ctrlId == IDC_POPOUT_IMAGE) {
	if (pPopout->m_bSendImage) return "2";
	else return "0";
    }
    if (ctrlId == IDC_POPOUT_SOIS) {
	if (pPopout->m_bSendSois) return "2";
	else return "0";
    }
    return "";
}

void PlanePopOut::SendImage()
{
    CMilliTimer tm(true);

    for (unsigned int i=0 ; i<points.size() ; i++)
    {
	int m_label = points_label.at(i);
	PointCloud::SurfacePoint& pt = points.at(i);
	switch (m_label)
	{
	    case 0: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(255,0,0)); break;
	    case 1: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(0,255,0)); break;
	    case 2: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(0,0,255)); break;
	    case 3: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(0,255,255)); break;
	    case 4: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(128,128,0)); break;
	    case 5: cvCircle(ROIMaskImg, ProjectPointOnImage(pt.p), 2, CV_RGB(255,255,255)); break;
	}
    }
//     CvFont a;
//     cvInitFont( &a, CV_FONT_HERSHEY_PLAIN, 1, 1, 0 , 1 );
//     for (unsigned int i=0 ; i<vSOIonImg.size() ; i++)
//     {
// 	CvPoint p;
// 	CvRect& rsoi = vSOIonImg.at(i);
// 	p.x = (int)(rsoi.x+0.3 * rsoi.width);
// 	p.y = (int)(rsoi.y+0.5 * rsoi.height);
// 	cvPutText(ROIMaskImg, vSOIid.at(i).c_str(), p, &a,CV_RGB(255,255,255));
//     }

    long long t1 = tm.elapsed();
    m_display.setImage(ID_OBJECT_IMAGE, ROIMaskImg);
    long long t2 = tm.elapsed();
//     if (bSaveImage)
// 	cvSaveImage("/tmp/planes_image.jpg", ROIMaskImg);
    long size = ROIMaskImg->imageSize;
    long long t3 = tm.elapsed();

    if (1) {
	ostringstream str;
	str << "<h3>Plane popout - SendImage</h3>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Size: " << size << " bytes.<br>";
	str << "Sent: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	if (bSaveImage)
	    str << "Saved: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	m_display.setHtml("LOG", "log.PPO.SendImage", str.str());
    }
}

void PlanePopOut::SendPoints(const PointCloud::SurfacePointSeq& points, std::vector<int> &labels,
	bool bColorByLabels, CMilliTimer& tmSendPoints)
{
    if (tmSendPoints.elapsed() < 500) // 2Hz
	return;
    tmSendPoints.restart();

    if (points.size() != labels.size()) {
	error(" *** MISMATCH IN SIZE of points and labels (in SendPoints) ***");
	return;
    }

    std::ostringstream str;
    str.unsetf(ios::floatfield); // unset floatfield
    str.precision(5); // set the _maximum_ precision

    str << "function render()\nglPointSize(2)\nglBegin(GL_POINTS)\n";
    str << "v=glVertex\nc=glColor\n";
    int plab = -9999;
    int colors = 1;
    cogx::Math::ColorRGB coPrev;
    coPrev.r = coPrev.g = coPrev.b = 0;
    str << "glColor(0,0,0)\n";
    for(size_t i = 0; i < points.size(); i++)
    {
	const PointCloud::SurfacePoint &p = points[i];
	if ( isinf(p.p.x) || isinf(p.p.y) || isinf(p.p.z) )
	    continue;

	if (!bColorByLabels) {
#define CO3(bc) int(1000.0*bc/255)/1000.0
	    if (coPrev != p.c) {
		colors++;
		str << "c(" << CO3(p.c.r) << "," << CO3(p.c.g) << "," << CO3(p.c.b) << ")\n";
		coPrev = p.c;
	    }
#undef CO3
	}
	else {
	    int lab = labels.at(i);
	    if (lab == -1) 
	      continue; // skip this point

	    if (plab != lab) {
		colors++;
		plab = lab;
		switch (lab) {
		    case 0: str << "c(0.0,0.0,1.0)\n"; break;
		    case 1: str << "c(0.0,1.0,0.0)\n"; break;
		    case 2: str << "c(1.0,0.0,.0)\n"; break;
		    case 3: str << "c(0.0,0.5,0.5)\n"; break;
		    case 4: str << "c(0.5,0.5,0.0)\n"; break;
// 		    case -1: str << "c(0.5,0.0,0.5)\n"; break;
 		    default:  str << "c(1.0,1.0,0.0)\n"; break;
		}
	    }
	}
	str << "v(" << p.p.x << "," << p.p.y << "," << p.p.z << ")\n";
    }
    str << "glEnd()\nend\n";
    long long t1 = tmSendPoints.elapsed();
    string S = str.str();
    long long t2 = tmSendPoints.elapsed();
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_POINTS, S);
    long long t3 = tmSendPoints.elapsed();
    if (1) {
	str.str("");
	str.clear();
	str << "<h3>Plane popout - SendPoints</h3>";
	str << "Labels: " << (bColorByLabels ? "ON" : "OFF") << "<br>";
	str << "Points: " << points.size() << "<br>";
	str << "Colors: " << colors << " color changes<br>";
	str << "Strlen: " << S.length() << "<br>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Converted: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	str << "Sent: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	m_display.setHtml("LOG", "log.PPO.SendPoints", str.str());
    }
}

void PlanePopOut::SendPlaneGrid()
{
    static CMilliTimer tmSendPlaneGrid(true);
    if (tmSendPlaneGrid.elapsed() < 100) // 10Hz
	return;
    tmSendPlaneGrid.restart();

    CMilliTimer tm(true);
    std::ostringstream str;
    str << "function render()\n";

    if (mConvexHullPoints.size() > 2)
    {
	str << "glBegin(GL_LINE_LOOP)\n";
	str << "glPointSize(2.0)\n";
	str << "glColor(1.0,1.0,1.0)\n";
	str << "v=glVertex\n";
	for(int i = 0; i < mConvexHullPoints.size(); i++) {
	    Vector3& p = mConvexHullPoints.at(i);
	    str << "v(" << p.x << "," << p.y << "," << p.z << ")\n";
	}
	Vector3& p = mConvexHullPoints.at(0);
	str << "v(" << p.x << "," << p.y << "," << p.z << ")\n";
	str << "glEnd()\n";
    }
    str << "end\n";

    long long t1 = tm.elapsed_micros();
    string S = str.str();
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_PLANE, S);
    long long t2 = tm.elapsed_micros();

    if (1) {
	str.str("");
	str.clear();
	str << "<h3>Plane popout - SendPlaneGrid</h3>";
	str << "Strlen: " << S.length() << "<br>";
	str << "Generated: " << t1 << "&mu;s from start (in " << t1 << "&mu;s).<br>";
	str << "Sent: " << t2 << "&mu;s from start (in " << (t2-t1) << "&mu;s).<br>";
	m_display.setHtml("LOG", "log.PPO.SendPlaneGrid", str.str());
    }
}

void PlanePopOut::SendOverlays()
{
    std::ostringstream str;
    str << "function render()\n";
    str << "v=glVertex\nc=glColor\n";
    str << "glBegin(GL_LINES)\n";
    str << "c(1.0,0.0,0.0)\n";
    str << "v(0., 0., 0.)\n";
    str << "v(0.1, 0., 0.)\n";
    str << "c(0.0,1.0,0.0)\n";
    str << "v(0., 0., 0.)\n";
    str << "v(0., 0.1, 0.)\n";
    str << "c(0.0,0.0,1.0)\n";
    str << "v(0., 0., 0.)\n";
    str << "v(0., 0., 0.1)\n";
    str << "glEnd()\n";
    str << "showLabel(0.1, 0, 0, 'x', 12)\n";
    str << "showLabel(0, 0.1, 0, 'y', 12)\n";
    str << "showLabel(0, 0, 0.1, 'z', 12)\n";
    str << "end\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_OVERLAY, str.str());
}

void PlanePopOut::SendSoi(PlanePopOut::ObjPara& soiobj)
{
    ostringstream ss;
    ss  << "setSoi('" << soiobj.id << "',"
	<< soiobj.c.x << ","
	<< soiobj.c.y << ","
	<< soiobj.c.z << ","
	<< soiobj.s.x << ","
	<< soiobj.s.y << ","
	<< soiobj.s.z << ")\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI, ss.str());
}

void PlanePopOut::SendRemoveSoi(PlanePopOut::ObjPara& soiobj)
{
    ostringstream ss;
    ss  << "removeSoi('" << soiobj.id << "')";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI, ss.str());
}

#endif

void PlanePopOut::runComponent()
{
    sleepComponent(3000);
//     log("Component PlanePopOut is running now");
    // note: this must be called in the run loop, not in configure or start as these are all different threads!
    //     int argc = 1;
    //     char argv0[] = "PlanePopOut";
    //     char *argv[1] = {argv0};

#ifdef FEAT_VISUALIZATION
    SendOverlays();
#endif

    try {
	while(isRunning())
	{
	    try{
		if ( ! GetImageData()) {
		    sleepComponent(1);		
		    continue;		
		}		//log("Hoho, we get the image data from PCL");
		if ( ! GetPlaneAndSOIs()) {
		    sleepComponent(1);
		    continue;		
		}	//log("Haha, we get the Sois and Plane from PCL");
	    }
	    catch (...) {
		error(" *** PPO GetImageData or GetPlaneAndSOIs is FUCKED UP *** ");
	    }
	    try{
		CalSOIHist(points,points_label, vec_histogram);			/// clear vec_histogram before store the new inside
	    }
	    catch (...) {
		error(" *** PPO CalSOIHist is FUCKED UP *** ");
	    }
	    // 	log("Yeah, we get the color histograms of all the sois");
	    try{
		if (sois.size() != 0)
		{						//log("we get some sois, now analyze them");
		    ConvexHullOfPlane(points,points_label);	//log("get the convex hull of the dominant plane");
		    CalCenterOfSOIs();				//log("Cal center of all sois");
		    CalSizeOfSOIs(); 				//log("cal the center and the soze of bounding cuboids");	//cal bounding Cuboids and centers of the points cloud
		    BoundingSphere(points,points_label); 	//log("cal the radius of bounding spheres");			// get bounding spheres, SOIs and ROIs
		    //if (SendDensePoints==1) CollectDensePoints(image.camPars, points);		/// TODO
#ifdef FEAT_VISUALIZATION
		    if (m_bSendImage)
		    {
			SendImage();
			// 		cout<<"send Imgs"<<endl;
		    }
#endif
		}
		else
		{
		    v3size.clear();
		    v3center.clear();
		    vdradius.clear();
		    // 	    log("there is no objects, now strating cal convex hull");
		    ConvexHullOfPlane(points,points_label);			//log("although there is no object, we still can get the convex hull of the dominant plane");
		}
	    }
	    catch (...) {
		error(" *** PPO ConvexHullOfPlane is FUCKED UP *** ");
	    }

	    try {
		if (doDisplay)
		{
		    DisplayInTG();
		}
#ifdef FEAT_VISUALIZATION
		// 	log("Start to send points");
		// 	int debug_labels=0;
		// 	for (int i=0; i<points_label.size(); i++)
		// 	  if (points_label[i]== 0)	{debug_labels++; /*log("plane point is at (%f, %f, %f)", points[i].p.x, points[i].p.y, points[i].p.z);*/}
		// 	log("Thera are %d points on the plane", debug_labels);

		if (m_bSendPoints) SendPoints(points, points_label, m_bColorByLabel, m_tmSendPoints); //log("Done sendpoints");
		if (m_bSendPlaneGrid) SendPlaneGrid();
		// 	log("Done FEAT_VISUALIZATION");
	    }
	    catch (...) {
		error(" *** PPO display is FUCKED UP *** ");
	    }
#endif
	    // 	AddConvexHullinWM();
	    //log("Done AddConvexHullinWM");

	    try {
		static int lastSize = -1;
		//log("A, B, C, D = %f, %f, %f, %f", A,B,C,D);
		//Pre2CurrentList.clear();
		if (lastSize != v3center.size()) {
		    // 	    log("SOI COUNT = %d",v3center.size());	
		    lastSize = v3center.size();
		}
		CurrentObjList.clear();
		// 	log("start create objects");
		for(unsigned int i=0; i<v3center.size(); i++)  //create objects
		{
		    ObjPara OP;				//log("v3center size is %d", v3center.size());
		    OP.c = v3center.at(i);		//log("v3size size is %d, i = %d",v3size.size(),i);
		    OP.s = v3size.at(i);		//log("vdradius size is %d", vdradius.size());
		    OP.r = vdradius.at(i);
		    OP.id = "";
		    OP.bComCurrentPre = false;
		    OP.bInWM = false;
		    OP.count = 0;				//log("SOIPointsSeq size is %d", SOIPointsSeq.size());
		    OP.pointsInOneSOI = SOIPointsSeq.at(i);	//log("BGPointsSeq size is %d", BGPointsSeq.size());
		    OP.BGInOneSOI = BGPointsSeq.at(i);		//log("EQPointsSeq size is %d", EQPointsSeq.size());
		    OP.EQInOneSOI = EQPointsSeq.at(i);		//log("vec_histogram size is %d", vec_histogram.size());
		    OP.hist = vec_histogram.at(i);
		    CurrentObjList.push_back(OP);
		}
		// 	log("Start SOIManagement");
		SOIManagement();
	    }
	    catch (...) {
		error(" *** PPO SOIManagement is FUCKED UP *** ");
	    }
	    // 	log("Done SOIManagement");
	    try {
		CleanupAll();
	    }
	    catch (...) {
		error(" *** PPO CleanupAll is FUCKED UP *** ");
	    }
	}
    }
    catch (...) {
	error(" *** PPO isRunning is FUCKED UP *** ");
    }
    sleepComponent(50);
}

void PlanePopOut::CleanupAll()
{
    A=B=C=D=0.0;
    //     dpc->values[0]=dpc->values[1]=dpc->values[2]=dpc->values[3]=0.0;
    cvReleaseImage(&iplImage_l);
    cvReleaseImage(&iplImage_r);
    cvReleaseImage(&iplImage_k);
    cvReleaseImage(&ROIMaskImg);
    mConvexHullPoints.clear();
    points.clear();
    points_label.clear();
    v3center.clear();
    vdradius.clear();
    v3size.clear();
    sois.clear();
    vec_histogram.clear();
    SOIPointsSeq.clear();
    BGPointsSeq.clear();
    EQPointsSeq.clear();
    Pre2CurrentList.clear();
}

void PlanePopOut::SaveHistogramImg(CvHistogram* hist, std::string str)
{
    int h_bins = 16, s_bins = 8;

    int height = 240;
    int width = (h_bins*s_bins*6);
    IplImage* hist_img = cvCreateImage( cvSize(width,height), 8, 3 );
    cvZero( hist_img );
    float max_value = 0;
    cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );

    /** temp images, for HSV to RGB tranformation */
    IplImage * hsv_color = cvCreateImage(cvSize(1,1),8,3);
    IplImage * rgb_color = cvCreateImage(cvSize(1,1),8,3);
    int bin_w = width / (h_bins * s_bins);
    for(int h = 0; h < h_bins; h++)
    {
	for(int s = 0; s < s_bins; s++)
	{
	    int i = h*s_bins + s;
	    /** calculate the height of the bar */
	    float bin_val = cvQueryHistValue_2D( hist, h, s );
	    int intensity = cvRound(bin_val*height/max_value);

	    /** Get the RGB color of this bar */
	    cvSet2D(hsv_color,0,0,cvScalar(h*180.f / h_bins,s*255.f/s_bins,255,0));
	    cvCvtColor(hsv_color,rgb_color,CV_HSV2RGB);
	    CvScalar color = cvGet2D(rgb_color,0,0);

	    cvRectangle( hist_img, cvPoint(i*bin_w,height),
		    cvPoint((i+1)*bin_w,height - intensity),
		    color, -1, 8, 0 );
	}
    }
    std::string path = str;
    path.insert(0,"/tmp/H-S-histogram_"); path.insert(path.length(),".jpg");
    cvSaveImage(path.c_str(), hist_img);
    cvReleaseImage(&hist_img);
    cvReleaseImage(&hsv_color);
    cvReleaseImage(&rgb_color);
}

void PlanePopOut::SOIManagement()
{
//     log("Start SOI Management! There are %d SOI in the previous scene", PreviousObjList.size());
    if (PreviousObjList.empty())
    {
	for(unsigned int i=0; i<CurrentObjList.size(); i++)
	{
	    CurrentObjList.at(i).count++;
	    PreviousObjList.push_back(CurrentObjList.at(i));
	}
	return;
    }
    //--------------there is no SOI in CurrentObjList----------------------------
    if (CurrentObjList.empty())
    {
	for (unsigned int j=0; j<PreviousObjList.size(); j++)
	{
	    if(PreviousObjList.at(j).bInWM == true)
	    {
		PreviousObjList.at(j).count = PreviousObjList.at(j).count-1;
		if(PreviousObjList.at(j).count > StableTime-AgonalTime) Pre2CurrentList.push_back(PreviousObjList.at(j));
		else
		{
		    if (bWriteSoisToWm)
		    {
#ifdef FEAT_VISUALIZATION
			SendRemoveSoi(PreviousObjList.at(j));
#endif
			deleteFromWorkingMemory(PreviousObjList.at(j).id);
			//  cout<<"Delete!! ID of the deleted SOI = "<<PreviousObjList.at(j).id<<endl;
		    }
		}
	    }
	    else
		if(PreviousObjList.at(j).count > 0) Pre2CurrentList.push_back(PreviousObjList.at(j));
	}
	PreviousObjList.clear();
	if (!Pre2CurrentList.empty())
	{
	    if (Pre2CurrentList.size()>0)
		for (unsigned int i=0; i<Pre2CurrentList.size(); i++)
		    PreviousObjList.push_back(Pre2CurrentList.at(i));
	}
	return;
    }
    //     for (unsigned int j=0; j<PreviousObjList.size(); j++)
    // 	log("id in PreviousObjList are %s", PreviousObjList.at(j).id.c_str());

    //-----------------There are SOIs in CurrentObjList, so compare with objects in PreviousObjList
    std::vector <SOIMatch> myMatchingSOIVector;
    for (unsigned int j=0; j<PreviousObjList.size(); j++)
    {
	int matchingObjIndex = 0;
	double max_matching_probability = 0.0;
	for(unsigned int i=0; i<CurrentObjList.size(); i++)
	{
	    if (CurrentObjList.at(i).bComCurrentPre == false)
	    {
		float probability = Compare2SOI(CurrentObjList.at(i), PreviousObjList.at(j));
// 		log("The matching probability of %d SOI in Current and %s in Previous is %f",i+1, PreviousObjList.at(j).id.c_str(), probability);
		if (probability > max_matching_probability)
		{
		    max_matching_probability = probability;
		    matchingObjIndex = i;
		}
	    }
	}
// 	log("The matching probability of %d in Current and %d in Previous is %f",matchingObjIndex, j, max_matching_probability);
	if (max_matching_probability>Treshold_Comp2SOI)
	{
	    SOIMatch SOIm;
	    SOIm.p = j; SOIm.c = matchingObjIndex; SOIm.pro = max_matching_probability;
	    myMatchingSOIVector.push_back(SOIm);
	    CurrentObjList.at(matchingObjIndex).bComCurrentPre = true;
	}
    }
    //-------------------handle the disappearing object
    for (unsigned int j=0; j<PreviousObjList.size(); j++)
    {
	int matchingResult = IsMatchingWithOneSOI(j,myMatchingSOIVector);
	if (matchingResult<0)	//disappearing object
	{
	    if(PreviousObjList.at(j).bInWM == true)
	    {
		PreviousObjList.at(j).count = PreviousObjList.at(j).count-1;
		if(PreviousObjList.at(j).count > StableTime-AgonalTime) Pre2CurrentList.push_back(PreviousObjList.at(j));
		else
		{
		    if (bWriteSoisToWm)
		    {
#ifdef FEAT_VISUALIZATION
			SendRemoveSoi(PreviousObjList.at(j));
#endif
			// cout<<"count of obj = "<<PreviousObjList.at(j).count<<endl;
			deleteFromWorkingMemory(PreviousObjList.at(j).id);
// 			cout<<"Delete!! ID of the deleted SOI = "<<PreviousObjList.at(j).id<<endl;
		    }
		}
	    }
	    else
	    {
		PreviousObjList.at(j).count = PreviousObjList.at(j).count -1;
		if(PreviousObjList.at(j).count > 0) Pre2CurrentList.push_back(PreviousObjList.at(j));
	    }
	}
	else	//find the matching object with this previous SOI
	{
	    if(PreviousObjList.at(j).bInWM == true)
	    {
		CurrentObjList.at(matchingResult).bInWM = true;
		CurrentObjList.at(matchingResult).id = PreviousObjList.at(j).id;
		CurrentObjList.at(matchingResult).hist = PreviousObjList.at(j).hist;
		// 		CurrentObjList.at(matchingResult).rect = PreviousObjList.at(j).rect;
		CurrentObjList.at(matchingResult).count = PreviousObjList.at(j).count;
		if (dist(CurrentObjList.at(matchingResult).c, PreviousObjList.at(j).c)/norm(CurrentObjList.at(matchingResult).c) > 0.15)
		{
		    int i = matchingResult;
		    if (bWriteSoisToWm)
		    {
			SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r,CurrentObjList.at(i).pointsInOneSOI, CurrentObjList.at(i).BGInOneSOI, CurrentObjList.at(i).EQInOneSOI);
// 			log("Overwrite Object in the WM, id is %s", CurrentObjList.at(i).id.c_str());
			overwriteWorkingMemory(CurrentObjList.at(i).id, obj);
#ifdef FEAT_VISUALIZATION
			SendSoi(CurrentObjList.at(i));
#endif
		    }
		}
		else
		{
#ifdef FEAT_VISUALIZATION
		    SendRemoveSoi(PreviousObjList.at(j));
#endif
		    CurrentObjList.at(matchingResult).c = PreviousObjList.at(j).c;
#ifdef FEAT_VISUALIZATION
		    SendSoi(CurrentObjList.at(matchingResult));
#endif
		}
	    }
	    else
	    {
		int i = matchingResult;
		CurrentObjList.at(i).count = PreviousObjList.at(j).count+1;
		if (CurrentObjList.at(i).count >= StableTime)
		{
		    if (bWriteSoisToWm)
		    {
			CurrentObjList.at(i).bInWM = true;
			CurrentObjList.at(i).id = newDataID();
			SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r, CurrentObjList.at(i).pointsInOneSOI, CurrentObjList.at(i).BGInOneSOI, CurrentObjList.at(i).EQInOneSOI);
// 			log("Add an New Object in the WM, id is %s", CurrentObjList.at(i).id.c_str());
			addToWorkingMemory(CurrentObjList.at(i).id, obj);

#ifdef FEAT_VISUALIZATION
			SendSoi(CurrentObjList.at(i));
#endif
		    }
#ifdef SAVE_SOI_PATCH
// 		    std::string path = CurrentObjList.at(i).id;
// 		    SaveHistogramImg(CurrentObjList.at(i).hist, path);
// 		    path.insert(0,"/tmp/"); path.insert(path.length(),".jpg");
// 		    IplImage* cropped = cvCreateImage( cvSize(CurrentObjList.at(i).rect.width,CurrentObjList.at(i).rect.height), previousImg->depth, previousImg->nChannels );
// 		    cvSetImageROI( previousImg, CurrentObjList.at(i).rect);
// 		    cvCopy( previousImg, cropped );
// 		    cvResetImageROI( previousImg );
// 		    cvSaveImage(path.c_str(), cropped);
// 		    cvReleaseImage(&cropped);
#endif
		    //	    log("222222222 Add an New Object in the WM, id is %s", CurrentObjList.at(i).id.c_str());
		    //    log("objects number = %u",objnumber);
		    //	    cout<<"New!! ID of the added SOI = "<<CurrentObjList.at(i).id<<endl;
		}
	    }
	}
    }
    //--------------------handle the new appearing object
    for(unsigned int i=0; i<CurrentObjList.size(); i++)
    {
	if (CurrentObjList.at(i).bComCurrentPre ==false)
	{
	    CurrentObjList.at(i).count = CurrentObjList.at(i).count-2;
	    // 	    log("We have new object, Wooo Hooo.... There are %d objects in CurrentObjList and this is the %d one", CurrentObjList.size(), i);
	}
    }
//         for (unsigned int j=0; j<CurrentObjList.size(); j++)
// 	    log("id in CurrentObjList are %s", CurrentObjList.at(j).id.c_str()); 
    PreviousObjList.clear();
    for (unsigned int i=0; i<CurrentObjList.size(); i++)
	PreviousObjList.push_back(CurrentObjList.at(i));
    if (Pre2CurrentList.size()>0)
	for (unsigned int i=0; i<Pre2CurrentList.size(); i++)
	    PreviousObjList.push_back(Pre2CurrentList.at(i));

//     vSOIonImg.clear();
    vSOIid.clear();
    for (unsigned int i=0; i<PreviousObjList.size(); i++)
    {
// 	vSOIonImg.push_back(PreviousObjList.at(i).rect);
	vSOIid.push_back(PreviousObjList.at(i).id);
    }
    myMatchingSOIVector.clear();
}

int PlanePopOut::IsMatchingWithOneSOI(int index, std::vector <SOIMatch> mlist)
{
    int IndexMatching = -1;
    for (unsigned int i=0; i<mlist.size(); i++)
    {
	if (index == mlist.at(i).p)
	{
	    IndexMatching = mlist.at(i).c;
	    break;
	}
    }
    return IndexMatching;
}

// @author: mmarko
void PlanePopOut::GetStableSOIs(std::vector<SOIPtr>& soiList)
{
    // The component doesn't have any locking. We copy the object list to *reduce*
    // the amount of concurrent access from multiple threads to CurrentObjList.
    // TODO: implement locking of CurrentObjList.
    vector<ObjPara> allpars = CurrentObjList;

    for (int i = 0; i < allpars.size(); i++)
    {
	ObjPara& par = allpars.at(i);
	if (par.count >= StableTime)
	{
	    SOIPtr obj = createObj(par.c, par.s, par.r, par.pointsInOneSOI, par.BGInOneSOI, par.EQInOneSOI);
	    soiList.push_back(obj);
	}
    }
}

// @author: mmarko
void PlanePopOut::onAdd_GetStableSoisCommand(const cast::cdl::WorkingMemoryChange& _wmc)
{
    class CCmd:
	public VisionCommandNotifier<GetStableSoisCommand, GetStableSoisCommandPtr>
    {
	public:
	    CCmd(cast::WorkingMemoryReaderComponent* pReader)
		: VisionCommandNotifier<GetStableSoisCommand, GetStableSoisCommandPtr>(pReader) {}
	protected:
	    virtual void doFail() { pcmd->status = VisionData::VCFAILED; }
	    virtual void doSucceed() { pcmd->status = VisionData::VCSUCCEEDED; }
    } cmd(this);

    println("PlanePopOut: GetStableSoisCommand.");

    if (! cmd.read(_wmc.address)) {
	debug("PlanePopOut: GetStableSoisCommand deleted while working...");
	return;
    }

    // TODO: getComponentID || stereoServer->componentId
    if (cmd.pcmd->componentId != getComponentID()) {
	debug("GetStableSoisCommand is not for me, but for '%s'.", cmd.pcmd->componentId.c_str());
	return;
    }

    debug("PlanePopOut: Will handle a GetStableSoisCommand.");

    /* TODO: the command should be handled in runComponent, where it would wait until
     * the sois are stable */
    GetStableSOIs(cmd.pcmd->sois);
    cmd.succeed();
    debug("PlanePopOut: GetStableSoisCommand found %d SOIs.", cmd.pcmd->sois.size());
}

CvPoint PlanePopOut::ProjectPointOnImage(Vector3 p)
{
    double u, v;
    double scale = 1;
    if(p.x == 0.) p.x=0.000001;   // instead: assert(Z != 0.);

    u = fx*p.x/scale + cx*p.z/scale;
    v = fy*p.y/scale + cy*p.z/scale;
    u /= p.z;
    v /= p.z;
    CvPoint re;
    re.x = (int) u; re.y = (int) v;
    return re;
}


/**
 * @brief Get images with the resolution, defined in the cast file, from video server.
 */
bool PlanePopOut::GetImageData()
{
    sois.clear();
    int pointCloudWidth = 320;                                /// TODO get from cast-file!
    int pointCloudHeight = pointCloudWidth *3/4;
    int kinectImageWidth = 640;
    int kinectImageHeight = kinectImageWidth *3/4;

    points.clear();
    //  getPoints(true, pointCloudWidth, points);
    getCompletePoints(true, pointCloudWidth, points);            // call get points only once, if noCont option is on!!! (false means no transformation!!!)
    //     log("there are %d points from GetPoints",points.size());
    ConvertKinectPoints2MatCloud(points, kinect_point_cloud, pointCloudWidth);
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pclA::ConvertCvMat2PCLCloud(kinect_point_cloud, *pcl_cloud);

    // get rectified images from point cloud server
    getRectImage(0, kinectImageWidth, image_l);            // 0 = left image / we take it with kinect image width
    getRectImage(1, kinectImageWidth, image_r);            // 1 = right image / we take it with kinect image width
    iplImage_l = convertImageToIpl(image_l);
    iplImage_r = convertImageToIpl(image_r);

    if(bWithKinect == true)
    {
	getRectImage(2, kinectImageWidth, image_k);            // 2 = kinect image / we take it with kinect image width
	iplImage_k = convertImageToIpl(image_k);
    }

    if (pcl_cloud->points.size()<100)	
    {
	//       log("less than 100 points???");
	return false;
    }
    else 
    {
	//       log("there are %d points from pcl", pcl_cloud->points.size());
	return true;

    }
}

bool PlanePopOut::GetPlaneAndSOIs()
{
    if (! planePopout)
	planePopout = new pclA::PlanePopout();

    try {
	if (!planePopout->CalculateSOIs(pcl_cloud))	{
	    //       log("Cal SOIs error!!!");
	    return false;
	}

	planePopout->GetSOIs(sois);
	planePopout->GetDominantPlaneCoefficients(dpc);
	if (dpc->values[3]>0)  {
	    A = dpc->values[0]; B = dpc->values[1]; C = dpc->values[2]; D = dpc->values[3];
	}
	else {
	    A = -dpc->values[0]; B = -dpc->values[1]; C = -dpc->values[2]; D = -dpc->values[3];
	}
	planePopout->GetTableHulls(tablehull);	//log("We get the table hull");
	planePopout->CollectTableInliers(pcl_cloud,dpc);	
	planePopout->GetPlanePoints(planepoints);	//log("There are %d inliers on the plane !", planepoints->indices.size());
    }
    catch (...) {
	error(" *** PPO CalculateSOIs ... GetPlanePoints is FUCKED UP *** ");
    }
    int w,h;
    if (bWithKinect) {
	w =iplImage_k->width;	h=iplImage_k->height;
	ROIMaskImg=cvCreateImage(cvSize(w,h),8,3);
	cvCopy(iplImage_k, ROIMaskImg,NULL);
    }
    else {
	w =iplImage_l->width;	h=iplImage_l->height;
	ROIMaskImg=cvCreateImage(cvSize(w,h),8,3);
	cvCopy(iplImage_l, ROIMaskImg,NULL);
    }
    objnumber=sois.size();	//log("In GetPlaneAndSOIs: There are %d SOIs !", objnumber);
    points.clear();	points.resize(pcl_cloud->points.size());
    points_label.clear();	points_label.assign(pcl_cloud->points.size(), -1);
    for (unsigned i=0; i<planepoints->indices.size(); i++)	// use indices of plane points to lable them
    {
	int index = planepoints->indices[i];		//index of plane point
// 	log("the index of plane point is %d", index);
	points_label.at(index) = 0;	//plane points	== label 0
	PointCloud::SurfacePoint PushStructure;
	Vector3 tmp;
	tmp.x = pcl_cloud->points[index].x; tmp.y = pcl_cloud->points[index].y; tmp.z = pcl_cloud->points[index].z;
//  	log("plane point at (%f, %f, %f)", tmp.x, tmp.y, tmp.z);
	PushStructure.p = tmp;
	PushStructure.c.r = pcl_cloud->points[index].r;	PushStructure.c.g = pcl_cloud->points[index].g;	PushStructure.c.b = pcl_cloud->points[index].b;
	points[index] = PushStructure;
    }	//log("label all the plane points");
    for (unsigned i=0; i<pcl_cloud->points.size(); i++)		// check all the points in which SOI and lable them
    {
	if (points_label[i] != 0)
	{
	    PointCloud::SurfacePoint PushStructure;
	    Vector3 tmp; tmp.x = pcl_cloud->points[i].x; tmp.y = pcl_cloud->points[i].y; tmp.z = pcl_cloud->points[i].z;
	    PushStructure.p = tmp;
	    PushStructure.c.r = pcl_cloud->points[i].r;	PushStructure.c.g = pcl_cloud->points[i].g;	PushStructure.c.b = pcl_cloud->points[i].b;
	    points[i] = PushStructure;
	    int index_of_SOI_point = planePopout->IsInSOI(tmp.x,tmp.y,tmp.z);
	    if (index_of_SOI_point !=0) points_label[i] = index_of_SOI_point;
	}
    }	//log("lable all the SOI points");
    return true;
}

/**
 * Calculate the Histogram of all the SOIs.
 */
void PlanePopOut::CalSOIHist(PointCloud::SurfacePointSeq pcloud, std::vector< int > label, std::vector <CvHistogram*> & vH)
{
    vH.clear();
    int max = *max_element(label.begin(), label.end());
    if (max == 0)	return;
    std::vector <PointCloud::SurfacePointSeq> VpointsOfSOI;
    PointCloud::SurfacePointSeq pseq = pcloud; pseq.clear();
    VpointsOfSOI.assign(max, pseq);

    for (unsigned int i=0; i<pcloud.size(); i++)
    {
	int k = label[i];
	if (k>0)
	    VpointsOfSOI[k-1].push_back(pcloud[i]);
    }

    for (unsigned j=0; j<VpointsOfSOI.size(); j++)
    {
	CvHistogram* h;
	IplImage* tmp = cvCreateImage(cvSize(1, VpointsOfSOI[j].size()), 8, 3);		///temp image, store all the points in the SOI in a matrix way
	for (unsigned int i=0; i<VpointsOfSOI[j].size(); i++)
	{
	    CvScalar v;	  
	    v.val[0] = VpointsOfSOI[j][i].c.b;	    v.val[1] = VpointsOfSOI[j][i].c.g;	    v.val[2] = VpointsOfSOI[j][i].c.r;
	    cvSet2D(tmp, i, 0, v);
	}
	IplImage* h_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
	IplImage* s_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
	IplImage* v_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
	IplImage* planes[] = { h_plane, s_plane };
	IplImage* hsv = cvCreateImage( cvGetSize(tmp), 8, 3 );
	int h_bins = 16, s_bins = 8;
	int hist_size[] = {h_bins, s_bins};
	/* hue varies from 0 (~0 deg red) to 180 (~360 deg red again) */
	float h_ranges[] = { 0, 180 };
	/* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
	float s_ranges[] = { 0, 255 };
	float* ranges[] = { h_ranges, s_ranges };
	float max_value = 0;

	cvCvtColor( tmp, hsv, CV_BGR2HSV );
	cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
	h = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	cvCalcHist( planes, h, 0, 0 );
	cvGetMinMaxHistValue( h, 0, &max_value, 0, 0 );

	cvReleaseImage(&h_plane);
	cvReleaseImage(&s_plane);
	cvReleaseImage(&v_plane);
	cvReleaseImage(&hsv);
	cvReleaseImage(&tmp);
	vH.push_back(h);
    }
}

SOIPtr PlanePopOut::createObj(Vector3 center, Vector3 size, double radius, PointCloud::SurfacePointSeq psIn1SOI, PointCloud::SurfacePointSeq BGpIn1SOI, PointCloud::SurfacePointSeq EQpIn1SOI)
{
    //debug("create an object at (%f, %f, %f) now", center.x, center.y, center.z);
    VisionData::SOIPtr obs = new VisionData::SOI;
    obs->sourceId = getComponentID();
    obs->status = 0;
    obs->boundingBox.pos.x = obs->boundingSphere.pos.x = center.x;
    obs->boundingBox.pos.y = obs->boundingSphere.pos.y = center.y;
    obs->boundingBox.pos.z = obs->boundingSphere.pos.z = center.z;
    obs->boundingBox.size.x = size.x;
    obs->boundingBox.size.y = size.y;
    obs->boundingBox.size.z = size.z;	//cout<<"radius in SOI = "<<radius<<endl;
    obs->boundingSphere.rad = radius;       //cout<<"radius in SOI (obs) = "<<obs->boundingSphere.rad<<endl;
    obs->time = getCASTTime();
    obs->points = psIn1SOI;//cout<<"points in 1 SOI = "<<obs->points.at(1).p<<obs->points.at(2).p<<obs->points.at(10).p<<endl;
    obs->BGpoints =	BGpIn1SOI;
    obs->EQpoints =	EQpIn1SOI;//cout<<"EQ points in 1 SOI = "<<obs->EQpoints.size()<<endl;

    return obs;
}

// compare two color histogram using Kullback–Leibler divergence
double PlanePopOut::CompareHistKLD(CvHistogram* h1, CvHistogram* h2)
{
    cvNormalizeHist( h1, 1.0 ); // Normalize it
    cvNormalizeHist( h2, 1.0 ); 
    int h_bins = 16, s_bins = 8;
    double KLD = 0.0;

    for(int h = 0; h < h_bins; h++)
    {
	for(int s = 0; s < s_bins; s++)
	{
	    /** calculate the height of the bar */
	    float bin_val1 = cvQueryHistValue_2D( h1, h, s );
	    float bin_val2 = cvQueryHistValue_2D( h2, h, s );
	    if (bin_val1 != 0.0 && bin_val2!= 0.0)	KLD = (log10(bin_val1/bin_val2)*bin_val1+log10(bin_val2/bin_val1)*bin_val2)/2 + KLD;
	    //log("Now bin_val = %f, %f", bin_val1, bin_val2);
	    //log("Now KLD = %f", KLD);
	}
    }
    return KLD;
}

float PlanePopOut::Compare2SOI(ObjPara obj1, ObjPara obj2)
{
    //return the probability of matching of two objects, 0.0~1.0

    float wC, wP; //magic weight for color histogram and Size/Position/Pose
    wP = 0.1;
    wC = 1.0-wP;
    double dist_histogram = CompareHistKLD(obj1.hist, obj2.hist);
//     log("The KLD of two objects are %f.", dist_histogram);
//     double sizeRatio;
//     double s1, s2; 
//     s1 = obj1.pointsInOneSOI.size();  s2 = obj2.pointsInOneSOI.size();
//     double smax; if (s1>s2) smax=s1; else smax=s2;
//     sizeRatio = 1.0-exp(-(s2-s1)*(s2-s1)*3.14159/smax);

    return 1.0-wC*abs(dist_histogram);//-wP*sizeRatio;
}

void PlanePopOut::AddConvexHullinWM()
{
    double T_CenterHull = 0.5 * mConvexHullRadius;
    VisionData::ConvexHullPtr CHPtr = new VisionData::ConvexHull;
    Pose3 p3;
    setIdentity(p3);
    Vector3 v3;
    setZero(v3);

    if (pre_mConvexHullRadius == 0.0)
    {
	if (mConvexHullPoints.size()>0)
	{
	    //debug("There are %u points in the convex hull", mConvexHullPoints.size());
	    CHPtr->PointsSeq = mConvexHullPoints;
	    CHPtr->time = getCASTTime();
	    p3.pos = mCenterOfHull;

	    CHPtr->center = p3;
	    CHPtr->radius = mConvexHullRadius;
	    CHPtr->density = mConvexHullDensity;
	    CHPtr->Objects = mObjSeq;
	    CHPtr->plane.a = A; CHPtr->plane.b = B; CHPtr->plane.c = C; CHPtr->plane.d = D;
	    pre_id = newDataID();
	    addToWorkingMemory(pre_id,CHPtr);

	    pre_mConvexHullRadius = mConvexHullRadius;
	    pre_mCenterOfHull = mCenterOfHull;
	}
    }
    else
    {
	if (mConvexHullPoints.size()>0)
	{
	    //debug("There are %u points in the convex hull", mConvexHullPoints.size());
	    CHPtr->PointsSeq = mConvexHullPoints;
	    CHPtr->time = getCASTTime();
	    p3.pos = mCenterOfHull;

	    CHPtr->center = p3;
	    CHPtr->radius = mConvexHullRadius;
	    CHPtr->density = mConvexHullDensity;
	    CHPtr->Objects = mObjSeq;
	    CHPtr->plane.a = A; CHPtr->plane.b = B; CHPtr->plane.c = C; CHPtr->plane.d = D;
	    if (dist(pre_mCenterOfHull, mCenterOfHull) > T_CenterHull)
	    {
		//cout<<"dist = "<<dist(pre_mCenterOfHull, mCenterOfHull)<<"  T = "<<T_CenterHull<<endl;
		//debug("add sth into WM");
		pre_id = newDataID();
		addToWorkingMemory(pre_id,CHPtr);
		pre_mConvexHullRadius = mConvexHullRadius;
		pre_mCenterOfHull = mCenterOfHull;
	    }
	    else
	    {
		overwriteWorkingMemory(pre_id, CHPtr);
	    }
	}
    }

    mConvexHullPoints.clear();
    //mObjSeq.clear();
    mCenterOfHull.x = mCenterOfHull.y = mCenterOfHull.z = 0.0;
    mConvexHullRadius = 0.0;
    mConvexHullDensity = 0.0;
}

Vector3 PlanePopOut::ProjectOnDominantPlane(Vector3 InputP)
{
    Vector3 OutputP;
    OutputP.x = ((B*B+C*C)*InputP.x-A*(B*InputP.y+C*InputP.z+D))/(A*A+B*B+C*C);
    OutputP.y = ((A*A+C*C)*InputP.y-B*(A*InputP.x+C*InputP.z+D))/(A*A+B*B+C*C);
    OutputP.z = ((B*B+A*A)*InputP.z-C*(B*InputP.y+A*InputP.x+D))/(A*A+B*B+C*C);

    return OutputP;
}

Matrix33 PlanePopOut::GetAffineRotMatrix()
{
    Vector3 vb; //translation vector
    Vector3 v3normal;  //normal vector of dominant plane
    v3normal.x = A;	v3normal.y = B;	v3normal.z = C;
    normalise(v3normal);
    if(v3normal.z < 0)
	v3normal *= -1.0;

    Matrix33 rot;
    setIdentity(rot);
    setColumn(rot,2,v3normal);
    vb.x = 1;	vb.y = 0;	vb.z = 0;
    vb = vb-(v3normal*v3normal.x);
    normalise(vb);
    setRow(rot,0,vb);
    setZero(vb);
    vb = cross(getRow(rot,2),getRow(rot,0));
    setRow(rot,1,vb);
    setZero(vb);

    return rot;
}

inline Vector3 PlanePopOut::AffineTrans(Matrix33 m33, Vector3 v3)
{
    return m33*v3;
}

void PlanePopOut::ConvexHullOfPlane(PointCloud::SurfacePointSeq points, std::vector <int> labels)
{
    mConvexHullPoints.clear();
    Vector3 tmp;	Vector3 cen; cen.x=cen.y=cen.z=0.0;
    for (unsigned int i=0; i<tablehull->points.size(); i++)
    {
	cen.x=cen.x+tablehull->points[i].x; cen.y=cen.y+tablehull->points[i].y; cen.z=cen.z+tablehull->points[i].z;
	tmp.x=tablehull->points[i].x; tmp.y=tablehull->points[i].y; tmp.z=tablehull->points[i].z;
	mConvexHullPoints.push_back(tmp);
    }
    if (tablehull->points.size()!=0)
    {cen.x=cen.x/tablehull->points.size(); cen.y=cen.y/tablehull->points.size(); cen.z=cen.z/tablehull->points.size();}
    mCenterOfHull = cen;
    mConvexHullRadius = dist(cen,tmp);
    int Num_Inliers = 0;
    for (unsigned int i = 0; i<points.size(); i++)
	if (labels[i]==0)	Num_Inliers++;
    if (mConvexHullRadius !=0) mConvexHullDensity = Num_Inliers/PI/mConvexHullRadius/mConvexHullRadius;
//     log("PlanePopOut::ConvexHullOfPlane, there are %d inliers on the plane!", Num_Inliers);
}

void PlanePopOut::CalCenterOfSOIs()
{
    unsigned n=objnumber;
    double x=0.0;	double y=0.0;	double z=0.0;
    for (unsigned i=0; i<n; i++)
    {
	for (unsigned j=0; j<sois[i]->points.size(); j++)
	{
	    x= sois[i]->points[j].x+x;
	    y= sois[i]->points[j].y+y;
	    z= sois[i]->points[j].z+z;
	}
	x=x/sois[i]->points.size();
	y=y/sois[i]->points.size();
	z=z/sois[i]->points.size();
	Vector3 tmp;	tmp.x = x; tmp.y = y; tmp.z = z;
	v3center.push_back(tmp);
    }
}

void PlanePopOut::CalSizeOfSOIs()
{
    unsigned n=sois.size();
    Vector3 t; t.x=0.0; t.y=0.0; t.z=0.0;	double maxdist=0.0;
    v3size.clear();	v3size.assign(n,t);
    vdradius.clear(); vdradius.assign(n,0.0);

    for (unsigned i=0; i<n; i++)
    {
	for (unsigned j=0; j<sois[i]->points.size()/2; j++)
	{
	    t.x= sois[i]->points[j].x;
	    t.y= sois[i]->points[j].y;
	    t.z= sois[i]->points[j].z;
	    double dd = dist(t,v3center[i]);
	    if(dd> maxdist) maxdist= dd;
	}
	vdradius[i] =maxdist;
	Vector3 tm; tm.x=tm.y=tm.z=2*maxdist/sqrt(3);
	v3size[i] =tm;
    }
}

void PlanePopOut::BoundingSphere(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    PointCloud::SurfacePointSeq center;
    SOIPointsSeq.clear(); SOIPointsSeq.assign(objnumber, center);
    BGPointsSeq.clear();  BGPointsSeq.assign(objnumber, center);
    EQPointsSeq.clear();  EQPointsSeq.assign(objnumber, center);
    Vector3 initial_vector;
    initial_vector.x = 0;
    initial_vector.y = 0;
    initial_vector.z = 0;
    PointCloud::SurfacePoint InitialStructure;
    InitialStructure.p = initial_vector;
    center.assign(objnumber,InitialStructure);

    for (int i = 0; i<objnumber; i++)
    {
	Vector3 Center_DP = ProjectOnDominantPlane(center.at(i).p);//cout<<" center on DP ="<<Center_DP<<endl;
	for (unsigned int j = 0; j<points.size(); j++)
	{
	    PointCloud::SurfacePoint PushStructure;
	    PushStructure.p = points.at(j).p;
	    PushStructure.c = points.at(j).c;	//cout<<"in BG"<<PushStructure.c.r<<PushStructure.c.g<<PushStructure.c.b<<endl;
	    Vector3 Point_DP = ProjectOnDominantPlane(PushStructure.p);
	    int label = labels.at(j);
	    if (label > 0 && dist(Point_DP,Center_DP) < Shrink_SOI*vdradius.at(i))
		SOIPointsSeq.at(label-1).push_back(PushStructure);

	    if (label == -1 && dist(Point_DP,Center_DP) < Lower_BG*vdradius.at(i)) // equivocal points
		EQPointsSeq.at(i).push_back(PushStructure);

	    if (label == 0 && dist(Point_DP,Center_DP) < Upper_BG*vdradius.at(i) && dist(Point_DP,Center_DP) > Lower_BG*vdradius.at(i)) //BG nearby also required
		BGPointsSeq.at(i).push_back(PushStructure);

	}
    }
    center.clear();
}



void PlanePopOut::Points2Cloud(cv::Mat_<cv::Point3f> &cloud, cv::Mat_<cv::Point3f> &colCloud)
{
    cloud = cv::Mat_<cv::Point3f>(1, pointsN.size());
    colCloud = cv::Mat_<cv::Point3f>(1, pointsN.size());    

    for(unsigned i = 0; i<pointsN.size(); i++)
    {
	cv::Point3f p, cp;
	p.x = (float) pointsN[i].p.x;
	p.y = (float) pointsN[i].p.y;
	p.z = (float) pointsN[i].p.z;
	if (points_label.at(i) == 0)	//points belong to the dominant plane
	{
	    cp.x = 255.0;	// change rgb to bgr
	    cp.y = 0.0;
	    cp.z = 0.0;
	}
	else
	{
	    cp.x = (uchar) pointsN[i].c.b;	// change rgb to bgr
	    cp.y = (uchar) pointsN[i].c.g;
	    cp.z = (uchar) pointsN[i].c.r;
	}

	cloud.at<cv::Point3f>(0, i) = p;
	colCloud.at<cv::Point3f>(0, i) = cp;
    }
}

void PlanePopOut::DisplayInTG()
{
    cv::Mat_<cv::Point3f> cloud;
    cv::Mat_<cv::Point3f> colCloud;
    Points2Cloud(cloud, colCloud);
    if(doDisplay)
    {
	tgRenderer->Clear();
	tgRenderer->SetPointCloud(cloud, colCloud);
    }
}

}
// vim: set sw=4 ts=8 noet list :vim
