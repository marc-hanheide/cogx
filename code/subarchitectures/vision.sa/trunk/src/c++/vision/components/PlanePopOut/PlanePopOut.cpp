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

#define USE_MOTION_DETECTION
//#define SAVE_SOI_PATCH  //todo: fix that !crop the ROI may cause the crash since the size of the ROI may exceeds the boundaries of img  
#define USE_PSO	0	//0=use RANSAC, 1=use PSO to estimate multiple planes

#define Shrink_SOI 1
#define Upper_BG 1.5
#define Lower_BG 1.1	// 1.1-1.5 radius of BoundingSphere

#define MAX_V 0.1
#define label4initial		-3
#define label4plane		0	//0, -10, -20, -30... for multiple planes
#define label4objcandidant	-2
#define label4objs		1	//1,2,3,4.....for multiple objs
#define label4ambiguousness	-1

#define SendDensePoints  1 	//0 send sparse points ,1 send dense points (recollect them after the segmentation)
#define Treshold_Comp2SOI	0.8	//the similarity of 2 SOIs higher than this will make the system treat these 2 SOI as the sam one

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

int objnumber = 0;

PointCloud::SurfacePointSeq points;
PointCloud::SurfacePointSeq pointsN;

VisionData::ObjSeq mObjSeq;
VisionData::Vector3Seq mConvexHullPoints;
Vector3 mCenterOfHull;
double mConvexHullRadius;
double mConvexHullDensity;

Vector3 pre_mCenterOfHull;
double pre_mConvexHullRadius;
std::string pre_id;

vector <int> points_label;  //0->plane; 1~999->objects index; -1->discarded points
vector <CvRect> vSOIonImg;
vector <std::string> vSOIid;
int AgonalTime;	//The dying object could be "remembered" for "AgonalTime" of frames
int StableTime;	// Torleration error, even there are "Torleration" frames without data, previous data will still be used
//this makes stable obj

double A, B, C, D;
int N;  // 1/N points will be used
bool mbDrawWire;
bool doDisplay;
Vector3 v3dmax;
Vector3 v3dmin;



void PlanePopOut::configure(const map<string,string> & _config)
{
    // first let the base classes configure themselves
    configureServerCommunication(_config);

    map<string,string>::const_iterator it;

    useGlobalPoints = true;
    doDisplay = false;
    AgonalTime = 30;
    StableTime = 2;
    if((it = _config.find("--globalPoints")) != _config.end())
    {
	istringstream str(it->second);
	str >> boolalpha >> useGlobalPoints;
    }
    if((it = _config.find("--display")) != _config.end())
    {
	doDisplay = true;
    }
    if((it = _config.find("--minObjHeight")) != _config.end())
    {
	istringstream str(it->second);
	str >> min_height_of_obj;
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
    if((it = _config.find("--stereoconfig")) != _config.end())   // Configuration of stereo camera
    {
	stereoconfig = it->second;
    }
    else println("WARNING: No stereoconfig specified!");

    bWriteSoisToWm = true;
    if((it = _config.find("--generate-sois")) != _config.end())
    {
	bWriteSoisToWm = ! (it->second == "0" || it->second == "false" || it->second == "off");
    }
    println("%s write SOIs to WM", bWriteSoisToWm ? "WILL" : "WILL NOT"); 

    StereoCamera *stereo_cam = new StereoCamera();
    if(!stereo_cam->ReadSVSCalib(stereoconfig)) 
	throw (std::runtime_error("PlanePopOut::configure: Warning: Cannot open calibration file for stereo camera."));
    cv::Mat intrinsic = stereo_cam->GetIntrinsic(0);	// 0 == LEFT

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
    mConvexHullDensity = 0.0;
    pre_mCenterOfHull.x = pre_mCenterOfHull.y = pre_mCenterOfHull.z = 0.0;
    pre_mConvexHullRadius = 0.0;
    pre_id = "";
    bIsMoving = false;
    CurrentBestDistSquared = 999999.0;
    bHorizontalFound = false;
    bVerticalOn = false;
    previousImg=cvCreateImage(cvSize(1,1),8,3);

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
    //Video::Image image;
    //m_display.setImage(ID_OBJECT_IMAGE, image);
    m_tmSendPoints.restart();
#endif

    // we want to receive detected VisualObject
    addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
	    new MemberFunctionChangeReceiver<PlanePopOut>(this,
		&PlanePopOut::newVisualObject));
    // when VisualObject is deleted
    addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::DELETE),
	    new MemberFunctionChangeReceiver<PlanePopOut>(this,
		&PlanePopOut::deleteVisualObject));

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
	pPopout->SendSyncAllSois();
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

void SendImage(PointCloud::SurfacePointSeq& points, std::vector <int> &labels, const Video::Image& img, cogx::display::CDisplayClient& m_display, PlanePopOut *powner)
{
    //static castutils::CMilliTimer tmSendImage(true);
    //if (tmSendImage.elapsed() < 500) // 2Hz
    //    return;
    //tmSendImage.restart();

    castutils::CMilliTimer tm(true);
    IplImage *iplImg = convertImageToIpl(img);
    Video::CameraParameters c = img.camPars;

    for (unsigned int i=0 ; i<points.size() ; i++)
    {
	int m_label = labels.at(i);
	PointCloud::SurfacePoint& pt = points.at(i);
	switch (m_label)
	{
	    case 0:   cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(255,0,0)); break;
	    case -10: cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(0,255,0)); break;
	    case -20: cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(0,0,255)); break;
	    case -30: cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(0,255,255)); break;
	    case -40: cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(128,128,0)); break;
	    case -50: cvCircle(iplImg, powner->ProjectPointOnImage(pt.p,c), 2, CV_RGB(255,255,255)); break;
	}
    }
    CvFont a;
    cvInitFont( &a, CV_FONT_HERSHEY_PLAIN, 1, 1, 0 , 1 );
    for (unsigned int i=0 ; i<vSOIonImg.size() ; i++)
    {
	CvPoint p;
	CvRect& rsoi = vSOIonImg.at(i);
	p.x = (int)(rsoi.x+0.3 * rsoi.width);
	p.y = (int)(rsoi.y+0.5 * rsoi.height);
	cvPutText(iplImg, vSOIid.at(i).c_str(), p, &a,CV_RGB(255,255,255));
    }

    bool bSaveImage = true;
    long long t1 = tm.elapsed();
    m_display.setImage(ID_OBJECT_IMAGE, iplImg);
    long long t2 = tm.elapsed();
    if (bSaveImage)
	cvSaveImage("/tmp/planes_image.jpg", iplImg);
    long size = iplImg->imageSize;
    cvReleaseImage(&iplImg);
    long long t3 = tm.elapsed();

    if (1) {
	ostringstream str;
	str << "<h3>Plane popout - SendImage</h3>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Size: " << size << " bytes.<br>";
	str << "Sent: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	if (bSaveImage)
	    str << "Saved: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	m_display.setHtml("INFO", "log.PPO.SendImage", str.str());
    }
}

void SendPoints(const PointCloud::SurfacePointSeq& points, std::vector<int> &labels, bool bColorByLabels,
	cogx::display::CDisplayClient& m_display, castutils::CMilliTimer& tmSendPoints, PlanePopOut *powner)
{
    if (tmSendPoints.elapsed() < 500) // 2Hz
	return;
    tmSendPoints.restart();

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
	    if (plab != lab) {
		colors++;
		plab = lab;
		switch (lab) {
		    case 0:   str << "c(1.0,0.0,0.0)\n"; break;
		    case -10: str << "c(0.0,1.0,0.0)\n"; break;
		    case -20: str << "c(0.0,0.0,1.0)\n"; break;
		    case -30: str << "c(0.0,1.0,1.0)\n"; break;
		    case -40: str << "c(0.5,0.5,0.0)\n"; break;
		    case -5:  str << "c(0.0,0.0,0.0)\n"; break;
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
	str << "Colors: " << colors << "<br>";
	str << "Strlen: " << S.length() << "<br>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Converted: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	str << "Sent: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	m_display.setHtml("INFO", "log.PPO.SendPoints", str.str());
    }
}

void SendPlaneGrid(cogx::display::CDisplayClient& m_display, PlanePopOut *powner)
{
    static castutils::CMilliTimer tmSendPlaneGrid(true);
    if (tmSendPlaneGrid.elapsed() < 100) // 10Hz
	return;
    tmSendPlaneGrid.restart();

    castutils::CMilliTimer tm(true);
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
	m_display.setHtml("INFO", "log.PPO.SendPlaneGrid", str.str());
    }
}

void SendOverlays(cogx::display::CDisplayClient& m_display, PlanePopOut *powner)
{
    std::ostringstream str;
    str << "function render()\n";
    str << "glBegin(GL_LINES)\n";
    str << "glColor(1.0,0.0,0.0)\n";
    str << "glVertex(0., 0., 0.)\n";
    str << "glVertex(0.1, 0., 0.)\n";
    str << "glColor(0.0,1.0,0.0)\n";
    str << "glVertex(0., 0., 0.)\n";
    str << "glVertex(0., 0.1, 0.)\n";
    str << "glColor(0.0,0.0,1.0)\n";
    str << "glVertex(0., 0., 0.)\n";
    str << "glVertex(0., 0., 0.1)\n";
    str << "glEnd()\n";
    str << "showLabel(0.1, 0, 0, 'x', 12)\n";
    str << "showLabel(0, 0.1, 0, 'y', 12)\n";
    str << "showLabel(0, 0, 0.1, 'z', 12)\n";
    str << "end\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_OVERLAY, str.str());
}

void SendSoi(cogx::display::CDisplayClient& m_display, PlanePopOut::ObjPara& soiobj)
{
    ostringstream ss;
    ss << "function render()\n";
    ss << "glPushMatrix()\n";
    ss << "glColor(0.0, 0.0, 1.0, 0.2)\n";
    ss << "glTranslate("
	<< soiobj.c.x << ","
	<< soiobj.c.y << ","
	<< soiobj.c.z << ")\n";
    ss << "StdModel:cylinder(" << soiobj.s.x << "," << soiobj.s.y << "," << soiobj.s.z << ",12)\n";
    ss << "glPopMatrix()\n";
    ss << "end\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI + soiobj.id, ss.str());
}

void SendRemoveSoi(cogx::display::CDisplayClient& m_display, PlanePopOut::ObjPara& soiobj)
{
    // XXX: ATM the parts are not removed from LuaGL, so we just create an empty render()
    m_display.setLuaGlObject(ID_OBJECT_3D, ID_PART_3D_SOI + soiobj.id, "function render()\nend\n");
    m_display.removePart(ID_OBJECT_3D, ID_PART_3D_SOI + soiobj.id);
}

void PlanePopOut::SendSyncAllSois()
{
    vector<ObjPara> sois = CurrentObjList; // copy to minimize race conditions
    for (int i = 0; i < sois.size(); i++) {
	ObjPara& soi = sois.at(i);
	if (m_bSendSois) SendSoi(m_display, soi);
	else SendRemoveSoi(m_display, soi);
    }
}

#endif

void PlanePopOut::runComponent()
{
    sleepComponent(3000);
    log("Component PlanePopOut is running now");
    // note: this must be called in the run loop, not in configure or start as these are all different threads!
    int argc = 1;
    char argv0[] = "PlanePopOut";
    char *argv[1] = {argv0};
    int stereoWidth = 640;

#ifdef FEAT_VISUALIZATION
    SendOverlays(m_display, this);
#endif
    while(isRunning())
    {
	long long t0 = gethrtime();
	PointCloud::SurfacePointSeq tempPoints = points;
	points.resize(0);

	getPoints(useGlobalPoints, stereoWidth, points);
	Video::Image image;
	getRectImage(LEFT, stereoWidth, image);
#ifdef USE_MOTION_DETECTION
	if (previousImg->width == 1)	{previousImg = convertImageToIpl(image); bIsMoving = true;}
	{
	    IplImage* Cimg = convertImageToIpl(image);
	    IplImage* subimg=cvCreateImage(cvSize(Cimg->width,Cimg->height),Cimg->depth,Cimg->nChannels);
	    cvAbsDiff(Cimg,previousImg, subimg);
	    if (IsMoving(subimg))
	    {
		bIsMoving = true; 
		//log("Motion detected, freeze the 3D analysis");
	    }
	    else bIsMoving = false;
	    cvCopy(Cimg,previousImg , NULL);
	    cvReleaseImage(&Cimg);  
	    cvReleaseImage(&subimg);
	}
#endif

	if (bIsMoving)	continue;
	if (points.size() == 0)
	{
	    points = tempPoints;
	    log("Attention: getpoints() gets ZERO point!!");
	}
	else
	{	//cout<<"we get "<<points.size()<<" points"<<endl;
	    tempPoints.clear();
	    pointsN.clear();
	    objnumber = 0;
	    if (points.size()>= 10000)	N = (int)points.size()/10000;
	    else N = 1;
	    random_shuffle ( points.begin(), points.end() );
	    /* for debug
	       for (unsigned int k = 0; k<100; k++)
	       {
	       log("The %d point is at %f, %f, %f",k, points.at(k).p.x, points.at(k).p.y, points.at(k).p.z);
	       }
	       for debug */
	    for (PointCloud::SurfacePointSeq::iterator it=points.begin(); it<points.end(); it+=N)
		if ((*it).p.x*(*it).p.x+(*it).p.y*(*it).p.y+(*it).p.z*(*it).p.z<20 && (*it).p.x*(*it).p.x!=0)
		    pointsN.push_back(*it);
	    points_label.clear();
	    points_label.assign(pointsN.size(), -3);

	    /* for debug *
	       for (unsigned int k = 0; k<100; k++)
	       {
	       log("The %d point is at %f, %f, %f",k, pointsN.at(k).p.x, pointsN.at(k).p.y, pointsN.at(k).p.z);
	       }
	       / for debug */

	    bool executeflag = false;
	    executeflag = RANSAC(pointsN,points_label);
	
	    //log("parameters of plane: A, B, C = (%f, %f, %f)", A, B, C);
	    if (executeflag == true && A!=0 && B!=0 && C!=0)
	    {	//cout<<"after ransac we have "<<points.size()<<" points"<<endl;
		SplitPoints(pointsN,points_label);
		if (objnumber != 0)
		{
		    ConvexHullOfPlane(pointsN,points_label);
		    BoundingPrism(pointsN,points_label);
		    DrawCuboids(pointsN,points_label); //cal bounding Cuboids and centers of the points cloud
		    BoundingSphere(pointsN,points_label); // get bounding spheres, SOIs and ROIs
		    if (SendDensePoints==1) CollectDensePoints(image.camPars, points);
		    //cout<<"m_bSendImage is "<<m_bSendImage<<endl;
		    //log("Done CollectDensePoints");
#ifdef FEAT_VISUALIZATION
		    if (m_bSendImage)
		    {
			SendImage(pointsN,points_label,image, m_display, this);
			//cout<<"send Imgs"<<endl;
		    }
#endif
		}
		else
		{
		    v3size.clear();
		    v3center.clear();
		    vdradius.clear();
		    //cout<<"there is no objects, now strating cal convex hull"<<endl;
		    ConvexHullOfPlane(pointsN,points_label);
		}
		if (doDisplay)
		{
		    DisplayInTG();
		}
#ifdef FEAT_VISUALIZATION
		if (m_bSendPoints) SendPoints(pointsN, points_label, m_bColorByLabel, m_display, m_tmSendPoints, this);
		if (m_bSendPlaneGrid) SendPlaneGrid(m_display, this);
		//log("Done FEAT_VISUALIZATION");
#endif

		AddConvexHullinWM();
		//log("Done AddConvexHullinWM");
	    }
	    else	log("Wrong with the execution of Plane fitting!");
	}
	if (para_a!=0.0 || para_b!=0.0 || para_c!=0.0 || para_d!=0.0)
	{
	    static int lastSize = -1;
	    //log("A, B, C, D = %f, %f, %f, %f", A,B,C,D);
	    CurrentObjList.clear();
	    //Pre2CurrentList.clear();
	    if (lastSize != v3center.size()) {
		log("SOI COUNT = %d",v3center.size());	
		lastSize = v3center.size();
	    }
	    for(unsigned int i=0; i<v3center.size(); i++)  //create objects
	    {
		ObjPara OP;
		OP.c = v3center.at(i);
		OP.s = v3size.at(i);
		OP.r = vdradius.at(i);
		OP.id = "";
		OP.bComCurrentPre = false;
		OP.bInWM = false;
		OP.count = 0;
		OP.pointsInOneSOI = SOIPointsSeq.at(i);
		OP.BGInOneSOI = BGPointsSeq.at(i);
		OP.EQInOneSOI = EQPointsSeq.at(i);
		OP.hist = GetSurfAndHistogram(SOIPointsSeq.at(i), image,OP.surf, OP.rect);
		CurrentObjList.push_back(OP);
	    }

	    SOIManagement();
	    //log("Done SOIManagement");
	}
	//	long long t1 = gethrtime();
	//	double dt = (t1 - t0) * 1e-9;
	// 	log("run time / frame rate: %lf / %lf", dt, 1./dt);
	//cout<<"SOI in the WM = "<<PreviousObjList.size()<<endl;
	// wait a bit so we don't hog the CPU
	sleepComponent(50);
    }
}

bool PlanePopOut::IsMoving(IplImage * subimg)
{
    IplImage * pImage8uGray=NULL;
    pImage8uGray=cvCreateImage(cvGetSize(subimg),IPL_DEPTH_8U,1);
    cvCvtColor(subimg,pImage8uGray,CV_BGR2GRAY);
    int x, y;
    for(y = 0; y < pImage8uGray->height; y++){
	for(x = 0; x < pImage8uGray->width; x++){
	    int a = ((uchar*)(pImage8uGray->imageData+pImage8uGray->widthStep*y))[x];
	    if (a>50)	
	    {
		cvReleaseImage(&pImage8uGray);
		return true;
	    }
	}
    }
    cvReleaseImage(&pImage8uGray);
    return false;
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

void PlanePopOut::SOIManagement()
{
    //log("There are %d SOI in the Current scene", CurrentObjList.size());
    Pre2CurrentList.clear();
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
			deleteFromWorkingMemory(PreviousObjList.at(j).id);
			//  cout<<"Delete!! ID of the deleted SOI = "<<PreviousObjList.at(j).id<<endl;
#ifdef FEAT_VISUALIZATION
			if (m_bSendSois) SendRemoveSoi(m_display, PreviousObjList.at(j));
#endif
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
		//log("The matching probability of %d SOI in Current and %s in Previous is %f",i+1, PreviousObjList.at(j).id.c_str(), probability);
		if (probability > max_matching_probability)
		{
		    max_matching_probability = probability;
		    matchingObjIndex = i;
		}
	    }
	}
	//log("The matching probability of %d in Current and %d in Previous is %f",matchingObjIndex, j, max_matching_probability);
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
			// cout<<"count of obj = "<<PreviousObjList.at(j).count<<endl;
			deleteFromWorkingMemory(PreviousObjList.at(j).id);
			// cout<<"Delete!! ID of the deleted SOI = "<<PreviousObjList.at(j).id<<endl;
#ifdef FEAT_VISUALIZATION
			if (m_bSendSois) SendRemoveSoi(m_display, PreviousObjList.at(j));
#endif
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
			debug("Overwrite Object in the WM, id is %s", CurrentObjList.at(i).id.c_str());
			overwriteWorkingMemory(CurrentObjList.at(i).id, obj);
#ifdef FEAT_VISUALIZATION
			if (m_bSendSois) SendSoi(m_display, CurrentObjList.at(i));
#endif
		    }
		}
		else
		    CurrentObjList.at(matchingResult).c = PreviousObjList.at(j).c;
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
			debug("Add an New Object in the WM, id is %s", CurrentObjList.at(i).id.c_str());
			addToWorkingMemory(CurrentObjList.at(i).id, obj);

#ifdef FEAT_VISUALIZATION
			if (m_bSendSois) SendSoi(m_display, CurrentObjList.at(i));
#endif
		    }
#ifdef SAVE_SOI_PATCH
		    std::string path = CurrentObjList.at(i).id;
		    SaveHistogramImg(CurrentObjList.at(i).hist, path);
		    path.insert(0,"/tmp/"); path.insert(path.length(),".jpg");
		    IplImage* cropped = cvCreateImage( cvSize(CurrentObjList.at(i).rect.width,CurrentObjList.at(i).rect.height), previousImg->depth, previousImg->nChannels );
		    cvSetImageROI( previousImg, CurrentObjList.at(i).rect);
		    cvCopy( previousImg, cropped );
		    cvResetImageROI( previousImg );
		    cvSaveImage(path.c_str(), cropped);
		    cvReleaseImage(&cropped);

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
    //     for (unsigned int j=0; j<CurrentObjList.size(); j++)
    // 	/*log*/("id in CurrentObjList are %s", CurrentObjList.at(j).id.c_str()); 
    PreviousObjList.clear();
    for (unsigned int i=0; i<CurrentObjList.size(); i++)
	PreviousObjList.push_back(CurrentObjList.at(i));
    if (Pre2CurrentList.size()>0)
	for (unsigned int i=0; i<Pre2CurrentList.size(); i++)
	    PreviousObjList.push_back(Pre2CurrentList.at(i));

    vSOIonImg.clear();
    vSOIid.clear();
    for (unsigned int i=0; i<PreviousObjList.size(); i++)
    {
	vSOIonImg.push_back(PreviousObjList.at(i).rect);
	vSOIid.push_back(PreviousObjList.at(i).id);
    }
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

CvHistogram* PlanePopOut::GetSurfAndHistogram(PointCloud::SurfacePointSeq points, Video::Image img, IpVec& ips, CvRect &r)
{
    Video::CameraParameters c = img.camPars;
    IplImage *iplImage = convertImageToIpl(img);
    float maxx,maxy,minx,miny;	maxx=0.0; maxy=0.0; minx = 99999.0; miny= 99999.0;
    IplImage* tmp = cvCreateImage(cvSize(1, (int)points.size()), 8, 3);
    for (unsigned int i=0; i<points.size(); i++)
    {
	Vector3 v3OneObj = points.at(i).p;
	Vector2 SOIPointOnImg; SOIPointOnImg = projectPoint(c, v3OneObj);
	if (SOIPointOnImg.x>maxx)	maxx = SOIPointOnImg.x;
	if (SOIPointOnImg.y>maxy)	maxy = SOIPointOnImg.y;
	if (SOIPointOnImg.x<minx)	minx = SOIPointOnImg.x;
	if (SOIPointOnImg.y<miny)	miny = SOIPointOnImg.y;

	cogx::Math::ColorRGB rgb = points.at(i).c;
	CvScalar v;	  
	v.val[0] = rgb.b;
	v.val[1] = rgb.g;
	v.val[2] = rgb.r;
	cvSet2D(tmp, i, 0, v);
    }
    r.x = (int)minx; r.y = (int)miny; r.width =(int)(maxx-minx); r.height =(int)(maxy-miny); 
    IplImage* imgpatch=cvCreateImage(cvSize(r.width,r.height),iplImage->depth,iplImage->nChannels);
    imgpatch = Video::crop(iplImage,r);
    /*    std::string path = "surfpatch";
	  path.insert(0,"/tmp/"); path.insert(path.length(),".jpg");
	  cvSaveImage(path.c_str(), imgpatch);
	  */
    if (r.width>16 && r.height>16)
	surfDetDes(imgpatch,ips, false, 4, 4, 2, 0.0001f);
    //---------------------Done the surf detection, now calculate the color histogram-----------------------//

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
    CvHistogram* hist;
    float max_value = 0;

    cvCvtColor( tmp, hsv, CV_BGR2HSV );
    cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
    hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
    cvCalcHist( planes, hist, 0, 0 );
    cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );


    cvReleaseImage(&imgpatch);
    cvReleaseImage(&iplImage);
    cvReleaseImage(&h_plane);
    cvReleaseImage(&s_plane);
    cvReleaseImage(&v_plane);
    cvReleaseImage(&hsv);
    cvReleaseImage(&tmp);
    //log("Done get surf!");
    return hist;
}

Vector3 PlanePopOut::PixelRGB2HSV(cogx::Math::ColorRGB rgb)
{
    CvScalar v;	  
    v.val[0] = rgb.r;
    v.val[1] = rgb.g;
    v.val[2] = rgb.b;
    double r,g,b; r=v.val[0]/255.0; g=v.val[1]/255.0; b=v.val[2]/255.0;
    double maxC = b;
    if (maxC < g) maxC = g;
    if (maxC < r) maxC = r;
    double minC = b;
    if (minC > g) minC = g;
    if (minC > r) minC = r;

    double delta = maxC - minC;

    double V = maxC;
    double S = 0;
    double H = 0;

    if (delta == 0)
    {
	H = 0;
	S = 0;
    }
    else
    {
	S = delta / maxC;
	double dR = 60*(maxC - r)/delta + 180;
	double dG = 60*(maxC - g)/delta + 180;
	double dB = 60*(maxC - b)/delta + 180;
	if (r == maxC)
	    H = dB - dG;
	else if (g == maxC)
	    H = 120 + dR - dB;
	else
	    H = 240 + dG - dR;
    }

    if (H<0)
	H+=360;
    if (H>=360)
	H-=360;

    Vector3 hsv; hsv.x=H; hsv.y=S; hsv.z=V;
    return hsv;
}

void PlanePopOut::CollectDensePoints(Video::CameraParameters &cam, PointCloud::SurfacePointSeq points)
{
    CvPoint* points2D = (CvPoint*)malloc( points.size() * sizeof(points2D[0]));
    for (unsigned int i=0; i<SOIPointsSeq.size(); i++)
    {
	PointCloud::SurfacePointSeq DenseSurfacePoints;
	for (unsigned int j=0; j<SOIPointsSeq.at(i).size(); j++)
	{
	    Vector3 v3OneObj = SOIPointsSeq.at(i).at(j).p;
	    Vector2 SOIPointOnImg; SOIPointOnImg = projectPoint(cam, v3OneObj);
	    CvPoint cvp;	cvp.x = (int)SOIPointOnImg.x; cvp.y = (int)SOIPointOnImg.y;
	    points2D[j] = cvp;
	}
	int* hull = (int*)malloc( SOIPointsSeq.at(i).size() * sizeof(hull[0]));

	CvMat pointMat = cvMat( 1, SOIPointsSeq.at(i).size(), CV_32SC2, points2D);
	CvMat* hullMat = cvCreateMat (1, SOIPointsSeq.at(i).size(), CV_32SC2);

	cvConvexHull2(&pointMat, hullMat, CV_CLOCKWISE, 0);
	for (unsigned int k=0; k<points.size(); k++)
	{
	    Vector3 v3OneObj = points.at(k).p;
	    Vector2 PointOnImg; PointOnImg = projectPoint(cam, v3OneObj);
	    CvPoint2D32f p32f; p32f.x = PointOnImg.x; p32f.y = PointOnImg.y;
	    //log("Just before cvPointPolygonTest");
	    int r = cvPointPolygonTest(hullMat, p32f,0);
	    //log("Just after cvPointPolygonTest");
	    if (r>=0) DenseSurfacePoints.push_back(points.at(k));
	}
	cvReleaseMat(&hullMat);
	SOIPointsSeq.at(i)= DenseSurfacePoints;
	free(hull);
    }
    free(points2D);	
}

void PlanePopOut::CalRadiusCenter4BoundingSphere(PointCloud::SurfacePointSeq points, Vector3 &c, double &r)
{
    for (unsigned int i = 0 ; i<points.size() ; i++)
    {
	c = c+points.at(i).p;
    }
    c = c/points.size();
    for (unsigned int i = 0 ; i<points.size() ; i++)
    {
	double d = dist(points.at(i).p,c);
	if (d>r)	r = d;
    }
}

vector<double> PlanePopOut::Hypo2ParaSpace(vector<Vector3> vv3Hypo)
{
    if (vv3Hypo.size() != 3)
    {
	// need three points to determine a plane
	std::cout << " size =  " <<vv3Hypo.size()<< std::endl;
	vector<double> r;
	r.assign(4,0.0);
	return(r);
    }

    double para_a, para_b, para_c, para_d;
    //para_a = ( (vv3Hypo.at(1).y-vv3Hypo.at(0).y)*(vv3Hypo.at(2).z-vv3Hypo.at(0).z)-(vv3Hypo.at(1).z-vv3Hypo.at(0).z)*(vv3Hypo.at(2).y-vv3Hypo.at(0).y) );
    para_a = 0.0;	//parallel with baseline
    para_b = ( (vv3Hypo.at(1).z-vv3Hypo.at(0).z)*(vv3Hypo.at(2).x-vv3Hypo.at(0).x)-(vv3Hypo.at(1).x-vv3Hypo.at(0).x)*(vv3Hypo.at(2).z-vv3Hypo.at(0).z) );
    para_c = ( (vv3Hypo.at(1).x-vv3Hypo.at(0).x)*(vv3Hypo.at(2).y-vv3Hypo.at(0).y)-(vv3Hypo.at(1).y-vv3Hypo.at(0).y)*(vv3Hypo.at(2).x-vv3Hypo.at(0).x) );
    para_d = ( 0-(para_a*vv3Hypo.at(0).x+para_b*vv3Hypo.at(0).y+para_c*vv3Hypo.at(0).z) );
    if (para_c<0)
    {
	para_b = -para_b;
	para_c = -para_c;
	para_d = -para_d;
    }
    double temp = sqrt(para_a*para_a+para_b*para_b+para_c*para_c);

    vector<double> ABCD;
    ABCD.push_back(para_a/temp);
    ABCD.push_back(para_b/temp);
    ABCD.push_back(para_c/temp);
    ABCD.push_back(para_d/temp);

    return ABCD;
}

CvPoint PlanePopOut::ProjectPointOnImage(Vector3 p, const Video::CameraParameters &cam)
{
    cogx::Math::Vector2 p2 = projectPoint(cam, p);
    int x = p2.x;
    int y = p2.y;
    CvPoint re;
    re.x = x; re.y = y;
    return re;
}

Vector3 PlanePopOut::ProjectPointOnPlane(Vector3 p, double A, double B, double C, double D)
{
    if (A*p.x+B*p.y+C*p.z+D == 0)
	return p;
    Vector3 r;
    r.x = ((B*B+C*C)*p.x-A*(B*p.y+C*p.z+D))/(A*A+B*B+C*C);
    r.y = ((A*A+C*C)*p.y-B*(A*p.x+C*p.z+D))/(A*A+B*B+C*C);
    r.z = ((A*A+B*B)*p.z-C*(A*p.x+B*p.y+D))/(A*A+B*B+C*C);
    return r;
}





void PlanePopOut::FindVerticalPlanes(PointCloud::SurfacePointSeq &points, std::vector <int> &labels, double B, double C)
{	
    std::vector<int> candidants;
    for (unsigned int i=0; i<labels.size(); i++)
    {
	if (labels.at(i) == -2) //not belong to the dominant plane cluster
	    candidants.push_back(i);
    }
    unsigned int nPoints = candidants.size();
    int nRansacs = 100;
    int point1 = 0;
    int point2 = 0;
    int point3 = 0;
    Vector3 v3BestMean;
    Vector3 v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;

    for(int i=0; i<nRansacs; i++)
    {
	int nA, nB, nC;
	Vector3 v3Normal;
	//	    do
	//	    {
	nA = rand()%nPoints;
	nB = nA;
	nC = nA;
	while(nB == nA)
	    nB = rand()%nPoints;
	while(nC == nA || nC==nB)
	    nC = rand()%nPoints;
	Vector3 v3CA = points.at(candidants.at(nC)).p - points.at(candidants.at(nA)).p;
	Vector3 v3BA = points.at(candidants.at(nB)).p - points.at(candidants.at(nA)).p;
	v3Normal = cross(v3CA, v3BA);
	if (norm(v3Normal) != 0)
	    normalise(v3Normal);
	else v3Normal = 99999.9*v3Normal;
	//	    } while (fabs(v3Normal.x*B+v3Normal.z*C)>0.01); //the vertical planes

	Vector3 v3Mean = 0.33333333 * (points.at(candidants.at(nA)).p + points.at(candidants.at(nB)).p + points.at(candidants.at(nC)).p);
	double dSumError = 0.0;
	for(unsigned int i=0; i<nPoints; i++)
	{
	    Vector3 v3Diff = points.at(candidants.at(i)).p - v3Mean;
	    double dDistSq = dot(v3Diff, v3Diff);
	    if(dDistSq == 0.0)
		continue;
	    double dNormDist = fabs(dot(v3Diff, v3Normal));
	    if(dNormDist > min_height_of_obj)
		dNormDist = min_height_of_obj;
	    dSumError += dNormDist;
	}
	if(dSumError < dBestDistSquared)
	{
	    dBestDistSquared = dSumError;
	    v3BestMean = v3Mean;
	    v3BestNormal = v3Normal;
	    point1 = nA;
	    point2 = nB;
	    point3 = nC;
	}
    }
    //----------------------- now collect the supposed inlier set-----------------//
    for(unsigned int i=0; i<nPoints; i++)
    {
	Vector3 v3Diff = points.at(candidants.at(i)).p - v3BestMean;
	double dNormDist = fabs(dot(v3Diff, v3BestNormal));
	if(dNormDist < min_height_of_obj)
	    labels.at(candidants.at(i)) = -5; // vertical plane
    }
}

bool PlanePopOut::RANSAC(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    PointCloud::SurfacePointSeq R_points = points;
    unsigned int nPoints = R_points.size();
    if(nPoints < 10)
    {
	//std::cout << "  too few points to calc plane" << std::endl;
	return false;
    };
    int nRansacs = 100;
    int point1 = 0;
    int point2 = 0;
    int point3 = 0;
    Vector3 v3BestMean;
    Vector3 v3BestNormal;
    double dBestDistSquared = 9999999999999999.9;

    for(int i=0; i<nRansacs; i++)
    {
	int nA, nB, nC;
	Vector3 v3Normal;
	do
	{
	    nA = rand()%nPoints;
	    nB = nA;
	    nC = nA;
	    while(nB == nA)
		nB = rand()%nPoints;
	    while(nC == nA || nC==nB)
		nC = rand()%nPoints;
	    Vector3 v3CA = R_points.at(nC).p  - R_points.at(nA).p;
	    Vector3 v3BA = R_points.at(nB).p  - R_points.at(nA).p;
	    v3Normal = cross(v3CA, v3BA);
	    if (norm(v3Normal) != 0)
		normalise(v3Normal);
	    else v3Normal = 99999.9*v3Normal;
	} while ( fabs(v3Normal.x) < 0.00001); //the plane should parallel with the initialisation motion of camera  fabs(v3Normal.x/(dot(v3Normal,v3Normal)+1))>0.01 ||

	Vector3 v3Mean = 0.33333333 * (R_points.at(nA).p + R_points.at(nB).p + R_points.at(nC).p);
	//log("v3Normal (%f, %f, %f)",v3Normal.x,v3Normal.y,v3Normal.z);
	double dSumError = 0.0;
	for(unsigned int i=0; i<nPoints; i++)
	{
	    Vector3 v3Diff = R_points.at(i).p - v3Mean;
	    double dDistSq = dot(v3Diff, v3Diff); 
	    if(dDistSq == 0.0)
		continue;
	    double dNormDist = fabs(dot(v3Diff, v3Normal));//log("dNormDist (%f)",dNormDist);
	    if(dNormDist > min_height_of_obj)
		dNormDist = min_height_of_obj;
	    dSumError += dNormDist;
	}
	if(dSumError < dBestDistSquared)
	{
	    dBestDistSquared = dSumError;
	    v3BestMean = v3Mean;
	    v3BestNormal = v3Normal;
	    point1 = nA;
	    point2 = nB;
	    point3 = nC;
	}
    }
    //log("Index of three points are (%d, %d, %d)",point1,point2,point3);
    ////////////////////////////////use three points to cal plane////
    //log("dBestDistSquared = %f, CurrentBestDistSquared= %f",dBestDistSquared,CurrentBestDistSquared);
    if(dBestDistSquared<CurrentBestDistSquared)
    {
	para_a = ( (R_points.at(point2).p.y-R_points.at(point1).p.y)*(R_points.at(point3).p.z-R_points.at(point1).p.z)-(R_points.at(point2).p.z-R_points.at(point1).p.z)*(R_points.at(point3).p.y-R_points.at(point1).p.y) );
	para_b = ( (R_points.at(point2).p.z-R_points.at(point1).p.z)*(R_points.at(point3).p.x-R_points.at(point1).p.x)-(R_points.at(point2).p.x-R_points.at(point1).p.x)*(R_points.at(point3).p.z-R_points.at(point1).p.z) );
	para_c = ( (R_points.at(point2).p.x-R_points.at(point1).p.x)*(R_points.at(point3).p.y-R_points.at(point1).p.y)-(R_points.at(point2).p.y-R_points.at(point1).p.y)*(R_points.at(point3).p.x-R_points.at(point1).p.x) );
	para_d = ( 0-(para_a*R_points.at(point1).p.x+para_b*R_points.at(point1).p.y+para_c*R_points.at(point1).p.z) );
	CurrentBestDistSquared = dBestDistSquared;
    }
    else
    {
	para_a = A;
	para_b = B;
	para_c = C;
	para_d = D;
    }
    //log("The parameters of the dominate plane are (%f, %f, %f)",para_a, para_b,para_c,para_d);
    /////////////////////////////////end parameters calculation/////////////

    if (para_d<0)
    {
	para_a = -para_a;
	para_b = -para_b;
	para_c = -para_c;
	para_d = -para_d;
    }
    if (para_a*para_a+para_b*para_b+para_c*para_c != 0)
    {
	A = para_a;
	B = para_b;
	C = para_c;
	D = para_d;
    }
    // refine the RANSAC result
    /*
       std::vector < double > vddis;
       vddis.assign(nPoints, 0.0);
       for(unsigned int i=0; i<nPoints; i++)
       {
       Vector3 v3Diff = R_points.at(i).p - v3BestMean;
       double dNormDist = fabs(dot(v3Diff, v3BestNormal));
       vddis.at(i) = dNormDist;
       }
       sort(vddis.begin(),vddis.end());
       double step_density = 0.0;
       double step = 0.001;
       double thre = 0.0;
       double max_density= 0.0;
       for (double d=0.5*min_height_of_obj; d<3.0*min_height_of_obj; d=d+step)
       {
       int num= 0; double tmp = 0;
       for (unsigned int i=0; i<vddis.size(); i++)
       if (vddis.at(i)>d+step) {num = i; break;}
       else	tmp = vddis.at(i)+tmp;
       tmp = num/(d+step); 
       if (tmp>max_density) max_density = tmp;
    //log("the density of window = %f, the num is %d, the previous density is %f", tmp, num, step_density);
    //log("Now the d = %f", d);
    if (step_density == 0.0 || tmp>step_density)	step_density = tmp;
    else
    {
    if ((max_density-tmp)/max_density>1-1/sqrt(3))
    {thre = d+step; break;}
    else step_density = tmp;
    }
    }
    if (thre<min_height_of_obj) thre=min_height_of_obj;
    //log("the refined min_height_of_obj = %f", thre);
    */
    //----------------------- now collect the supposed inlier set-----------------//
    double dmin = 9999.0;
    double dmax = 0.0;

    if (v3BestMean.x != 0 || v3BestMean.y != 0 || v3BestMean.z != 0)
    {
	for(unsigned int i=0; i<nPoints; i++)
	{
	    Vector3 v3Diff = R_points.at(i).p - v3BestMean;
	    double dNormDist = fabs(dot(v3Diff, v3BestNormal));
	    double DominantPlaneHeight=fabs(D)/sqrt(A*A+B*B+C*C);
	    if(dNormDist < min_height_of_obj)
	    {
		labels.at(i) = 0; // dominant plane
		double ddist = dot(R_points.at(i).p,R_points.at(i).p);
		if (ddist > dmax) {dmax = ddist; v3dmax = R_points.at(i).p;}
		else if (ddist < dmin) {dmin = ddist; v3dmin = R_points.at(i).p;}
	    }
	    else
	    {
		/*				double d_parameter = -(A*R_points.at(i).p.x+B*R_points.at(i).p.y+C*R_points.at(i).p.z);
						if (d_parameter > 0 && d_parameter < D && fabs(d_parameter-D) > sqrt(A*A+B*B+C*C)*min_height_of_obj)
						labels.at(i) = -2; // objects
						if (d_parameter > 0 && d_parameter < D && fabs(d_parameter-D) <= sqrt(A*A+B*B+C*C)*min_height_of_obj)
						labels.at(i) = -1; // cannot distingush
						*/

		double distance_point_plane =
		    fabs(A*R_points.at(i).p.x+B*R_points.at(i).p.y+C*R_points.at(i).p.z+D)/ sqrt(A*A+B*B+C*C);
		if (R_points.at(i).p.z>DominantPlaneHeight)	//over the dominant plane
		    labels.at(i) = -2; // objects
		if (distance_point_plane <0.01 && R_points.at(i).p.z>DominantPlaneHeight)
		    labels.at(i) = -1; // cannot distingush
	    }
	}
    }
    return true;
}

void PlanePopOut::SplitPoints(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    std::vector<int> candidants;
    std::vector <int> S_label = labels;
    for (unsigned int i=0; i<S_label.size(); i++)
    {
	if (S_label.at(i) == -2) //not belong to the dominant plane cluster
	    candidants.push_back(i);
    }
    //cout<<"candidants.size() =  "<<candidants.size()<<endl;
    std::vector<int> one_obj;
    objnumber = 1;
    unsigned int points_of_one_object = 0;
    std::stack <int> objstack;
    double split_threshold = Calc_SplitThreshold(points, labels);
    unsigned int obj_number_threshold;
    obj_number_threshold = 100;
    while(!candidants.empty())
    {
	S_label.at(*candidants.begin()) = objnumber;
	objstack.push(*candidants.begin());
	points_of_one_object++;
	candidants.erase(candidants.begin());
	while(!objstack.empty())
	{
	    int seed = objstack.top();
	    objstack.pop();
	    for(std::vector<int>::iterator it=candidants.begin(); it<candidants.end(); it++)
	    {
		if (dist(points.at(seed).p, points.at(*it).p)<split_threshold)
		{
		    S_label.at(*it) = S_label.at(seed);
		    objstack.push(*it);
		    points_of_one_object++;
		    candidants.erase(it);
		}
	    }
	}//cout<<"points_of_one_object =  "<<points_of_one_object<<endl;cout<<"candidants.size() =  "<<candidants.size()<<endl;
	if (points_of_one_object>obj_number_threshold)
	{
	    labels = S_label;
	    objnumber++;
	}
	else
	    S_label = labels;
	points_of_one_object = 0;
    }
    objnumber--;
}

double PlanePopOut::Calc_SplitThreshold(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    double max_x = -99999.0;
    double min_x = 99999.0;
    double max_y = -99999.0;
    double min_y = 99999.0;
    double max_z = -99999.0;
    double min_z = 99999.0;

    for(unsigned int i=0; i<points.size(); i++)
    {
	if (labels.at(i) == -2)
	{
	    Vector3 v3Obj = points.at(i).p;
	    if (v3Obj.x>max_x) max_x = v3Obj.x;
	    if (v3Obj.x<min_x) min_x = v3Obj.x;
	    if (v3Obj.y>max_y) max_y = v3Obj.y;
	    if (v3Obj.y<min_y) min_y = v3Obj.y;
	    if (v3Obj.z>max_z) max_z = v3Obj.z;
	    if (v3Obj.z<min_z) min_z = v3Obj.z;
	}
    }
    return sqrt((max_x-min_x)*(max_x-min_x)+(max_y-min_y)*(max_y-min_y)+(max_z-min_z)*(max_z-min_z))/25;
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

    float wS, wC, wP; //magic weight for SURF matching, color histogram and Size/Position/Pose
    IpPairVec matches;
    wP = 0.1;
    if (obj1.surf.size() == 0 || obj2.surf.size()==0) wS = 0.0;
    else if (obj1.surf.size() < 40 || obj2.surf.size()<40)
    {
	if (obj1.surf.size() < 10 || obj2.surf.size()<10)
	    wS = 0;
	else
	    wS = 0.1;
	getMatches(obj1.surf,obj2.surf,matches);
    }
    else
    {
	wS = 0.3;
	getMatches(obj1.surf,obj2.surf,matches);
    }
    wC = 1.0-wS-wP;
    double dist_histogram = CompareHistKLD(obj1.hist, obj2.hist);
    double surfmacthingRatio;
    if (matches.size()== 0 || obj1.surf.size() == 0 || obj2.surf.size()==0)	surfmacthingRatio = 0.0;
    else 
    {
	if(obj1.surf.size()>obj2.surf.size())
	    surfmacthingRatio =1.0-(float)matches.size()/(float)obj1.surf.size();
	else
	    surfmacthingRatio =1.0- (float)matches.size()/(float)obj2.surf.size();
    }
    /*  
	log("Finish surf matching, there are %d features matched", matches.size());
	log("Finish surf matching, there are %d features in obj1", obj1.surf.size());
	log("Finish surf matching, there are %d features in obj2", obj2.surf.size());

	log("Finish color histogram comparison, the distance is %f",dist_histogram);
	*/   
    double sizeRatio;
    //    int s1 = obj1.pointsInOneSOI.size();
    //    int s2 = obj2.pointsInOneSOI.size();
    double s1, s2; 
    //     s1 = obj1.r; s2 = obj2.r;
    s1 = obj1.pointsInOneSOI.size();  s2 = obj2.pointsInOneSOI.size();
    double smax; if (s1>s2) smax=s1; else smax=s2;
    sizeRatio = 1.0-exp(-(s2-s1)*(s2-s1)*3.14159/smax);
    /*
       if (obj1.r== 0 || obj2.r==0) sizeRatio = 0.0;
       else
       {
       if (obj1.r>obj2.r)	sizeRatio = abs(obj1.r-obj2.r)/obj1.r;
       else sizeRatio = abs(obj1.r-obj2.r)/obj2.r;
       }
       */
    //     log("ratio = %f, %f, %f", wC, wS, wP);
    //     log("dist_histogram, surfmacthingRatio, sizeRatio are %f, %f, %f",dist_histogram, surfmacthingRatio, sizeRatio);

    return 1.0-wS*surfmacthingRatio-wC*abs(dist_histogram)-wP*sizeRatio;
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
    mObjSeq.clear();
    mCenterOfHull.x = mCenterOfHull.y = mCenterOfHull.z = 0.0;
    mConvexHullRadius = 0.0;
    mConvexHullDensity = 0.0;
}
void PlanePopOut::DrawOneCuboid(Vector3 Max, Vector3 Min)
{
    glLineWidth(1);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //////////////////////top/////////////////////////////////////
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(Max.x, Max.y, Max.z);
    glVertex3f(Min.x, Max.y, Max.z);
    glVertex3f(Min.x, Min.y, Max.z);
    glVertex3f(Max.x, Min.y, Max.z);
    glEnd();
    /////////////////////////////bottom///////////////////////
    glBegin(GL_LINE_LOOP);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(Min.x, Min.y, Min.z);
    glVertex3f(Max.x, Min.y, Min.z);
    glVertex3f(Max.x, Max.y, Min.z);
    glVertex3f(Min.x, Max.y, Min.z);
    glEnd();
    //////////////////////verticle lines//////////////////////////
    glBegin(GL_LINES);
    glVertex3f(Min.x, Min.y, Min.z);
    glVertex3f(Min.x, Min.y, Max.z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(Max.x, Min.y, Min.z);
    glVertex3f(Max.x, Min.y, Max.z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(Max.x, Max.y, Min.z);
    glVertex3f(Max.x, Max.y, Max.z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(Min.x, Max.y, Min.z);
    glVertex3f(Min.x, Max.y, Max.z);
    glEnd();

    glDisable(GL_BLEND);
}

void PlanePopOut::DrawCuboids(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    PointCloud::SurfacePointSeq Max;
    PointCloud::SurfacePointSeq Min;
    Vector3 initial_vector;
    initial_vector.x = -9999;
    initial_vector.y = -9999;
    initial_vector.z = -9999;
    PointCloud::SurfacePoint InitialStructure;
    InitialStructure.p = initial_vector;
    InitialStructure.c.r = InitialStructure.c.g = InitialStructure.c.b = 0;
    Max.assign(objnumber, InitialStructure);
    initial_vector.x = 9999;
    initial_vector.y = 9999;
    initial_vector.z = 9999;
    InitialStructure.p = initial_vector;
    Min.assign(objnumber, InitialStructure);

    for(unsigned int i = 0; i<points.size(); i++)
    {
	Vector3 v3Obj = points.at(i).p;
	int label = labels.at(i);
	if (label > 0)
	{
	    if (v3Obj.x>Max.at(label-1).p.x) Max.at(label-1).p.x = v3Obj.x;
	    if (v3Obj.x<Min.at(label-1).p.x) Min.at(label-1).p.x = v3Obj.x;

	    if (v3Obj.y>Max.at(label-1).p.y) Max.at(label-1).p.y = v3Obj.y;
	    if (v3Obj.y<Min.at(label-1).p.y) Min.at(label-1).p.y = v3Obj.y;

	    if (v3Obj.z>Max.at(label-1).p.z) Max.at(label-1).p.z = v3Obj.z;
	    if (v3Obj.z<Min.at(label-1).p.z) Min.at(label-1).p.z = v3Obj.z;
	}
    }
    v3size.clear();
    vdradius.clear();
    for (int i=0; i<objnumber; i++)
    {
	Vector3 s;
	s.x = (Max.at(i).p.x-Min.at(i).p.x)/2;
	s.y = (Max.at(i).p.y-Min.at(i).p.y)/2;
	s.z = (Max.at(i).p.z-Min.at(i).p.z)/2;
	v3size.push_back(s);
	double rad = norm(s);
	vdradius.push_back(rad);
    }
    /*
       for (int i = 0; i<objnumber; i++)
       {
       DrawOneCuboid(Max.at(i),Min.at(i));
       }
       */
    Max.clear();
    Min.clear();
}

void PlanePopOut::DrawWireSphere(Vector3 center, double radius)
{
    glColor3f(1.0,1.0,1.0);
    glTranslatef(center.x, center.y, center.z);
    glutWireSphere(radius,10,10);
    glTranslatef(-center.x, -center.y, -center.z);
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

Vector3 PlanePopOut::GetAffineTransVec(Vector3 v3p) // translation vector from p to original point
{
    Matrix33 m33 = GetAffineRotMatrix();
    return -(m33*v3p);
}

inline Vector3 PlanePopOut::AffineTrans(Matrix33 m33, Vector3 v3)
{
    return m33*v3;
}

void PlanePopOut::ConvexHullOfPlane(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    CvPoint* points2D = (CvPoint*)malloc( points.size() * sizeof(points2D[0]));
    vector<Vector3> PlanePoints3D;
    Matrix33 AffineM33 = GetAffineRotMatrix();
    int j = 0;

    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* ptseq = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvContour),
	    sizeof(CvPoint), storage );


    for(unsigned int i = 0; i<points.size(); i++)
    {
	Vector3 v3Obj = points.at(i).p;
	int label = labels.at(i);
	if (label == 0) // collect points seq for drawing convex hull of the dominant plane
	{
	    CvPoint cvp;
	    Vector3 v3AfterAffine = AffineTrans(AffineM33, v3Obj);
	    cvp.x =100.0*v3AfterAffine.x; cvp.y =100.0*v3AfterAffine.y;
	    points2D[j] = cvp;
	    cvSeqPush( ptseq, &cvp );
	    j++;
	    PlanePoints3D.push_back(v3Obj);
	}
    }
    // calculate convex hull
    if (j>0)
    {
	//cout<<"2d points number ="<<j-1<<endl;
	int* hull = (int*)malloc( (j-1) * sizeof(hull[0]));
	CvSeq* cvhull;


	CvMat pointMat = cvMat( 1, j-1, CV_32SC2, points2D);
	CvMat hullMat = cvMat( 1, j-1, CV_32SC1, hull);
	cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);
	cvhull = cvConvexHull2( ptseq, 0, CV_CLOCKWISE, 1);


	//draw the hull
	if (hullMat.cols != 0)
	{
	    //cout<<"points in the hull ="<<hullMat.cols<<endl;
	    // 			if (mbDrawWire)	glBegin(GL_LINE_LOOP); else glBegin(GL_POLYGON);
	    // 			glColor3f(1.0,1.0,1.0);
	    Vector3 v3OnPlane;
	    for (int i = 0; i<hullMat.cols; i++)
	    {
		v3OnPlane = ProjectOnDominantPlane(PlanePoints3D.at(hull[i]));
		mConvexHullPoints.push_back(v3OnPlane);
		mCenterOfHull += v3OnPlane;
	    }
	    mConvexHullDensity = PlanePoints3D.size() / fabs(cvContourArea(cvhull));//cout<<"mConvexHullDensity = "<<mConvexHullDensity<<endl;
	    mCenterOfHull /= hullMat.cols;
	    mConvexHullRadius = sqrt((v3OnPlane.x-mCenterOfHull.x)*(v3OnPlane.x-mCenterOfHull.x)+(v3OnPlane.y-mCenterOfHull.y)*(v3OnPlane.y-mCenterOfHull.y)+(v3OnPlane.z-mCenterOfHull.z)*(v3OnPlane.z-mCenterOfHull.z));
	    //cout<<"mConvexHullRadius = "<<mConvexHullRadius<<endl;
	    // 			glEnd();
	}
	free( hull );
	cvClearSeq(cvhull);
    }
    cvClearMemStorage( storage );
    cvReleaseMemStorage(&storage);
    cvClearSeq(ptseq);
    PlanePoints3D.clear();
    free( points2D );
}

void PlanePopOut::DrawOnePrism(vector <Vector3> ppSeq, double hei, Vector3& v3c)
{
    //just the prism up to the plane will be calculated

    v3c.z = v3c.z+hei/2;
    VisionData::OneObj OObj;
    for (unsigned int i = 0; i<ppSeq.size(); i++)
    {
	Vector3 v;
	v.x =ppSeq.at(i).x;
	v.y =ppSeq.at(i).y;
	v.z =ppSeq.at(i).z+hei;
	OObj.pPlane.push_back(ppSeq.at(i)); OObj.pTop.push_back(v);
    }
    mObjSeq.push_back(OObj);

    /*	double dd = D - hei*sqrt(A*A+B*B+C*C);
	double half_dd = D- 0.5*hei*sqrt(A*A+B*B+C*C);
	Vector3 v3core;
	v3core.x =((C*C+B*B)*v3c.x-A*(C*v3c.z+B*v3c.y+half_dd))/(A*A+B*B+C*C);
	v3core.y =((A*A+C*C)*v3c.y-B*(A*v3c.x+C*v3c.z+half_dd))/(A*A+B*B+C*C);
	v3core.z =((A*A+B*B)*v3c.z-C*(A*v3c.x+B*v3c.y+half_dd))/(A*A+B*B+C*C);
	v3c = v3core;
	vector <Vector3> pphSeq;
	VisionData::OneObj OObj;
	for (unsigned int i = 0; i<ppSeq.size(); i++)
	{
	Vector3 v;
	v.x =((C*C+B*B)*ppSeq.at(i).x-A*(C*ppSeq.at(i).z+B*ppSeq.at(i).y+dd))/(A*A+B*B+C*C);
	v.y =((A*A+C*C)*ppSeq.at(i).y-B*(A*ppSeq.at(i).x+C*ppSeq.at(i).z+dd))/(A*A+B*B+C*C);
	v.z =((A*A+B*B)*ppSeq.at(i).z-C*(A*ppSeq.at(i).x+B*ppSeq.at(i).y+dd))/(A*A+B*B+C*C);
	pphSeq.push_back(v);
	OObj.pPlane.push_back(ppSeq.at(i)); OObj.pTop.push_back(v);
	}
	mObjSeq.push_back(OObj);
	*/
    /*
       glBegin(GL_POLYGON);
       glColor3f(1.0,1.0,1.0);
       for (unsigned int i = 0; i<ppSeq.size(); i++)
       glVertex3f(ppSeq.at(i).x,ppSeq.at(i).y,ppSeq.at(i).z);
       glEnd();
       glBegin(GL_POLYGON);
       for (unsigned int i = 0; i<ppSeq.size(); i++)
       glVertex3f(pphSeq.at(i).x,pphSeq.at(i).y,pphSeq.at(i).z);
       glEnd();
       for (unsigned int i = 0; i<ppSeq.size(); i++)
       {
       glBegin(GL_LINES);
       glVertex3f(ppSeq.at(i).x,ppSeq.at(i).y,ppSeq.at(i).z);
       glVertex3f(pphSeq.at(i).x,pphSeq.at(i).y,pphSeq.at(i).z);
       glEnd();
       }
       */
}

void PlanePopOut::BoundingPrism(PointCloud::SurfacePointSeq &pointsN, std::vector <int> &labels)
{
    CvPoint* points2D = (CvPoint*)malloc( pointsN.size() * sizeof(points2D[0]));

    Matrix33 AffineM33 = GetAffineRotMatrix();
    Matrix33 AffineM33_1;
    inverse(AffineM33,AffineM33_1);
    vector < CvPoint* > objSeq;
    objSeq.assign(objnumber,points2D);
    vector < int > index;
    index.assign(objnumber,0);
    vector < double > height;
    height.assign(objnumber,0.0);
    vector < vector<Vector3> > PlanePoints3DSeq;
    Vector3 v3init; v3init.x = v3init.y = v3init.z =0.0;
    vector<Vector3> vv3init; vv3init.assign(1,v3init);
    PlanePoints3DSeq.assign(objnumber, vv3init);


    for(unsigned int i = 0; i<pointsN.size(); i++)
    {
	Vector3 v3Obj = pointsN.at(i).p;
	int label = labels.at(i);
	if (label < 1)	continue;
	CvPoint cvp;
	Vector3 v3AfterAffine = AffineTrans(AffineM33, v3Obj);
	cvp.x =1000.0*v3AfterAffine.x; cvp.y =1000.0*v3AfterAffine.y;
	objSeq.at(label-1)[index.at(label-1)] = cvp;
	PlanePoints3DSeq.at(label-1).push_back(v3Obj);
	index.at(label-1)++;
	if (fabs(A*v3Obj.x+B*v3Obj.y+C*v3Obj.z+D)/sqrt(A*A+B*B+C*C) > height.at(label-1))
	    height.at(label-1) = fabs(A*v3Obj.x+B*v3Obj.y+C*v3Obj.z+D)/sqrt(A*A+B*B+C*C);
    }
    // calculate convex hull
    if (index.at(0)>0)
    {
	int* hull = (int*)malloc( pointsN.size() * sizeof(hull[0]));
	Vector3 temp_v; temp_v.x = temp_v.y = temp_v.z = 0.0;
	v3center.assign(objnumber, temp_v);
	for (int i = 0; i < objnumber; i++)
	{

	    CvMat pointMat = cvMat( 1, index.at(i)-1, CV_32SC2, objSeq.at(i));
	    CvMat hullMat = cvMat( 1, index.at(i)-1, CV_32SC1, hull);
	    cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);
	    //calculate convex hull points on the plane
	    std::vector <Vector3> v3OnPlane;
	    v3OnPlane.assign(hullMat.cols, v3init);
	    for (int j = 0; j<hullMat.cols; j++)
		v3OnPlane.at(j) =ProjectOnDominantPlane(PlanePoints3DSeq.at(i).at(hull[j]+1));

	    for (unsigned int k = 0; k<v3OnPlane.size(); k++)
	    {
		v3center.at(i) = v3center.at(i) + v3OnPlane.at(k);
	    }
	    v3center.at(i) = v3center.at(i)/ v3OnPlane.size();

	    DrawOnePrism(v3OnPlane, height.at(i), v3center.at(i));


	    v3OnPlane.clear();
	}
	free( hull );
    }
    free( points2D );
}

void PlanePopOut::BoundingSphere(PointCloud::SurfacePointSeq &points, std::vector <int> &labels)
{
    PointCloud::SurfacePointSeq center;
    Vector3 initial_vector;
    initial_vector.x = 0;
    initial_vector.y = 0;
    initial_vector.z = 0;
    PointCloud::SurfacePoint InitialStructure;
    InitialStructure.p = initial_vector;
    center.assign(objnumber,InitialStructure);

    for (unsigned int i = 0 ; i<v3center.size() ; i++)
    {
	center.at(i).p = v3center.at(i);
    }

    std::vector<double> radius_world;
    radius_world.assign(objnumber,0);
    PointCloud::SurfacePointSeq pointsInOneSOI;
    SOIPointsSeq.clear();
    SOIPointsSeq.assign(objnumber, pointsInOneSOI);
    BGPointsSeq.clear();
    BGPointsSeq.assign(objnumber, pointsInOneSOI);
    EQPointsSeq.clear();
    EQPointsSeq.assign(objnumber, pointsInOneSOI);

    ////////////////////calculte radius in the real world//////////////////
    for(unsigned int i = 0; i<points.size(); i++)
    {
	Vector3 v3Obj = points.at(i).p;
	int label = labels.at(i);
	if (label > 0 && dist(v3Obj,center.at(label-1).p) > radius_world.at(label-1))
	    radius_world.at(label-1) = dist(v3Obj,center.at(label-1).p);
    }
    /*		//vdradius.clear();
		for (int i=0; i<objnumber; i++)
		{
    //vdradius.push_back(radius_world.at(i));
    cout<<"in Bounding box, radius of "<<i<<" object is "<<vdradius.at(i)<<endl;
    cout<<"world radius of "<<i<<" object is "<<radius_world.at(i)<<endl;
    }

*/
    for (int i = 0; i<objnumber; i++)
    {
	//if (mbDrawWire)	DrawWireSphere(center.at(i).p,radius_world.at(i));
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

	    if (label == 0 && dist(Point_DP,Center_DP) < Upper_BG*radius_world.at(i) && dist(Point_DP,Center_DP) > Lower_BG*vdradius.at(i)) //BG nearby also required
		BGPointsSeq.at(i).push_back(PushStructure);

	}
    }

    center.clear();
    radius_world.clear();
    pointsInOneSOI.clear();
}

void PlanePopOut::newVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
    VisualObjectPtr vis_obj = getMemoryEntry<VisionData::VisualObject>(_wmc.address);

    if (vis_obj->model.get()) {
	VisionData::VertexSeq ver_seq = vis_obj->model->vertices;
	vlines.push_back(ver_seq.at(0).normal);
	vlineConfidence.push_back(vis_obj->detectionConfidence);
    }
}

void PlanePopOut::deleteVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
    vlines.clear();
    vlineConfidence.clear();
}

void PlanePopOut::RefinePlaneEstimation(vector <Vector3> lines)
{
    if (lines.empty())	return;
    Vector3 PlaneNormal;
    PlaneNormal.x = A; PlaneNormal.y = B; PlaneNormal.z = C;
    normalise(PlaneNormal);
    double newD = PlaneNormal.x *D/A;
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
