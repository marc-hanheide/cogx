/**
 * @author Michael Zillich
 * @date Sept 2011
 */

#include <math.h>
#include <time.h>
#include <algorithm>
#include <stack>
#include <vector>
#include <fstream>
#include <boost/interprocess/sync/scoped_lock.hpp>
// #ifdef __APPLE__
// #include <GL/glut.h> //nah: glut is installed via MacPorts and thus preferred
// #else
// //#include <GL/freeglut.h>
// #endif
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <cogxmath.h>
#include <VideoUtils.h>
#include <castutils/Timers.hpp>
#include "../VisionUtils.h"
#include "VideoUtils.h"
#include "StereoCamera.h"
#include "PlanePopOut.h"


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

// 0 send sparse points, 1 send dense points (recollect them after the segmentation)
#define SendDensePoints  1

// the similarity of 2 SOIs higher than this will make the system treat these 2 SOI as the sam one
#define Treshold_Comp2SOI	0.75

// image size (i.e. width) of the point cloud and colour image to obtain from the
// poin cloud server
#define PPO_POINTCLOUD_WIdTH 320
#define PPO_IMAGE_WIDTH      640

// minimum number of points in a point cloud to do any processing
#define PPO_MIN_POINTCLOUD_SIZE 100

// the label reserved for the dominant plane
#define PPO_LABEL_PLANE 0

// number of bins for colour histograms
#define PPO_COLHIST_H_BINS 16
#define PPO_COLHIST_S_BINS 8

// minimium match probability of 2 SOIs to consider them equal
#define PPO_MIN_MATCH_PROB 0.75

namespace cast
{
using namespace std;
using namespace PointCloud;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;
using namespace cdl;

static inline bool PointIsSane(const PointCloud::SurfacePoint &p)
{
  return p.p.x > -10000. && p.p.x < 10000. &&
	 p.p.y > -10000. && p.p.y < 10000. &&
	 p.p.z > -10000. && p.p.z < 10000.;
}

/**
 * Convert points from CAST format to PCL format.
 * NOTE: this should go to VisionUtils. But that introduces a PCL dependency in VisionUtils,
 * which affects all source files including VisionUtils.h. So leave it here for now.
 */
static inline void ConvertSurfacePoints2PCLCloud(const vector<PointCloud::SurfacePoint> &points,
	pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud,
	int width, int height)
{
    if((int)points.size() < width*height)
	throw runtime_error(cast::exceptionMessage(__HERE__,
		    "need %d x %d points, have %d", width, height, (int)points.size()));

    pcl_cloud.width = width;
    pcl_cloud.height = height;
    pcl_cloud.points.resize(width*height);
    for(size_t i = 0; i < pcl_cloud.points.size(); i++)
    {
	RGBValue color;
	if(PointIsSane(points[i]))
	{
	    pcl_cloud.points[i].x = (float) points[i].p.x;
	    pcl_cloud.points[i].y = (float) points[i].p.y;
	    pcl_cloud.points[i].z = (float) points[i].p.z;
	    color.r = points[i].c.r;
	    color.g = points[i].c.g;
	    color.b = points[i].c.b;
	    pcl_cloud.points[i].rgb = color.float_value;
	}
	else
	{
	    pcl_cloud.points[i].x = 0.;
	    pcl_cloud.points[i].y = 0.;
	    pcl_cloud.points[i].z = 0.;
	    color.r = 0;
	    color.g = 0;
	    color.b = 0;
	    pcl_cloud.points[i].rgb = color.float_value;
	}
    }
}

/**
 * compare two color histogram using Kullback–Leibler divergence
 */
static double CompareHistKLD(CvHistogram* h1, CvHistogram* h2)
{
    int h_bins = PPO_COLHIST_H_BINS, s_bins = PPO_COLHIST_S_BINS;
    double KLD = 0.0;

    for(int h = 0; h < h_bins; h++)
    {
	for(int s = 0; s < s_bins; s++)
	{
	    double v1 = cvQueryHistValue_2D( h1, h, s );
	    double v2 = cvQueryHistValue_2D( h2, h, s );
	    if (!iszero(v1) && !iszero(v2))
		KLD = (log10(v1/v2)*v1 + log10(v2/v1)*v2)/2 + KLD;
	}
    }
    return KLD;
}

void PlanePopOut::PlaneEntry::init(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
	pcl::PointIndices::Ptr planepoints, pcl::ModelCoefficients::Ptr pcl_domplane,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tablehull)
{
  if(pcl_cloud && planepoints && tablehull && pcl_domplane && pcl_domplane->values.size() >= 4)
  {
    valid = true;
    plane = plane3(pcl_domplane->values[0],
	    pcl_domplane->values[1],
	    pcl_domplane->values[2],
	    pcl_domplane->values[3]);
    normalisePlane(plane);        
    for (size_t i = 0; i < planepoints->indices.size(); i++)
    {
      int index = planepoints->indices[i];
      SurfacePoint p;
      p.p = vector3(pcl_cloud->points[index].x, pcl_cloud->points[index].y, pcl_cloud->points[index].z);
      p.c.r = pcl_cloud->points[index].r;
      p.c.g = pcl_cloud->points[index].g;
      p.c.b = pcl_cloud->points[index].b;
      planePoints.push_back(p);
    }
    for (size_t i = 0; i < tablehull->points.size(); i++)
      hullPoints.push_back(vector3(tablehull->points[i].x, tablehull->points[i].y, tablehull->points[i].z));
  }
  else
  {
    valid = false;
  }
}

void PlanePopOut::SOIEntry::init(const PlaneEntry &domPlane)
{
    // bounding sphere
    setZero(boundingSphere.pos);
    for (size_t i = 0; i < points.size(); i++)
      boundingSphere.pos += points[i].p;
    boundingSphere.pos /= (double)points.size();
    boundingSphere.rad = 0.;
    for (size_t i = 0; i < points.size(); i++)
      boundingSphere.rad = max(boundingSphere.rad, dist(boundingSphere.pos, points[i].p));

    // bounding box
    // TODO: actually we could get a better size estimate for the box
    // for now leave it at that
    boundingBox.pos = boundingSphere.pos;
    boundingBox.size.x = boundingSphere.rad;
    boundingBox.size.y = boundingSphere.rad;
    boundingBox.size.z = boundingSphere.rad;

    // background points
    for (size_t i = 0; i < domPlane.planePoints.size(); i++)
      if(pointInsideSphere(boundingSphere, domPlane.planePoints[i].p))
        BGpoints.push_back(domPlane.planePoints[i]);

    calcHistogram();

    // this is a first frame in which the SOI has been seen
    numStableFrames = 1;
}

/**
 * returns a comparison value between 0 and 1, with 0 being a perfect match
 */
double PlanePopOut::SOIEntry::compare(const PlanePopOut::SOIEntry &other)
{
    // weights for colour histogram, size and position match
    double wC = 0.2, wS = 0.2, wP = 0.6;
    // just make sure they sum to 1
    double s = wC + wS + wP;
    wC /= s;
    wS /= s;
    wP /= s;
    double distHistogram = abs(CompareHistKLD(hist, other.hist));
    double distSize = abs(boundingSphere.rad - other.boundingSphere.rad) /
        max(boundingSphere.rad, other.boundingSphere.rad);
    double distPos = dist(boundingSphere.pos, other.boundingSphere.pos)/(2.*boundingSphere.rad);
    return wC*distHistogram + wS*distSize + wP*distPos;
}

/**
 * returns a matching probability between two SOIs
 */
double PlanePopOut::SOIEntry::matchProbability(const PlanePopOut::SOIEntry &other)
{
    // NOTE: that this is of course not a proper probability
    // better probabilistic matches could be implemented
    // but we leave as is for the moment
    return 1.0 - compare(other);
}

PlanePopOut::SOIEntry& PlanePopOut::SOIEntry::operator= (const PlanePopOut::SOIEntry &other)
{
    boundingSphere = other.boundingSphere;
    boundingBox = other.boundingBox;
    points = other.points;
    BGpoints = other.BGpoints;
    WMId = other.WMId;
    numFramesNotSeen = other.numFramesNotSeen;
    numStableFrames = other.numStableFrames;
    hasMatch = other.hasMatch;
    // TODO: note that we should not need to delete hist, all hists are the same size anyway
    cvReleaseHist(&hist);
    if (other.hist != 0) 
        cvCopyHist(other.hist, &hist);
    dispColor = other.dispColor;
    return *this;
}

void PlanePopOut::SOIEntry::updateFrom(const PlanePopOut::SOIEntry &other)
{
    boundingSphere = other.boundingSphere;
    boundingBox = other.boundingBox;
    points = other.points;
    BGpoints = other.BGpoints;
    // TODO: note that we should not need to delete hist, all hists are the same size anyway
    cvReleaseHist(&hist);
    if (other.hist != 0) 
        cvCopyHist(other.hist, &hist);
    // this SOI has just been seen
    numFramesNotSeen = 0;
    numStableFrames++;
    // and dispColor are not copied
    // of course WMId is also not changed
}

void PlanePopOut::SOIEntry::establishMatch(PlanePopOut::SOIEntry &other)
{
    updateFrom(other);
    // these two SOIs are matched now
    hasMatch = true;
    other.hasMatch = true;
}

SOIPtr PlanePopOut::SOIEntry::createWMSOI(ManagedComponent *comp)
{
    VisionData::SOIPtr obs = new VisionData::SOI;
    obs->sourceId = comp->getComponentID();
    obs->status = 0;
    obs->boundingSphere = boundingSphere;
    obs->boundingBox = boundingBox;
    obs->time = comp->getCASTTime();
    obs->points = points;
    obs->BGpoints = BGpoints;
    // TODO: for now do not set EQpoints, don't know what thes are really
    return obs;
}

void PlanePopOut::SOIEntry::calcHistogram()
{
    IplImage* tmp = cvCreateImage(cvSize(1, points.size()), 8, 3);
    for (size_t i = 0; i < points.size(); i++)
    {
	CvScalar v;
	v.val[0] = points[i].c.b;
	v.val[1] = points[i].c.g;
	v.val[2] = points[i].c.r;
	cvSet2D(tmp, i, 0, v);
    }
    IplImage* h_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
    IplImage* s_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
    IplImage* v_plane = cvCreateImage( cvGetSize(tmp), 8, 1 );
    IplImage* planes[] = { h_plane, s_plane };
    IplImage* hsv = cvCreateImage( cvGetSize(tmp), 8, 3 );
    int h_bins = PPO_COLHIST_H_BINS, s_bins = PPO_COLHIST_S_BINS;
    int hist_size[] = {h_bins, s_bins};
    /* hue varies from 0 (~0 deg red) to 180 (~360 deg red again) */
    float h_ranges[] = { 0, 180 };
    /* saturation varies from 0 (black-gray-white) to 255 (pure spectrum color) */
    float s_ranges[] = { 0, 255 };
    float* ranges[] = { h_ranges, s_ranges };

    cvCvtColor( tmp, hsv, CV_BGR2HSV );
    cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
    hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
    cvCalcHist( planes, hist, 0, 0 );
    cvNormalizeHist( hist, 1.0 );

    cvReleaseImage(&h_plane);
    cvReleaseImage(&s_plane);
    cvReleaseImage(&v_plane);
    cvReleaseImage(&hsv);
    cvReleaseImage(&tmp);
}

int PlanePopOut::m_componentCount = 0;
IceUtil::Mutex PlanePopOut::m_planePopoutMutex;

PlanePopOut::PlanePopOut()
{
    iplDispImage = 0;

    // TODO: these settings should come from CAST config.
    pclA::PlanePopout::Parameter par;
    // Minimum and maximum object height (default 0.005m, and 0.7m))
    // for origin in camera
    //par.minObjectHeight = 0.005;
    //par.maxObjectHeight = 1.;
    // for origin in robot ego
    par.minObjectHeight = -1.;
    par.maxObjectHeight = -0.05;
    //par.thrSacDistance = 0.03;
    // filter points depending on z value (default = 0.5m - 1.5m)
    par.minZ = 0.3;
    par.maxZ = 1.5;
    m_planePopout = new pclA::PlanePopout(par);
    m_componentCount++;
}

PlanePopOut::~PlanePopOut()
{
    if (m_planePopout)
	delete m_planePopout;
    cvReleaseImage(&iplDispImage);
    m_componentCount--;
}


void PlanePopOut::configure(const map<string,string> & _config)
{
    // first let the base classes configure themselves
    configureServerCommunication(_config);

    map<string,string>::const_iterator it;

    camId = 0;
    doDisplay = false;
    AgonalTime = 10;
    StableTime = 2;

    if((it = _config.find("--camId")) != _config.end())
    {
	istringstream str(it->second);
	str >> camId;
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

    m_bWriteSoisToWm = true;
    if((it = _config.find("--generate-sois")) != _config.end())
    {
	m_bWriteSoisToWm = ! (it->second == "0" || it->second == "false" || it->second == "no");
    }
    println("%s write SOIs to WM", m_bWriteSoisToWm ? "WILL" : "WILL NOT"); 

    if(doDisplay)
    {
	// startup window size
	int winWidth = 640, winHeight = 480;
	cv::Mat intrinsic = (cv::Mat_<double>(3,3) << winWidth,0,winWidth/2, 0,winWidth,winHeight/2, 0,0,1);
	cv::Mat R = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,1);
	cv::Mat t = (cv::Mat_<double>(3,1) << 0,0,0);
	cv::Vec3d rotCenter(0,0,0.4);
	// Initialize 3D render engine 
	tgRenderer = new TGThread::TomGineThread(winWidth, winHeight);
	tgRenderer->SetParameter(intrinsic);
	tgRenderer->SetCamera(R, t, rotCenter);
	tgRenderer->SetCoordinateFrame(0.5);
    }

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
#define ID_PART_3D_SOI     "SOIs"
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
    startV11N();
#endif

    // @author: mmarko
    // we want to receive GetStableSoisCommand-s
    addChangeFilter(createLocalTypeFilter<VisionData::GetStableSoisCommand>(cdl::ADD),
	    new MemberFunctionChangeReceiver<PlanePopOut>(this,
		&PlanePopOut::onAdd_GetStableSoisCommand));
}

#ifdef FEAT_VISUALIZATION
void PlanePopOut::startV11N()
{
    m_bSendPoints = false;
    m_bSendPlaneGrid = true;
    m_bSendImage = true;
    m_bSendSois = true;
    m_bColorByLabel = false;
    m_display.connectIceClient(*this);
    m_display.setClientData(this);
    m_display.installEventReceiver();

    Visualization::ActionInfo act;
    string cid = " (" + getComponentID() + ")";

    //act.id = IDC_POPOUT_LABEL_COLOR;
    //act.label = "Color by label" + cid;
    //act.iconLabel = "Color";
    //act.iconSvg = "text:Co";
    //act.checkable = true;
    //m_display.addAction(ID_OBJECT_3D, act);

    act.id = guiid(IDC_POPOUT_POINTS);
    act.label = "Toggle Update 3D Points" + cid;
    act.iconLabel = "3D Points";
    act.iconSvg = "text:Pts";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = guiid(IDC_POPOUT_SOIS);
    act.label = "Toggle Update SOIs" + cid;
    act.iconLabel = "SOIs";
    act.iconSvg = "text:Soi";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = guiid(IDC_POPOUT_PLANEGRID);
    act.label = "Toggle Update Convex Hull of Principal Plane" + cid;
    act.iconLabel = "Plane Hull";
    act.iconSvg = "text:Hul";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_3D, act);

    act.id = guiid(IDC_POPOUT_IMAGE);
    act.label = "Toggle Update Image" + cid;
    act.iconLabel = "Image";
    act.iconSvg = "text:Img";
    act.checkable = true;
    m_display.addAction(ID_OBJECT_IMAGE, act);

    // Object displays (m_bXX) are set to off: we need to create dummy display objects
    // on the server so that we can activate displays through GUI
    ostringstream ss;
    ss <<  "function render()\nend\n"
	<< "setCamera('ppo.robot.head', -1.0, 0, 3.0, 1, 0, -1, 0, 0, 1)\n"
	<< "setCamera('ppo.robot.front', 4.0, 0, 4.0, -1, 0, -1, 0, 0, 1)\n"
	<< "setCamera('ppo.points.top', 0, 0, 4.0, 0, 0, -1, -1, 0, 0)\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_POINTS), ss.str());

    //Video::Image image;
    //m_display.setImage(ID_OBJECT_IMAGE, image);

    ss.str("");
    ss  << "showSois = " << (m_bSendSois ? "true" : "false") << "\n"
	<< "function render()\nend\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_SOI), ss.str());

    ss.str("");
    ss  << "showPoints = " << (m_bSendPoints ? "true" : "false") << "\n"
	<< "function render()\nend\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_POINTS), ss.str());

    ss.str("");
    ss  << "showPlaneGrid = " << (m_bSendPlaneGrid ? "true" : "false") << "\n"
	<< "function render()\nend\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_PLANE), ss.str());
}

void PlanePopOut::CDisplayClient::handleEvent(const Visualization::TEvent &event)
{
    if (!pPopout) return;
    pPopout->println("Got event: %s", event.sourceId.c_str());
    if (event.sourceId == pPopout->guiid(IDC_POPOUT_POINTS)) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendPoints = false;
	else pPopout->m_bSendPoints = true;
	pPopout->m_display.setLuaGlObject(ID_OBJECT_3D, pPopout->guiid(ID_PART_3D_POINTS), 
		pPopout->m_bSendPoints ? "showPoints=true" : "showPoints=false");
    }
    else if (event.sourceId == pPopout->guiid(IDC_POPOUT_PLANEGRID)) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendPlaneGrid = false;
	else pPopout->m_bSendPlaneGrid = true;
	pPopout->m_display.setLuaGlObject(ID_OBJECT_3D, pPopout->guiid(ID_PART_3D_PLANE), 
		pPopout->m_bSendPlaneGrid ? "showPlaneGrid=true" : "showPlaneGrid=false");
    }
    //else if (event.sourceId == IDC_POPOUT_LABEL_COLOR) {
    //    if (event.data == "0" || event.data=="") pPopout->m_bColorByLabel = false;
    //    else pPopout->m_bColorByLabel = true;
    //}
    else if (event.sourceId == pPopout->guiid(IDC_POPOUT_IMAGE)) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendImage = false;
	else pPopout->m_bSendImage = true;
    }
    else if (event.sourceId == pPopout->guiid(IDC_POPOUT_SOIS)) {
	if (event.data == "0" || event.data=="") pPopout->m_bSendSois = false;
	else pPopout->m_bSendSois = true;
	pPopout->m_display.setLuaGlObject(ID_OBJECT_3D, pPopout->guiid(ID_PART_3D_SOI), 
		pPopout->m_bSendSois ? "showSois=true" : "showSois=false");
    }
}

string PlanePopOut::CDisplayClient::getControlState(const string& ctrlId)
{
    if (!pPopout) return "";
    pPopout->println("Get control state: %s", ctrlId.c_str());
    if (ctrlId == pPopout->guiid(IDC_POPOUT_POINTS)) {
	if (pPopout->m_bSendPoints) return "2";
	else return "0";
    }
    if (ctrlId == pPopout->guiid(IDC_POPOUT_PLANEGRID)) {
	if (pPopout->m_bSendPlaneGrid) return "2";
	else return "0";
    }
    //if (ctrlId == IDC_POPOUT_LABEL_COLOR) {
    //    if (pPopout->m_bColorByLabel) return "2";
    //    else return "0";
    //}
    if (ctrlId == pPopout->guiid(IDC_POPOUT_IMAGE)) {
	if (pPopout->m_bSendImage) return "2";
	else return "0";
    }
    if (ctrlId == pPopout->guiid(IDC_POPOUT_SOIS)) {
	if (pPopout->m_bSendSois) return "2";
	else return "0";
    }
    return "";
}

void PlanePopOut::SendImage()
{
    if(iplDispImage != 0) {

	castutils::CMilliTimer tm(true);

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

	long size = iplDispImage->imageSize;
	m_display.setImage(guiid(ID_OBJECT_IMAGE), iplDispImage);

	long long t2 = tm.elapsed();

	if (1) {
	    ostringstream str;
	    str << "<h3>Plane popout - SendImage</h3>";
	    //str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	    str << "Size: " << size << " bytes.<br>";
	    str << "Sent: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	    m_display.setHtml("INFO", "log.PPO.SendImage", str.str());
	}
    } else {
	if (1) {
	    ostringstream str;
	    str << "<h3>Plane popout - SendImage</h3>";
	    str << "no image to send<br>";
	    m_display.setHtml("INFO", "log.PPO.SendImage", str.str());
	}
    }
}

/**
 * TODO: right now color by labels is ignored
 */
void PlanePopOut::SendPoints(bool bColorByLabels)
{
    castutils::CMilliTimer tm(true);

    int pointCnt = 0;
    std::ostringstream str;
    str.unsetf(ios::floatfield); // unset floatfield
    str.precision(3); // set the _maximum_ precision
    str << "function render()\n";
    str << "if not showPoints then return end\n";
    str << "glPointSize(2)\nglBegin(GL_POINTS)\n";
    str << "v=glVertex\nc=glColor\n";

#define FCHN(x) (float)x/255.0
    // (Approximately) Limit the number of points sent to the display server
    const double MAX_IN_PLANE = 2000.0;
    const double MAX_IN_SOI = 300.0;
    if (dominantPlane.valid) {
	str << "c("
	    << FCHN(dominantPlane.dispColor.r) << ","
	    << FCHN(dominantPlane.dispColor.g) << ","
	    << FCHN(dominantPlane.dispColor.b) << ")\n";
	
	int pctLimit = floor(0.5 + 1000 * (MAX_IN_PLANE / dominantPlane.planePoints.size()));
	if (pctLimit < 1) pctLimit = 1;
	
	for(size_t i = 0; i < dominantPlane.planePoints.size(); i++)
	{
	    if (rand() % 1000 > pctLimit)
		continue;
	    str << "v("
		<< dominantPlane.planePoints[i].p.x << ","
		<< dominantPlane.planePoints[i].p.y << ","
		<< dominantPlane.planePoints[i].p.z << ")\n";
	    pointCnt++;
	}
    }
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
    {
	str << "c(" 
	    << FCHN(it->dispColor.r) << ","
	    << FCHN(it->dispColor.g) << ","
	    << FCHN(it->dispColor.b) << ")\n";

	int pctLimit = floor(0.5 + 1000 * (MAX_IN_SOI / it->points.size()));
	if (pctLimit < 1) pctLimit = 1;

	for(size_t i = 0; i < it->points.size(); i++)
	{
	    if (rand() % 1000 > pctLimit)
		continue;
	    str << "v("
		<< it->points[i].p.x << ","
		<< it->points[i].p.y << ","
		<< it->points[i].p.z << ")\n";
	    pointCnt++;
	}
    }
#undef FCHN
    str << "glEnd()\n";
    str << "end\n";

    // logging
    long long t1 = tm.elapsed();
    string S = str.str();
    long long t2 = tm.elapsed();
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_POINTS), S);
    long long t3 = tm.elapsed();
    if (1) {
	str.str("");
	str.clear();
	str << "<h3>Plane popout - SendPoints</h3>";
	str << "Points: " << pointCnt << "<br>";
	str << "Strlen: " << S.length() << "<br>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Converted: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	str << "Sent: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	m_display.setHtml("INFO", "log.PPO.SendPoints", str.str());
    }
}

void PlanePopOut::SendPlaneGrid()
{
    castutils::CMilliTimer tm(true);
    ostringstream str;
    str << "function render()\n";
    str << "if not showPlaneGrid then return end\n";

#define FCHN(x) (float)x/255.0
    if (dominantPlane.valid) {
	str << "glBegin(GL_LINE_LOOP)\n";
	str << "glPointSize(2.0)\n";
	//str << "glColor("
	//    << FCHN(dominantPlane.dispColor.r) << ","
	//    << FCHN(dominantPlane.dispColor.g) << ","
	//    << FCHN(dominantPlane.dispColor.b) << ")\n";
	str << "glColor(1,0,0)\n";
	str << "v=glVertex\n";
	for(size_t i = 0; i < dominantPlane.hullPoints.size(); i++)
	    str << "v(" << dominantPlane.hullPoints[i].x << ","
		<< dominantPlane.hullPoints[i].y << ","
		<< dominantPlane.hullPoints[i].z << ")\n";
	str << "glEnd()\n";
    }
#undef FCHN
    str << "end\n";

    long long t1 = tm.elapsed_micros();
    string S = str.str();
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_PLANE), S);
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

void PlanePopOut::SendOverlays()
{
    ostringstream str;
    str << "function render()\n";
    str << "StdModel:zfloor(0.0)\n";
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


void PlanePopOut::SendSOIs(vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &sois)
{
    ostringstream str;
    str << "function render()\n"
	<< "if not showSois then return end\n";
    str << "v=glVertex\nc=glColor\n";
    str << "c(0.0,1.0,0.0)\n";
    for(size_t i = 0; i < sois.size(); i++)
    {
	size_t n = sois[i]->points.size();
	str << "glBegin(GL_LINE_LOOP)\n";
	for(size_t j = 0; j < n/2; j++)
	    str << "v("
		<< sois[i]->points[j].x << ","
		<< sois[i]->points[j].y << ","
		<< sois[i]->points[j].z << ")\n";
	str << "glEnd()\n";
	str << "glBegin(GL_LINE_LOOP)\n";
	for(size_t j = n/2; j < n; j++)
	    str << "v("
		<< sois[i]->points[j].x << ","
		<< sois[i]->points[j].y << ","
		<< sois[i]->points[j].z << ")\n";
	str << "glEnd()\n";
	str << "glBegin(GL_LINES)\n";
	for(size_t j = 0; j < n/2; j++)
	{
	    str << "v("
		<< sois[i]->points[j].x << ","
		<< sois[i]->points[j].y << ","
		<< sois[i]->points[j].z << ")\n";
	    str << "v("
		<< sois[i]->points[j + n/2].x << ","
		<< sois[i]->points[j + n/2].y << ","
		<< sois[i]->points[j + n/2].z << ")\n";
	}
	str << "glEnd()\n";
    }
    str << "end\n";
    m_display.setLuaGlObject(ID_OBJECT_3D, guiid(ID_PART_3D_SOI), str.str());
}
#endif

void PlanePopOut::runComponent()
{
#ifdef FEAT_VISUALIZATION
    SendOverlays();

    castutils::CRunningRate realRate;

    castutils::CCastPaceMaker<PlanePopOut> paceMaker(*this, 1000/5, 1);

    castutils::CMilliTimer tmSendPoints(true);
    tmSendPoints.setTimeout(500);

    castutils::CMilliTimer tmSendPlaneGrid(true);
    tmSendPlaneGrid.setTimeout(500);

    castutils::CMilliTimer tmSendImage(true);
    tmSendImage.setTimeout(750);

    castutils::CMilliTimer tmSendStatus(true);
    tmSendStatus.setTimeout(3000);
#endif

    try {
	while(isRunning()) {
	    paceMaker.sync();
	    realRate.tick();

	    try {
		GetImageData();
	    }
	    catch (exception& e) {
		error(" *** PPO GetImageData HAS CRASHED *** with: '%s'", e.what());
	    }
	    try {
		GetPlaneAndSOIs();
	    }
	    catch (exception& e) {
		error(" *** PPO GetPlaneANdSOIs HAS CRASHED *** with: '%s'", e.what());
	    }
	    try {
		TrackSOIs();
	    }
	    catch (exception& e) {
		error(" *** PPO TrackSOIs HAS CRASHED *** with: '%s'", e.what());
	    }

#ifdef FEAT_VISUALIZATION
	    if (m_bSendImage) {
		if (tmSendImage.isTimeoutReached()) {
		    SendImage();
		    tmSendImage.restart();
		}
	    }

	    if (tmSendStatus.isTimeoutReached()) {
		ostringstream ss;
		ss.precision(4); // set the _maximum_ precision
		ss << "<h3>PlanePopOut (" << getComponentID() << ") processing rate</h3>";
		ss << "current: " << realRate.getRate() << " tests/s<br>";
		ss << "from start: " << realRate.getTotalRate() << " tests/s<br>";
		m_display.setHtml("INFO", "ppo.rate/" + getComponentID(), ss.str());
		tmSendStatus.restart();
	    }

	    if (m_bSendPoints) {
		if (tmSendPoints.isTimeoutReached()) {
		    SendPoints(m_bColorByLabel);
		    tmSendPoints.restart();
		}
	    }
	    if (m_bSendPlaneGrid) {
		if (tmSendPlaneGrid.isTimeoutReached()) {
		    SendPlaneGrid();
		    tmSendPlaneGrid.restart();
		}
	    }
#endif
	    if (doDisplay)
		DisplayInTG();
	}
    }
    catch (exception& e) {
	error(" *** PPO isRunning HAS CRASHED *** with '%s'", e.what());
    }
    catch (...) {
	error(" *** PPO isRunning HAS CRASHED *** ");
    }
}

/**
 * Display in TomGine, i.e. own OpenGL rendering
 */
void PlanePopOut::DisplayInTG()
{
    cv::Mat_<cv::Point3f> cloud;
    cv::Mat_<cv::Point3f> colCloud;
    int cnt = (int)points.size();

    if (dominantPlane.valid)
	for(size_t i = 0; i < dominantPlane.planePoints.size(); i++)
	    cnt++;
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
	for(size_t i = 0; i < it->points.size(); i++)
	    cnt++;
    cloud = cv::Mat_<cv::Point3f>(1, cnt);
    colCloud = cv::Mat_<cv::Point3f>(1, cnt);    

    cnt = 0;
    if (dominantPlane.valid) {
	for(size_t i = 0; i < dominantPlane.planePoints.size(); i++)
	{
	    cv::Point3f p, cp;
	    p.x = (float)dominantPlane.planePoints[i].p.x;
	    p.y = (float)dominantPlane.planePoints[i].p.y;
	    p.z = (float)dominantPlane.planePoints[i].p.z;
	    cp.x = (float)dominantPlane.dispColor.r;
	    cp.y = (float)dominantPlane.dispColor.g;
	    cp.z = (float)dominantPlane.dispColor.b;
	    cloud.at<cv::Point3f>(0, cnt) = p;
	    colCloud.at<cv::Point3f>(0, cnt) = cp;
	    cnt++;
	}
    }
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
    {
	for(size_t i = 0; i < it->points.size(); i++)
	{
	    cv::Point3f p, cp;
	    p.x = (float)it->points[i].p.x;
	    p.y = (float)it->points[i].p.y;
	    p.z = (float)it->points[i].p.z;
	    cp.x = (float)it->dispColor.r;
	    cp.y = (float)it->dispColor.g;
	    cp.z = (float)it->dispColor.b;
	    cloud.at<cv::Point3f>(0, cnt) = p;
	    colCloud.at<cv::Point3f>(0, cnt) = cp;
	    cnt++;
	}
    }
    // NOTE: it is important that these unlabeled points are added to the display cloud
    // after the other points. This will let the plane and SOI points overwrite
    // the unlabled points. (apparently points are drawn in reverse order)
    for(size_t i = 0; i < points.size(); i++)
    {
	cv::Point3f p, cp;
	p.x = (float) points[i].p.x;
	p.y = (float) points[i].p.y;
	p.z = (float) points[i].p.z;
	cp.x = (uchar) points[i].c.b;	// change rgb to bgr
	cp.y = (uchar) points[i].c.g;
	cp.z = (uchar) points[i].c.r;
	cloud.at<cv::Point3f>(0, cnt) = p;
	colCloud.at<cv::Point3f>(0, cnt) = cp;
        cnt++;
    }
    tgRenderer->Clear();
    tgRenderer->SetPointCloud(cloud, colCloud);
}

/**
 * Get point cloud (and if visualisation is on also rectified image) from point cloud server.
 * On return, points (and iplDispImage) filled.
 */
void PlanePopOut::GetImageData()
{
    // TODO get from cast-file!
    int pointCloudWidth = PPO_POINTCLOUD_WIdTH;
    int imageWidth = PPO_IMAGE_WIDTH;

    points.clear();
    getCompletePoints(true, pointCloudWidth, points);

#ifdef FEAT_VISUALIZATION
    if (m_bSendImage) {
	Video::Image image;
	// get rectified image from point cloud server
	getRectImage(camId, imageWidth, image);
        cvReleaseImage(&iplDispImage);
	iplDispImage = convertImageToIpl(image);
    }
#endif
}


#define LOG_SHOW_PCL_LOCKS 0
#if LOG_SHOW_PCL_LOCKS
class LockerDebug {
public:
    IceUtil::Mutex::Lock lock;
    CASTComponent* pc;
    LockerDebug(IceUtil::Mutex& mutex, CASTComponent* pComponent): lock(mutex) {
	pc = pComponent;
	pc->log("*** LOCKED ***");
    }
    ~LockerDebug() {
	pc->log("*** UN-LOCKED ***");
    }
};
#endif


/**
 * Dominant plane detection and Euclidean Clustering
 * On return, we have dominentPlane valid and updated (if any could be found)
 * and the list of currentSOIs (possibly of length 0)
 */
void PlanePopOut::GetPlaneAndSOIs()
{
    // clear plane and SOIs
    dominantPlane.clear();
    currentSOIs.clear();

    try
    {
	// remembers whether the actual plane and SOIs computation succeeded
	bool ppo_ok = false;

	if (points.size() < PPO_MIN_POINTCLOUD_SIZE)
	    return;

	// first convert to PCL format, for PCL based plane and SOI detection
	// TODO get from cast-file!
	int pointCloudWidth = PPO_POINTCLOUD_WIdTH;
	int pointCloudHeight = 240;
	pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	ConvertSurfacePoints2PCLCloud(points, *pcl_cloud, pointCloudWidth, pointCloudHeight);

	// Dominant plane coefficients
	pcl::ModelCoefficients::Ptr pcl_domplane;
	// table hull points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tablehull;
	// the indices of points on the dominant plane
	pcl::PointIndices::Ptr planepoints;
	// detected SOIs: convex hull prisms, with first half of points the bottom
	// and second half the top polygon
	vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_sois;

	{
	    // libqhull which is used by PCL is not thread-safe.
	    // When multiple instances of PPO are calling PCL, we lock to avoid crashes.
	    // This will not prevent conflicts with other components using PCL.
    #if LOG_SHOW_PCL_LOCKS
	    auto_ptr<LockerDebug> pcllock;
	    if (m_componentCount > 1) {
		pcllock = auto_ptr<LockerDebug>(new LockerDebug(m_planePopoutMutex, this));
	    }
    #else
	    auto_ptr<IceUtil::Mutex::Lock> pcllock;
	    if (m_componentCount > 1) {
	       pcllock = auto_ptr<IceUtil::Mutex::Lock>(new IceUtil::Mutex::Lock(m_planePopoutMutex));
	    }
    #endif

	    //log("got %d points, after conversion: %d", (int)points.size(), (int)pcl_cloud->points.size());

	    /// TODO
	    std::vector<unsigned> labels;   /// TODO unused?
	    ppo_ok = m_planePopout->CalculateSOIs(pcl_cloud);
	    if(ppo_ok)
	    {
		m_planePopout->GetSOIs(pcl_sois, labels);
		m_planePopout->GetDominantPlaneCoefficients(pcl_domplane);
		m_planePopout->GetTableHull(tablehull);
		m_planePopout->CollectTableInliers(pcl_cloud, pcl_domplane);
		m_planePopout->GetPlanePoints(planepoints);
		log("****** HAPPY HAPPY SOIs");
	    }
	    else
		log("****** failed to calulcate SOIs");
	}

	if(ppo_ok)
	{
    #ifdef FEAT_VISUALIZATION
	    // NOTE: not nice having visualisiaton code here, but ok
	    if (m_bSendSois)
		SendSOIs(pcl_sois);
    #endif
	    // fill our dominant plane structure
	    dominantPlane.init(pcl_cloud, planepoints, pcl_domplane, tablehull);
	    if(dominantPlane.valid)
	    {
	      // fill our SOI structures
	      // NOTE: the point clouds returned as SOIs by the PlanePopout class are the
	      // vertices of the bounding prism. We are however interested in all original
	      // points inside the SOI.
	      /*for (size_t i = 0; i < pcl_cloud->points.size(); i++)
		{
		int soi_label = planePopout->IsInSOI(pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z);
	      // if point is in any SOI
	      if(soi_label != 0) {
	      SurfacePoint p;
	      p.p = vector3(pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z);
	      p.c.r = pcl_cloud->points[i].r;
	      p.c.g = pcl_cloud->points[i].g;
	      p.c.b = pcl_cloud->points[i].b;
	      currentSOIs[soi_label].points.push_back(p);
	      }
	      }*/

	      // NOTE: the above does not work for some as yet unknown reason, so we essentially
	      // do the same thing "by hand"
	      for (size_t i = 0; i < pcl_sois.size(); i++)
	      {
		// dummy call to create map entry
		currentSOIs[i].hist = 0;
		// NOTE: the following isPointIn2DPolygon() function needs the bottom
		// polygon of SOI prism, i.e. only the first half of the points
		pcl_sois[i]->points.resize(pcl_sois[i]->points.size()/2);
	      }
	      for (size_t i = 0; i < pcl_cloud->points.size(); i++)
	      {
		for(size_t j = 0; j < pcl_sois.size(); j++)
		{
		  if(pcl::isPointIn2DPolygon(pcl_cloud->points[i], *pcl_sois[j]))
		  {
		    SurfacePoint p;
		    p.p = vector3(pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z);
		    p.c.r = pcl_cloud->points[i].r;
		    p.c.g = pcl_cloud->points[i].g;
		    p.c.b = pcl_cloud->points[i].b;
		    currentSOIs[j].points.push_back(p);
		  }
		}
	      }
	      for (map<unsigned, SOIEntry>::iterator it = currentSOIs.begin(); it != currentSOIs.end(); it++)
		it->second.init(dominantPlane);
	    }
	}
    }
    catch(...)
    {
	log("caught unknown exception in %s, no SOIs found", __FUNCTION__);
    }
}

/**
 * Match tracked SOIs with currently found SOIs.
 * Update, create new or delete accordingly and reflect changes on WM.
 */
void PlanePopOut::TrackSOIs()
{
    // need to synchronise with GetStableSOIs
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(trackedSOIsMutex);

    // for each tracked SOI find the best match among current SOIs, if any
    // NOTE: this is rather primitive, more elaborate schemes are possible
    // but for now we leave as is
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
    {
	int best = -1;
	double best_match_prob = 0.;
        it->hasMatch = false;
	for (map<unsigned, SOIEntry>::iterator jt = currentSOIs.begin(); jt != currentSOIs.end(); jt++)
	{
	    if (!jt->second.hasMatch) {
		double match = it->matchProbability(jt->second);
		if (match > PPO_MIN_MATCH_PROB && match > best_match_prob) {
		    best_match_prob = match;
		    best = (int)jt->first;
		}
	    }
	}
	if (best != -1) {
	    it->establishMatch(currentSOIs[best]);

	    if (m_bWriteSoisToWm) {
		// if the SOI is already in WM, overwrite it
		if (!it->WMId.empty()) {
		    VisionData::SOIPtr wmsoi = it->createWMSOI(this);
		    overwriteWorkingMemory(it->WMId, wmsoi);
		} else {
		    // only if it has been stable for some time, add it to WM
		    if (it->numStableFrames >= StableTime) {
			VisionData::SOIPtr wmsoi = it->createWMSOI(this);
			it->WMId = newDataID();
			addToWorkingMemory(it->WMId, wmsoi);
			log("added SOI to WM");
		    }
		}
	    }
	} else {
	    // no update, i.e. not seen
	    it->numFramesNotSeen++;
	    it->numStableFrames = 0;
	}
    }

    // all current SOIs that were not used instantiate a new tracked SOI
    for (map<unsigned, SOIEntry>::iterator jt = currentSOIs.begin(); jt != currentSOIs.end(); jt++)
    {
#if 0
	log("current soi %d found a matching tracked soi : %s",
		(int)jt->first, (jt->second.hasMatch ? "yes" : "no"));
#endif
	if (!jt->second.hasMatch) {
	    trackedSOIs.push_back(jt->second);
	    // NOTE: has_match of the current SOI and the new tracked SOI are not set true in this
	    // case.
	    // NOTE: no WM action yet here. only once a SOI has been seen longer than StableTime
	    // will it be added to WM (see above)
	}
    }

    // now remove SOIs which have not been updated for a while
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); )
    {
	if (it->numFramesNotSeen > AgonalTime) {
	    // if it was added to WM at all
	    if (m_bWriteSoisToWm) {
		if (!it->WMId.empty()) {
		    deleteFromWorkingMemory(it->WMId);
		    log("removed SOI from WM");
		}
	    }
	    it = trackedSOIs.erase(it);
	} else {
	    it++; 
	}
    }

#if 0
    log("have %d tracked sois", (int)trackedSOIs.size());
    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
    {
	ostringstream str;
	str << "tracked SOI at: "
	    << it->boundingSphere.pos << " with " << it->points.size() << " points "
            << " stable: " << it->numStableFrames << " not seen:" << it->numFramesNotSeen;
	log("%s", str.str().c_str());
    }
#endif
}

// @author: mmarko
void PlanePopOut::GetStableSOIs(vector<SOIPtr>& soiList)
{
    // need to synchronise with TrackSOIs
    // TODO: (maybe) here we could use a read lock and a write lock in TrackSOIs
    // TODO: trackedSOIs have to be (read-)locked in all places
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(trackedSOIsMutex);

    for (list<SOIEntry>::iterator it = trackedSOIs.begin(); it != trackedSOIs.end(); it++)
    {
	if (it->numStableFrames >= StableTime) {
	    SOIPtr ps = it->createWMSOI(this);
	    soiList.push_back(ps);
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

    // TODO: the command should be handled in runComponent, where it would wait until
    // the sois are stable
    GetStableSOIs(cmd.pcmd->sois);
    cmd.succeed();
    debug("PlanePopOut: GetStableSoisCommand found %d SOIs.", cmd.pcmd->sois.size());
}

#if 0
void PlanePopOut::SaveHistogramImg(CvHistogram* hist, string str)
{
    int h_bins = 16, s_bins = 8;

    int height = 240;
    int width = (h_bins*s_bins*6);
    IplImage* hist_img = cvCreateImage( cvSize(width,height), 8, 3 );
    cvZero( hist_img );
    float max_value = 0;
    cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );

    // temp images, for HSV to RGB tranformation
    IplImage * hsv_color = cvCreateImage(cvSize(1,1),8,3);
    IplImage * rgb_color = cvCreateImage(cvSize(1,1),8,3);
    int bin_w = width / (h_bins * s_bins);
    for(int h = 0; h < h_bins; h++)
    {
	for(int s = 0; s < s_bins; s++)
	{
	    int i = h*s_bins + s;
	    // calculate the height of the bar
	    float bin_val = cvQueryHistValue_2D( hist, h, s );
	    int intensity = cvRound(bin_val*height/max_value);

	    // Get the RGB color of this bar
	    cvSet2D(hsv_color,0,0,cvScalar(h*180.f / h_bins,s*255.f/s_bins,255,0));
	    cvCvtColor(hsv_color,rgb_color,CV_HSV2RGB);
	    CvScalar color = cvGet2D(rgb_color,0,0);

	    cvRectangle( hist_img, cvPoint(i*bin_w,height),
		    cvPoint((i+1)*bin_w,height - intensity),
		    color, -1, 8, 0 );
	}
    }
    string path = str;
    path.insert(0,"/tmp/H-S-histogram_"); path.insert(path.length(),".jpg");
    cvSaveImage(path.c_str(), hist_img);
    cvReleaseImage(&hist_img);
    cvReleaseImage(&hsv_color);
    cvReleaseImage(&rgb_color);
}
#endif

}
// vim: set sw=4 sts=4 ts=8 noet list :vim

