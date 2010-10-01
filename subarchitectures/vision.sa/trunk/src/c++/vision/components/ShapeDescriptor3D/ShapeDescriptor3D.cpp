/**
 * @author Michael Zillich
 * @date September 2010
 */

#include <sstream>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <../../VisionUtils.h>
#include "PlaneRANSAC.h"
#include "ShapeDescriptor3D.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ShapeDescriptor3D();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace VisionData;

// default stereo image width
#define STEREO_WIDTH_DEF 320

// default size of angle histogram for RAS shape descriptor
#define HISTOGRAM_SIZE_DEF 24

#ifdef FEAT_VISUALIZATION
  // display objects
  #define ID_OBJECT_3D      "ShapeDescriptor3D.3D"
  #define ID_OBJECT_HIST    "ShapeDescriptor3D.Hist"
  //#define ID_OBJECT_IMAGE   "ShapeDescriptor3D.Image"

  // display controls
  //#define IDC_POINTS "ShapeDescriptor3D.show.points"
#endif

static void DrawPoint3D(std::ostringstream &str,
                double x, double y, double z, P::RGBColor col);
static void DrawLine3D(std::ostringstream &str,
                double x1, double y1, double z1, 
                double x2, double y2, double z2, P::RGBColor col);;
static void DrawCross3D(std::ostringstream &str,
                double x, double y, double z, double size, P::RGBColor col);
static void DrawScene3D(std::ostringstream &str, P::Scene3D &scene, map<unsigned, P::RGBColor> &colors);

ShapeDescriptor3D::ShapeDescriptor3D()
  : camId(0)
{
  stereoWidth = STEREO_WIDTH_DEF;
  histogramSize = HISTOGRAM_SIZE_DEF;
  useKeyPoints = false;
  ransacThr = 0.005;
  planeMinPoints = 100;
  logImages = false;
#ifdef FEAT_VISUALIZATION
  m_display.setClientData(this);
#endif
}

ShapeDescriptor3D::~ShapeDescriptor3D()
{
}

void ShapeDescriptor3D::configure(const std::map<std::string,std::string> & _config)
{
  P::RASDescriptor ras;
  log("configure: ras size %d", ras.size);
  for (unsigned i=0; i<ras.Size(); i++)
    log(" %f", ras.data[i]);

  // first let the base classes configure themselves
  configureStereoCommunication(_config);

  map<string,string>::const_iterator it;

  if((it = _config.find("--stereoWidth")) != _config.end())
  {
    istringstream str(it->second);
    str >> stereoWidth;
  }

  if((it = _config.find("--histogramSize")) != _config.end())
  {
    istringstream str(it->second);
    str >> histogramSize;
    assert(histogramSize > 0);
  }

  // note: use either --useKeyPoints or --use3DPoints
  if((it = _config.find("--useKeyPoints")) != _config.end())
  {
    useKeyPoints = true;
  }

  // note: use either --useKeyPoints or --use3DPoints
  if((it = _config.find("--use3DPoints")) != _config.end())
  {
    useKeyPoints = false;
  }

  if((it = _config.find("--planeDistThr")) != _config.end())
  {
    istringstream str(it->second);
    str >> ransacThr;
    assert(ransacThr > 0.);
  }

  if((it = _config.find("--planeMinPoints")) != _config.end())
  {
    istringstream str(it->second);
    str >> planeMinPoints;
    assert(planeMinPoints > 0);
  }

  if((it = _config.find("--logImages")) != _config.end())
  {
    logImages = true;
  }

#ifdef FEAT_VISUALIZATION
	m_display.configureDisplayClient(_config);
#endif
}

void ShapeDescriptor3D::start()
{
  startStereoCommunication(*this);

  addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::ADD),
    new MemberFunctionChangeReceiver<ShapeDescriptor3D>(this,
      &ShapeDescriptor3D::newProtoObject));
  /*addChangeFilter(createLocalTypeFilter<VisionData::ProtoObject>(cdl::OVERWRITE),
    new MemberFunctionChangeReceiver<ShapeDescriptor3D>(this,
      &ShapeDescriptor3D::updatedProtoObject));*/

#ifdef FEAT_VISUALIZATION
  m_display.connectIceClient(*this);
  m_display.installEventReceiver();
  //m_display.addCheckBox(ID_OBJECT_3D, IDC_POINTS, "Show 3D points");

  // Object displays (m_bXX) are set to off: we need to create dummy display objects
  // on the server so that we can activate displays through GUI
  m_display.setLuaGlObject(ID_OBJECT_3D, "3D planes", "function render()\nend\n");
  m_display.setLuaGlObject(ID_OBJECT_HIST, "angle histogram", "function render()\nend\n");
#endif
}

void ShapeDescriptor3D::runComponent()
{
}

void ShapeDescriptor3D::newProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  pobjPtr = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  if(useKeyPoints)
    calculateDescriptor(*pobjPtr);
  else
    calculateDescriptor2(*pobjPtr);
  overwriteWorkingMemory(_wmc.address, pobjPtr);
}

void ShapeDescriptor3D::updatedProtoObject(const cdl::WorkingMemoryChange & _wmc)
{
  pobjPtr = getMemoryEntry<VisionData::ProtoObject>(_wmc.address);
  if(useKeyPoints)
    calculateDescriptor(*pobjPtr);
  else
    calculateDescriptor2(*pobjPtr);
  overwriteWorkingMemory(_wmc.address, pobjPtr);
}

void ShapeDescriptor3D::calculateDescriptor(ProtoObject &pobj)
{
  Video::Image image[2];
  IplImage *iplImage[2];
  IplImage *iplDebug[2];
  IplImage *iplMask;
  P::RASDescriptor ras;

  for(int i = LEFT; i <= RIGHT; i++)
  {
    StereoClient::getRectImage(i, stereoWidth, image[i]);
    iplImage[i] = convertImageToIplGray(image[i]);
    iplDebug[i] = cvCreateImage(cvGetSize (iplImage[i]), 8, 3 );
    cvConvertImage(iplImage[i], iplDebug[i]);
  }
  iplMask = cvCreateImage(cvGetSize(iplImage[LEFT]), 8, 1);

  // project SOI associated with the proto object to image ROI
  string soiAddr = pobj.SOIList.front();
  SOIPtr soiPtr = getMemoryEntry<VisionData::SOI>(soiAddr);
  // HACK: 1.3 is the ominous dilate factor of SOIFilter
  // super shitty hack
  //soiPtr->boundingSphere.rad *= 1.3;
  // HACK note: this is done in SOIFilter and the SOI is updated on WM!
  ROIPtr roiPtr = projectSOI(image[LEFT].camPars, *soiPtr);

  // prepare mask image with white ROI on black background
  cvSet(iplMask, cvScalar(0));
  int xoffs = (int)(roiPtr->rect.pos.x - roiPtr->rect.width/2);
  int yoffs = (int)(roiPtr->rect.pos.y - roiPtr->rect.height/2);
  for(int y = 0; y < pobj.mask.height; y++)
    for(int x = 0; x < pobj.mask.width; x++)
      // foreground object is labelled 1
      if(pobj.mask.data[pobj.mask.width*y + x] == 1)
      {
        int yy = y + yoffs;
        int xx = x + xoffs;
        if(xx >= 0 && xx < iplMask->width && yy >= 0 && yy < iplMask->height)
          iplMask->imageData[yy*iplMask->width + xx] = 255;
      }

  // pepare camera parameters in format expected by dshape
  CvMat *intrinsic[2];
  CvMat *distortion[2];
  CvMat *rot = cvCreateMat(3, 3, CV_32FC1);
  CvMat *trans = cvCreateMat(3, 1, CV_32FC1);
  for(int i = LEFT; i <= RIGHT; i++)
  {
    intrinsic[i] = cvCreateMat(3, 3, CV_32FC1);
    distortion[i] = cvCreateMat(5, 1, CV_32FC1);
    cvSet(intrinsic[i], cvScalar(0));
    cvmSet(intrinsic[i], 0, 0, image[i].camPars.fx);
    cvmSet(intrinsic[i], 0, 2, image[i].camPars.cx);
    cvmSet(intrinsic[i], 1, 1, image[i].camPars.fy);
    cvmSet(intrinsic[i], 1, 2, image[i].camPars.cy);
    cvmSet(intrinsic[i], 2, 2, 1.);
    cvSet(distortion[i], cvScalar(0));
  }
  cvSet(rot, cvScalar(0));
  cvmSet(rot, 0, 0, 1.);
  cvmSet(rot, 1, 1, 1.);
  cvmSet(rot, 2, 2, 1.);
  cvSet(trans, cvScalar(0));
  // note: dshape expects mm, we have m. also it expects a negative x
  // (probably stemming from an error in SVS valib file)
  cvmSet(trans, 0, 0, 1000.*image[RIGHT].camPars.pose.pos.x);

  cvSave("intr-L.xml", intrinsic[LEFT]);
  cvSave("intr-R.xml", intrinsic[RIGHT]);
  cvSave("dist-L.xml", distortion[LEFT]);
  cvSave("dist-R.xml", distortion[RIGHT]);
  cvSave("rot-R.xml", rot);
  cvSave("trans-R.xml", trans);

  dshape.SetDebugImage(iplDebug[LEFT], iplDebug[RIGHT]);
  dshape.SetCameraParameter(intrinsic[LEFT], intrinsic[RIGHT],
      distortion[LEFT], distortion[RIGHT], rot, trans);
  // note: we do not use relative scale between planes, therefore sizeScale = 1
  dshape.Operate(iplImage[LEFT], iplImage[RIGHT], ras, histogramSize, 1, iplMask);

  // set shape descriptpr of proto object
  pobj.rasShapeDesc.angleHistogram.resize(ras.Size());
  for(unsigned i=0; i < ras.Size(); i++)
    pobj.rasShapeDesc.angleHistogram[i] = ras.data[i];

  if(logImages)
  {
    cvSaveImage("shape-mask.jpg", iplMask);
    cvSaveImage("shape-img-L.jpg", iplImage[LEFT]);
    cvSaveImage("shape-img-R.jpg", iplImage[RIGHT]);
    cvSaveImage("shape-debug-L.jpg", iplDebug[LEFT]);
    cvSaveImage("shape-debug-R.jpg", iplDebug[RIGHT]);
  }

  // HACK
  log("calculate descr: ras size %d", ras.size);
  for (unsigned i=0; i<ras.Size(); i++)
    log(" %f", ras.data[i]);

#ifdef FEAT_VISUALIZATION
  redraw3D();
  redrawHistogram(pobj);
#endif

  for(int i = LEFT; i <= RIGHT; i++)
  {
    cvReleaseImage(&iplImage[i]);
    cvReleaseImage(&iplDebug[i]);
    cvReleaseMat(&intrinsic[i]);
    cvReleaseMat(&distortion[i]);
  }
  cvReleaseImage(&iplMask);
  cvReleaseMat(&rot);
  cvReleaseMat(&trans);
}

void ShapeDescriptor3D::calculateDescriptor2(ProtoObject &pobj)
{
  vector<Plane3> planes;
  int n = (int)pobj.points.size();
  P::RASDescriptor ras;

  log("using 3D points, planeDistThr: %lf, planeMinPoints: %d", ransacThr, planeMinPoints);

  findPlanes(planes, pobj.points, n);

  for(size_t i = 0; i < rasPlanes.Size(); i++)
    delete rasPlanes[i];
  rasPlanes.Clear();

  // copy cogx plane representation to RAS plane representation
  // note: we only fill parts of the RAS plane structure
  for(size_t i = 0; i < planes.size(); i++)
  {
    P::Plane *rasPlane = new P::Plane;

    vector<CvPoint2D32f> planePoints;
    Pose3 planePose;
    definePlaneCoordSys(planes[i], planePose);
    for(size_t j = 0; j < pobj.points.size(); j++)
    {
      // if inlier
      if(fabs(distPointToPlane(pobj.points[j].p, planes[i])) < ransacThr)
      {
        Vector3 q = transformInverse(planePose, pobj.points[j].p);
        planePoints.push_back(cvPoint2D32f(q.x, q.y));
      }
    }
    int *hull = (int*)malloc(planePoints.size() * sizeof(hull[0]));
    CvMat hullMat = cvMat(1, planePoints.size(), CV_32SC1, hull);
    CvMat pointMat = cvMat(1, planePoints.size(), CV_32FC2, &planePoints[0]);
    cvConvexHull2(&pointMat, &hullMat, CV_CLOCKWISE, 0);

    rasPlane->p = P::Vector3(planePose.pos.x, planePose.pos.y, planePose.pos.z);
    Vector3 n = getColumn(planePose.rot, 2);
    rasPlane->n = P::Vector3(n.x, n.y, n.z);
    for(int j = 0; j < hullMat.cols; j++)
    {
      Vector3 q = transform(planePose,
        vector3(planePoints[hull[j]].x, planePoints[hull[j]].y, 0.));
      rasPlane->contour.PushBack(P::Vector3(q.x, q.y, q.z));
    }

    free(hull);

    rasPlanes.PushBack(rasPlane);

    cout << "plane " << i << ": " << planes[i] << endl;
  }

  dshape.ComputeRAShapeDescriptor(rasPlanes, ras, histogramSize, 1);

  // set shape descriptpr of proto object
  pobj.rasShapeDesc.angleHistogram.resize(ras.Size());
  for(unsigned i=0; i < ras.Size(); i++)
    pobj.rasShapeDesc.angleHistogram[i] = ras.data[i];
  
#ifdef FEAT_VISUALIZATION
  redraw3D();
  redrawHistogram(pobj);
#endif
}

/**ŗŘ
 * Removes inliers from points.
 * To pe precise: inliers are moved to end of points array and n decreased
 * accordingly. So after call to this function points[0..n-1] are all outliers.
 */
void ShapeDescriptor3D::removeInliers(SurfacePointSeq &points, int &n,
    const Plane3 &plane, double thr)
{
  SurfacePoint t;
  for(int i = 0; i < n; )
  {
    // if inlier
    if(fabs(distPointToPlane(points[i].p, plane)) < thr)
    {
      // move point i to end of points and decrease n
      t = points[i];
      points[i] = points[n - 1];
      points[n - 1] = t;
      n--;
    }
    else
    {
      i++;
    }
  }
}

void ShapeDescriptor3D::findPlanes(vector<Plane3> &planes, SurfacePointSeq &points, int &n)
{
  PlaneRANSAC planeDetector(ransacThr, planeMinPoints);
  Plane3 plane;
  while(planeDetector.detectPlane(points, n, vector3(0, 0, 0), false, plane))
  {
    removeInliers(points, n, plane, 2.*ransacThr);
    planes.push_back(plane);
  }
}

#ifdef FEAT_VISUALIZATION
void ShapeDescriptor3D::MyDisplayClient::handleEvent(const Visualization::TEvent &event)
{
	if(!owner)
    return;
	/*if(event.sourceId == IDC_POINTS)
	{
		if(event.data == "0" || event.data=="")
      ...
		else
      ...
	}*/
}

std::string ShapeDescriptor3D::MyDisplayClient::getControlState(const std::string &ctrlId)
{
	if(!owner)
    return "";
	/*if(ctrlId == IDC_POPOUT_POINTS)
	{
		if(...)
      return "2";
		else
      return "0";
	}*/
	return "";
}

void ShapeDescriptor3D::redrawHistogram(const ProtoObject &pobj)
{
  //int w = 100, h = 100;
  std::ostringstream str;
  str << "function render()\n";
  //str << "glViewport(0, 0, " <<  w << ", " << h << ")\n";
  str << "glMatrixMode(GL_PROJECTION)\n";
  str << "glLoadIdentity()\n";
  str << "gluOrtho2D(0, 1, 0, 1)\n";
  str << "glPixelZoom(1.0, 1.0)\n";
  str << "glMatrixMode(GL_MODELVIEW)\n";
  str << "glLoadIdentity()\n";
  
  // draw a few grid lines
  str << "glColor(0.5, 0.5, 0.5)\n";
  str << "glBegin(GL_LINES)\n";
  for(double x = 0.0; x <= 1.0; x += 0.25)
  {
    // steps of pi/2
    str << "glVertex(" << x << ", " << 0.0 << ")\n";
    str << "glVertex(" << x << ", " << 1.0 << ")\n";
  }
  str << "glEnd()\n";
  str << "glLineStipple(1, 0x00ff)\n";
  str << "glEnable(GL_LINE_STIPPLE)\n";
  str << "glBegin(GL_LINES)\n";
  for(double x = 0.0; x <= 1.0; x += 0.125)
  {
    // steps of pi/4
    str << "glVertex(" << x << ", " << 0.0 << ")\n";
    str << "glVertex(" << x << ", " << 1.0 << ")\n";
  }
  str << "glEnd()\n";
  str << "glDisable(GL_LINE_STIPPLE)\n";

  // draw the histogram
  str << "glColor(0.0, 0.0, 1.0)\n";
  str << "glBegin(GL_QUADS)\n";
  double w = 1./(double)pobj.rasShapeDesc.angleHistogram.size();
  for(size_t i = 0; i < pobj.rasShapeDesc.angleHistogram.size(); i++)
  {
    double x = (double)i/(double)pobj.rasShapeDesc.angleHistogram.size();
    double y = pobj.rasShapeDesc.angleHistogram[i];
    str << "glVertex(" << x << ", " << 0 << ")\n";
    str << "glVertex(" << x + w << ", " << 0 << ")\n";
    str << "glVertex(" << x + w << ", " << y << ")\n";
    str << "glVertex(" << x << ", " << y << ")\n";
  }
  str << "glEnd()\n";

  str << "end\n";
  m_display.setLuaGlObject(ID_OBJECT_HIST, "angle histogram", str.str());
}

void ShapeDescriptor3D::redraw3D()
{
  std::ostringstream str;
  P::Scene3D scene;
  dshape.GetScene(scene);
  str << "function render()\n";
  DrawScene3D(str, scene, displayColors);
  
  for(size_t i = 0; i < rasPlanes.Size(); i++)
  {
    for(size_t j = 0; j < rasPlanes[i]->contour.Size(); j++)
    {
      P::Vector3 &p1 = rasPlanes[i]->contour[j];
      P::Vector3 &p2 = (j+1 < rasPlanes[i]->contour.Size() ? rasPlanes[i]->contour[j+1] : rasPlanes[i]->contour[0]);
      DrawLine3D(str, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, P::RGBColor(255, 255, 255));
    }
  }

  for(size_t i = 0; i < pobjPtr->points.size(); i++)
  {
    DrawPoint3D(str, pobjPtr->points[i].p.x, pobjPtr->points[i].p.y, pobjPtr->points[i].p.z,
      P::RGBColor((unsigned char)pobjPtr->points[i].c.r,
                  (unsigned char)pobjPtr->points[i].c.g,
                  (unsigned char)pobjPtr->points[i].c.b));
  }
  str << "end\n";
  m_display.setLuaGlObject(ID_OBJECT_3D, "3D planes", str.str());
}

void DrawPoint3D(std::ostringstream &str,
                 double x, double y, double z, P::RGBColor col)
{
  // scale graphics output cauas right now i can't zoom in the viewer
  double s = 10.;
  str << "glPointSize(2);\n";
  str << "glColor(" << (double)col.r/255. << ", "
      << (double)col.g/255. << ", "
      << (double)col.b/255. << ")\n";
  str << "glBegin(GL_POINTS)\n";
  str << "glVertex(" << s*x << ", " << s*y << ", " << s*z << ")\n";
  str << "glEnd()\n";
}

void DrawLine3D(std::ostringstream &str,
                double x1, double y1, double z1, 
                double x2, double y2, double z2, P::RGBColor col)
{
  // scale graphics output cauas right now i can't zoom in the viewer
  double s = 10.;
  str << "glColor(" << (double)col.r/255. << ", "
      << (double)col.g/255. << ", "
      << (double)col.b/255. << ")\n";
  str << "glBegin(GL_LINES)\n";
  str << "glVertex(" << s*x1 << ", " << s*y1 << ", " << s*z1 << ")\n";
  str << "glVertex(" << s*x2 << ", " << s*y2 << ", " << s*z2 << ")\n";
  str << "glEnd()\n";
}

void DrawCross3D(std::ostringstream &str, 
                double x, double y, double z, double size, P::RGBColor col)
{
  DrawLine3D(str, x-size,y,z , x+size,y,z, col);
  DrawLine3D(str, x,y-size,z , x,y+size,z, col);
  DrawLine3D(str, x,y,z-size , x,y,z+size, col);
}

void DrawScene3D(std::ostringstream &str, P::Scene3D &scene, map<unsigned, P::RGBColor> &colors)
{
  str << "glDisable(GL_CULL_FACE)\n";
  str << "glDisable(GL_LIGHTING)\n";

  for(unsigned i = 0; i < scene.ids.Size(); i++)
  {
    map<unsigned, P::RGBColor>::iterator it = colors.find(scene.ids[i]);
    P::RGBColor col;
    if(it == colors.end())
    {
      col = P::RGBColor(rand()%255, rand()%255, rand()%255);
      colors[scene.ids[i]] = col;
    }
    else
      col = colors[scene.ids[i]];

    for(unsigned j = 0; j < scene.cs[i].Size(); j++)
    {
      DrawCross3D(str, scene.cs[i][j].x, scene.cs[i][j].y, scene.cs[i][j].z, 0.003, col);
    }
    for(unsigned j = 0; j < scene.contours[i].Size(); j++)
    {
      P::Vector3 &p1 = scene.contours[i][j];
      P::Vector3 &p2 = (j+1 < scene.contours[i].Size() ? scene.contours[i][j+1] : scene.contours[i][0]);
      DrawLine3D(str, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, col);
    }
  }
}

#endif

}

