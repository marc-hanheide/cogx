/**
 * @author Marko Mahnič
 * @date September 2010
 */

#include "VideoGrabber.h"
#include "StringFmt.h"

#include <highgui.h>
#include <VideoUtils.h>
#include <IceUtil/Timer.h>
#include <castutils/Timers.hpp>

#define IDOBJ_GRABBER "Video.Grabber"
#define IDOBJ_SETTINGS "Video.Grabber.Settings"

#define IDPART_SETTINGS_FORM "001.settings"
#define IDPART_SETTINGS_VALUES "002.values"
#define IDCMD_GRAB   "video.grab"
#define IDCMD_RECORD "video.record"
#define IDCMD_STOP   "video.stop"
#define IDCTRL_STREAMING "video.streaming"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
   cast::CASTComponentPtr newComponent()
   {
      return new cogx::CVideoGrabber();
   }
}

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <pthread.h> // XXX: Testing code
#include <unistd.h>

double fclocks()
{
   return 1.0 * clock() / CLOCKS_PER_SEC;
}
// --------------------------


namespace cogx
{

using namespace std;
using namespace cast;
namespace cxd = cogx::display;

Video::CIplImageCache CPreview::gCache;
int CDataSource::count = 0;

IplImage* cloneVideoImage(const Video::Image &img)
{
   // HACK: Share data between IplImage and vector
   IplImage* pImg = cvCreateImageHeader(cvSize(img.width, img.height), IPL_DEPTH_8U, 3);
   pImg->imageData = (char*) &(img.data[0]);
   pImg->imageDataOrigin = pImg->imageData;
   pImg->widthStep = img.width * 3;
   pImg->imageSize = pImg->widthStep * img.height;
   return pImg;
}

void releaseClonedImage(IplImage** pImagePtr)
{
   if (!pImagePtr) return;
   if (*pImagePtr) {
      // HACK: Share data between IplImage and vector
      (*pImagePtr)->imageData = NULL;
      (*pImagePtr)->imageDataOrigin = NULL;
      cvReleaseImage(pImagePtr);
   }
}

std::string CGrabbedItem::makeFilename(const CRecordingInfo& frameInfo, int deviceId, const std::string& ext)
{
   std::string fname = frameInfo.filenamePatt;
   std::string sval = _str_(frameInfo.counter, frameInfo.counterDigits, '0');
   _s_::replace(fname, "%c", sval);

   std::string fullname = fname;
   std::string devname;
   if (deviceId < frameInfo.deviceNames.size())
      devname = frameInfo.deviceNames[deviceId];
   else devname = "d" + _str_(deviceId, 2, '0');
   _s_::replace(fullname, "%d", devname);

   fullname = frameInfo.directory + "/" + fullname + ext;

   return fullname;
}

void CVideoGrabClient::grab(std::vector<CGrabbedItemPtr>& items)
{
   std::vector<Video::CCachedImagePtr> timgs;
   getCachedImages(timgs);
   for(auto itt = timgs.begin(); itt != timgs.end(); itt++) {
      items.push_back(CGrabbedItemPtr(new CGrabbedCachedImage(*itt)));
      //pack.images.push_back(*itt);
   }
}

void CVideoGrabClient::getPreviews(std::vector<CPreview>& previews,
      int width, int height, bool isGrabbing)
{
   std::vector<Video::CCachedImagePtr> timgs;
   getCachedImages(timgs);

   int i = 0;
   for (auto pCachedImg : timgs) {
      std::string cid = "pv-video-" + _str_(mId) + "." + _str_(i, 2, '0');
      previews.push_back(CPreview(cid, width, height));
      CPreview& pv = previews.back();

      pv.deviceName = "video." + _str_(mId) + "." + _str_(i);
      pv.deviceInfo = _str_(pCachedImg->width, "d") + "x" + _str_(pCachedImg->height, "d");

      IplImage *pPreview = pv.getImage();
      IplImage *pOrig = cloneVideoImage(*pCachedImg);
      cvResize(pOrig, pPreview);
      releaseClonedImage(&pOrig);
      i++;
   }
}

void CGrabbedCachedImage::save(const CRecordingInfo& frameInfo, int deviceId)
{
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   std::string fullname = makeFilename(frameInfo, deviceId, ".png");
   // TODO: println("Saving image: %s", fullname.c_str());
   // TODO: if (!m_fakeRecording) {
      IplImage *iplImage = cloneVideoImage(*mpImage);
      cvSaveImage(fullname.c_str(), iplImage);
      releaseClonedImage(&iplImage);
   //}
}

void CGrabbedImage::save(const CRecordingInfo& frameInfo, int deviceId)
{
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   std::string fullname = makeFilename(frameInfo, deviceId, ".png");
   IplImage *iplImage = cloneVideoImage(mImage);
   cvSaveImage(fullname.c_str(), iplImage);
   releaseClonedImage(&iplImage);
}

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
void CPcGrabClient::grab(std::vector<CGrabbedItemPtr>& items)
{
   if (mbGrabPoints) {
      CGrabbedPcPoints *pPoints = new CGrabbedPcPoints();
      getPoints(false, 320, pPoints->mPoints);
      //printf(" ***** getPoints: %ld\n", pPoints->mPoints.size());
      mLastPoints = CGrabbedItemPtr(pPoints);
      items.push_back(mLastPoints);
   }
   if (mbGrabDepth) {
   }
   if (mbGrabRectImage) {
      CGrabbedImage *pImage = new CGrabbedImage();
      getRectImage(0, 640, pImage->mImage);
      mLastRectImage = CGrabbedItemPtr(pImage);
      items.push_back(mLastRectImage);
   }
}

void CPcGrabClient::getPreviews(std::vector<CPreview>& previews,
      int width, int height, bool isGrabbing)
{
   if (!isGrabbing) {
      std::vector<CGrabbedItemPtr> items;
      grab(items); // data will be present also in mLastXXX
   }

   CGrabbedImage depthImg;
   if (mbGrabPoints || mbGrabDepth) {
      getRectImage(-1, 320, depthImg.mImage);
   }

   if (mbGrabPoints) {
      std::string cid = "pc-points-" + _str_(mId);
      previews.push_back(CPreview(cid, width, height));
      CPreview& pv = previews.back();
      pv.deviceName = "pc." + _str_(mId) + ".0";
      pv.deviceInfo = "3D points";

      IplImage* pPreview = pv.getImage();
      IplImage *pOrig = cloneVideoImage(depthImg.mImage);
      cvResize(pOrig, pPreview);
      releaseClonedImage(&pOrig);
   }
   if (mbGrabDepth) {
      std::string cid = "pc-depth-" + _str_(mId);
      previews.push_back(CPreview(cid, width, height));
      CPreview& pv = previews.back();
      pv.deviceName = "pc." + _str_(mId) + ".1";
      pv.deviceInfo = "Depth";

      IplImage* pPreview = pv.getImage();
      IplImage *pOrig = cloneVideoImage(depthImg.mImage);
      cvResize(pOrig, pPreview);
      releaseClonedImage(&pOrig);
   }
   if (mbGrabRectImage) {
      std::string cid = "pc-rectimg-" + _str_(mId);
      previews.push_back(CPreview(cid, width, height));
      CPreview& pv = previews.back();
      pv.deviceName = "pc." + _str_(mId) + ".2";
      pv.deviceInfo = "Rectified ";

      IplImage* pPreview = pv.getImage();
      CGrabbedImage *pGrabbed = dynamic_cast<CGrabbedImage*>(mLastRectImage.get());

      if (! pGrabbed) {
         cvZero(pPreview);
         pv.deviceInfo += "???";
      }
      else {
         pv.deviceInfo += _str_(pGrabbed->mImage.width) + "x" + _str_(pGrabbed->mImage.height);
         IplImage *pOrig = cloneVideoImage(pGrabbed->mImage);
         cvResize(pOrig, pPreview);
         releaseClonedImage(&pOrig);
      }
   }
}

#ifdef FEAT_VISUALIZATION
void CPcGrabClient::configExtraV11n(cogx::display::CDisplayClient& display)
{
   std::string devid = "pc." + _str_(mId) + ".2";
   std::ostringstream ss;
   ss <<  "function render()\nend\n"
      << "setCamera('ppo.robot.head', -1.0, 0, 3.0, 1, 0, -1, 0, 0, 1)\n"
      << "setCamera('ppo.robot.front', 4.0, 0, 4.0, -1, 0, -1, 0, 0, 1)\n"
      << "setCamera('ppo.points.top', 0, 0, 4.0, 0, 0, -1, -1, 0, 0)\n";
   display.setLuaGlObject("grab." + devid, "cameras", ss.str());
}

void CPcGrabClient::displayExtra(cogx::display::CDisplayClient& display)
{
    castutils::CMilliTimer tm(true);
    CGrabbedItemPtr points = mLastPoints;
    std::string devid = "pc." + _str_(mId) + ".2";
    CGrabbedPcPoints* pPoints = dynamic_cast<CGrabbedPcPoints*>(points.get());
    if (!pPoints)
       return;
    if (!pPoints->mPoints.size())
       return;

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
    int next = rand() % 10 + 5;
    for (auto sp : pPoints->mPoints) {
       next--;
       if (next > 0) {
          continue;
       }
       next = rand() % 10 + 5;
       str << "c("
          << FCHN(sp.c.r) << ","
          << FCHN(sp.c.g) << ","
          << FCHN(sp.c.b) << ")\n";
       str << "v("
          << sp.p.x << ","
          << sp.p.y << ","
          << sp.p.z << ")\n";
       pointCnt++;
    }
	
#undef FCHN
    str << "glEnd()\n";
    str << "end\n";

    // logging
    long long t1 = tm.elapsed();
    string S = str.str();
    long long t2 = tm.elapsed();
    display.setLuaGlObject("grab." + devid, devid, S);
    long long t3 = tm.elapsed();
    if (1) {
	str.str("");
	str.clear();
	str << "<h3>Grabber - SendPoints: " + devid + "</h3>";
	str << "Points: " << pointCnt << "<br>";
	str << "Strlen: " << S.length() << "<br>";
	str << "Generated: " << t1 << "ms from start (in " << t1 << "ms).<br>";
	str << "Converted: " << t2 << "ms from start (in " << (t2-t1) << "ms).<br>";
	str << "Sent: " << t3 << "ms from start (in " << (t3-t2) << "ms).<br>";
	display.setHtml("INFO", "log.grab." + devid, str.str());
    }
}
#endif

// Save surface points as R G B X Y Z, tab-separated
void CGrabbedPcPoints::save(const CRecordingInfo& frameInfo, int deviceId)
{
   std::string fullname = makeFilename(frameInfo, deviceId, ".dat");
   std::ofstream fo(fullname);
   fo << ";;R\tG\tB\tx\ty\tz" << std::endl;
   for (auto sfp : mPoints) {
      fo.precision(3);
      fo << sfp.c.r << "\t" << sfp.c.g << "\t" << sfp.c.b << "\t";
      fo.precision(6);
      fo << sfp.p.x << "\t" << sfp.p.y << "\t" << sfp.p.z
         << std::endl;
   }
   fo.close();
}
#endif

CVideoGrabber::CVideoGrabber()
{
   m_frameGrabMs = 200;
#ifdef FEAT_VISUALIZATION
   m_pDisplayCanvas = NULL;
   m_display.setClientData(this);
#endif
}

CVideoGrabber::~CVideoGrabber()
{
   m_fakeRecording = false;
#ifdef FEAT_VISUALIZATION
   releaseCanvas();
#endif
   for(unsigned int i = 0; i < m_video.size(); i++) {
      Video::CVideoClient2* pVideo = m_video[i];
      pVideo->setReceiving(false);
      pVideo->disconnect();
      delete pVideo;
   }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
   CPcGrabClient *pPc;
   for(unsigned int i = 0; i < m_pointcloud.size(); i++) {
      pPc = m_pointcloud[i];
      delete pPc;
   }
#endif
}

bool parseBool(const std::string& value, bool valEmpty=false, bool valInvalid=false)
{
   string s = _s_::lower(_s_::strip(value));
   if (s == "")
      return valEmpty;
   if (s == "1" || s == "yes" || s == "true" || s == "on")
      return true;
   if (s == "0" || s == "no" || s == "false" || s == "off")
      return false;
   return valInvalid;
}

void CVideoGrabber::configure(const map<string,string> & _config)
{
   map<string,string>::const_iterator it, iCam, iopt;

   string serverName;
   vector<int> camids;
   int id;

   // Video servers
   for (it = _config.begin(); it != _config.end(); it++) {
      string sffx = "";

      if (it->first == "--videoname") iCam = _config.find("--camids");
      else if (_s_::startswith(it->first, "--videoname.")) {
         sffx = it->first.substr(11);
         iCam = _config.find("--camids" + sffx);
      }
      else continue;

      serverName = _s_::strip(it->second);
      if (serverName == "") continue;
      camids.clear();

      if (iCam != _config.end()) {
         istringstream str(iCam->second);
         while(str >> id) camids.push_back(id);
      }

      if (camids.size() < 1) {
         throw runtime_error(exceptionMessage(__HERE__, 
                  "Grabber id='%s': No --camids%s for VideoServer id='%s'.",
                  getComponentID().c_str(), sffx.c_str(), serverName.c_str()));
      }

      CVideoGrabClient* pVideo = new CVideoGrabClient();
      pVideo->setServer(this, serverName, camids);
      pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
      m_video.push_back(pVideo);
   }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
   CPcGrabClient *pPc;
   // Pointcloud servers
   for (it = _config.begin(); it != _config.end(); it++) {
      if (it->first != "--pcserver" && !_s_::startswith(it->first, "--pcserver."))
         continue;

      serverName = _s_::strip(it->second);

      if (serverName == "") continue;

      string sffx = "";
      if (_s_::startswith(it->first, "--pcserver.")){
         sffx = it->first.substr(10);
      }

      map<string,string> conf;
      conf["--pcserver"] = serverName;
      pPc = new CPcGrabClient();

      iopt = _config.find("--pcgrabpoints" + sffx);
      if (iopt != _config.end()) {
         pPc->mbGrabPoints = parseBool(iopt->second, /*empty=*/ true);
      }
      iopt = _config.find("--pcgrabdepth" + sffx);
      if (iopt != _config.end()) {
         pPc->mbGrabDepth = parseBool(iopt->second, /*empty=*/ true);
      }
      iopt = _config.find("--pcgrabrect" + sffx);
      if (iopt != _config.end()) {
         pPc->mbGrabRectImage = parseBool(iopt->second, /*empty=*/ true);
      }

      pPc->configurePcComm(conf);
      m_pointcloud.push_back(pPc);
   }
#endif

   if((it = _config.find("--fakegrabbing")) != _config.end()) {
      m_fakeRecording = parseBool(it->second, /*empty=*/ true);
   }

   if((it = _config.find("--grab_ms")) != _config.end()) {
      istringstream str(it->second);
      str >> m_frameGrabMs;
   }
   if (m_frameGrabMs < 10) m_frameGrabMs = 10;

#ifdef FEAT_VISUALIZATION
   m_display.configureDisplayClient(_config);
#endif

   if(m_video.size() < 1)
      throw runtime_error(exceptionMessage(__HERE__, "No video server name given"));
}

void CVideoGrabber::start()
{
#ifdef FEAT_VISUALIZATION
   m_display.connectIceClient(*this);
   m_display.installEventReceiver();
   m_display.createForms();

   // TODO: Server Bug: only one (the first) checkbox control is shown (same id)...
   m_display.addCheckBox(IDOBJ_GRABBER, IDCTRL_STREAMING, "Streaming");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_GRAB, "&Grab");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_RECORD, "&Record");
   m_display.addButton(IDOBJ_GRABBER, IDCMD_STOP, "&Stop Recording");
   m_display.addCheckBox(IDOBJ_SETTINGS, IDCTRL_STREAMING, "Streaming");
#endif

   Video::CVideoClient2* pVideo;
#if 0 // XXX: code for testing
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, m_videoServerName, m_camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);

   std::vector<int> camIds;
   camIds.push_back(0);
   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, "VideoServer2", camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);

   pVideo = new Video::CVideoClient2();
   pVideo->setServer(this, "VideoServer3", camIds);
   pVideo->setReceiver(new Video::CReceiverMethod<CVideoGrabber>(this, &CVideoGrabber::receiveImages));
   m_video.push_back(pVideo);
#endif

   for(unsigned int i = 0; i < m_video.size(); i++) {
      pVideo = m_video[i];
      pVideo->setCaching(true);
      pVideo->connect();
      pVideo->setReceiving(true);
   }

#ifdef FEAT_VIDEOGRABBER_POINTCLOUD
   CPcGrabClient *pPc;
   for(unsigned int i = 0; i < m_pointcloud.size(); i++) {
      pPc = m_pointcloud[i];
      pPc->startPcComm(*this);
#ifdef FEAT_VISUALIZATION
      pPc->configExtraV11n(m_display);
#endif
   }
#endif
}

#ifdef FEAT_VISUALIZATION
void CVideoGrabber::CVvDisplayClient::createForms()
{
   //std::string head =
   //   "<style>"
   //   ".info { font-size: 90%; color: #808080; }"
   //   "</style>"
   //   ;
   //setHtmlHead(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, head);
   std::ostringstream ss;
   string help1, help2;

   ss <<
      "<table>"
      "<tr><td>Model: </td><td>"
      "<input type='text' name='model' style='width:20em;' />"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("model"));

   help1 = "Use \"%m\" to place the model name in the Directory.";
   help2 = "\"Create directory\" can only create one (the last) level.";
   ss <<
      "<table>"
      "<tr><td>Directory: </td><td>"
      "<input type='text' name='directory' style='width:20em;' title='" << help1 << "'/>"
      "</td></tr><tr><td>&nbsp;</td><td>"
      "<input type='checkbox' name='createdir' value='on' title='" << help2 << "'/>Create directory"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("directory"));
   m_frmSettings.add(new cxd::CFormValues::set("createdir", cxd::CFormValues::valuelist() << "on"));

   help1 = 
      "Filename to save to.\n"
      "Use \"%c\" to place the counter.\n"
      "Use \"%d\" to place the device name.\n"
      "Use \"%m\" to place the model name.\n";
   ss <<
      "<table>"
      "<tr><td>Filename: </td><td>"
      "<input type='text' name='filename' style='width:20em;' title='" << help1 << "'/>"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("filename"));

   ss << "<table>";
   vector<string> cntValues;
   ss << "<tr><td>Counter digits: </td><td>"
      "<select name='countersize' >";
   for(int i = 1; i < 10; i++) {
      ss << "<option>" << i << "</option>";
      cntValues.push_back(_str_(i));
   }
   ss << "</select>";

   ss <<
      "</td><td>Next value: <td>"
      "<input type='text' name='countervalue' style='width:5em;' />"
      "</td></tr>";
   m_frmSettings.add(new cxd::CFormValues::field("countervalue"));
   m_frmSettings.add(new cxd::CFormValues::choice("countersize", cxd::CFormValues::valuelist(cntValues)));
   ss << "</table>";

   help1 = "Space delimited list of device names (for paramerer \"%d\").";
   ss <<
      "<table>"
      "<tr><td>Device names: </td><td>"
      "<input type='text' name='devicenames' style='width:10em;' title='" << help1 << "' />"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("devicenames"));

   help1 = "Stop recording after limited time. Enter 0 or less for no limit.";
   ss <<
      "<table>"
      "<tr><td>Recording limit: </td><td>"
      "<input type='text' name='recordtimelimit' style='width:10em;' title='" << help1 << "' /> milliseconds"
      "</td></tr>"
      "</table>";
   m_frmSettings.add(new cxd::CFormValues::field("recordtimelimit"));

   ss << "<input type='submit' value='Apply' />";

   ss << 
      "<div class='v11ninfo'>"
      "If you change anything, don't forget to 'Apply'.<br>"
      "You can 'Save' current values for later use.<br>"
      "After you 'Load' the saved values, press 'Apply'.<br>"
      "More help is available in the tooltips."
      "</div>";

#if 0 // this makes DisplayServer crash because of mozilla plugins
   ss << R"(
      </td><td>
      <object type='application/cast-displayview' data='cogxdisp://view/Video.Grabber'
      width='200' height='400'>
      </object>
      </td></tr>
      )";
#endif

   // Set form defaults
   setDirectory("xdata/grab/%m");
   setImageFilenamePatt("frame-%c-%d");
   setDeviceNames("L R P D");
   setCounterDigits(3);
   setCounterValue(0);
   setRecordTimeLimit(5000);
   
   setHtmlForm(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, ss.str());
   showCurrentSettings();
}

void CVideoGrabber::CVvDisplayClient::showCurrentSettings()
{
   std::ostringstream ss;
   ss << "<hr>"
      << "<table><tr><td>";

   ss << "<table>"
      << "<tr><td>"
      << "<input type='button' value='Grab' @@ONCLICK@@('" << IDCMD_GRAB << "') />\n"
      << "</td></tr><tr><td>"
      << "<input type='button' value='Record' @@ONCLICK@@('" << IDCMD_RECORD << "') />\n"
      << "</td></tr><tr><td>"
      << "<input type='button' value='Stop Recording' @@ONCLICK@@('" << IDCMD_STOP << "') />\n"
      << "</td></tr>"
      << "</table>";

   ss << "</td><td>";

   ss <<
      "Current settings:"
      "<div>"
      "Saving to: " << getDirectory() << "/" << getImageFilenamePatt() << "<br>"
      "Next counter value: " << getCounterValue() << "<br>"
      "Time limit: " << getRecordTimeLimit() << "ms<br>"
      "Grabbing speed: " << pViewer->m_frameGrabMs << "ms, "
      << _str_(1000.0/pViewer->m_frameGrabMs, 0, 3, ' ') << "fps<br>"
      "</div>";

   ss << "</td></tr></table>";

   setActiveHtml(IDOBJ_SETTINGS, IDPART_SETTINGS_VALUES, ss.str());
}

std::string CVideoGrabber::CVvDisplayClient::getDirectory()
{
   std::string dir = _s_::strip(m_frmSettings.get("directory"));
   std::string model = _s_::strip(m_frmSettings.get("model"));
   _s_::replace(dir, "%m", model);
   return _s_::rstrip(dir, "/ ");
}

std::string CVideoGrabber::CVvDisplayClient::getModelName()
{
   std::string model = _s_::strip(m_frmSettings.get("model"));
   return model;
}

void CVideoGrabber::CVvDisplayClient::setDirectory(const std::string& dirname)
{
   m_frmSettings.setValue("directory", dirname);
}

bool CVideoGrabber::CVvDisplayClient::getCreateDirectory()
{
   return _s_::strip(m_frmSettings.get("createdir")) == "on";
}

std::string CVideoGrabber::CVvDisplayClient::getImageFilenamePatt()
{
   return _s_::strip(m_frmSettings.get("filename"));
}

void CVideoGrabber::CVvDisplayClient::setImageFilenamePatt(const std::string& pattern)
{
   m_frmSettings.setValue("filename", pattern);
}

std::string CVideoGrabber::CVvDisplayClient::getDeviceNames()
{
   return _s_::strip(m_frmSettings.get("devicenames"));
}

void CVideoGrabber::CVvDisplayClient::setDeviceNames(const std::string& names)
{
   m_frmSettings.setValue("devicenames", names);
}

long CVideoGrabber::CVvDisplayClient::getCounterDigits()
{
   // TODO: countersize is a set so getInt doesn't work ... 
   return atoi(m_frmSettings.get("countersize").c_str());
}

void CVideoGrabber::CVvDisplayClient::setCounterDigits(long nDigits)
{
   m_frmSettings.setValue("countersize", _str_(nDigits));
}

long CVideoGrabber::CVvDisplayClient::getCounterValue()
{
   return m_frmSettings.getInt("countervalue");
}

void CVideoGrabber::CVvDisplayClient::setCounterValue(long nValue)
{
   m_frmSettings.setValue("countervalue", _str_(nValue));
}

long CVideoGrabber::CVvDisplayClient::getRecordTimeLimit()
{
   return m_frmSettings.getInt("recordtimelimit");
}

void CVideoGrabber::CVvDisplayClient::setRecordTimeLimit(long nValue)
{
   m_frmSettings.setValue("recordtimelimit", _str_(nValue));
}

void CVideoGrabber::CVvDisplayClient::handleEvent(const Visualization::TEvent &event)
{
   //debug(event.data + " (received by VideoViewer)");
   if (event.type == Visualization::evCheckBoxChange) {
      if (event.sourceId == IDCTRL_STREAMING) {
         //debug(std::string("Time: ") + _str_(fclocks()));
         bool newrcv = (event.data != "0");
         if (newrcv != pViewer->m_bReceiving) {
            if(pViewer->m_bReceiving) {
               //pViewer->m_video.setReceiving(false);
               pViewer->println("Stopped receiving images");
            }
            else {
               //pViewer->m_video.setReceiving(true);
               pViewer->println("Started receiving images");
            }
            pViewer->m_bReceiving = !pViewer->m_bReceiving;
         }
      }
   }
   else if (event.type == Visualization::evButtonClick) {
      if (event.sourceId == IDCMD_GRAB) {
         pViewer->startGrabbing(IDCMD_GRAB);
      }
      else if (event.sourceId == IDCMD_RECORD) {
         pViewer->startGrabbing(IDCMD_RECORD);
      }
      else if (event.sourceId == IDCMD_STOP) {
         pViewer->stopGrabbing();
      }
   }
   else if (event.type == Visualization::evHtmlOnClick) {
      if (event.sourceId == IDCMD_GRAB) {
         pViewer->startGrabbing(IDCMD_GRAB);
      }
      else if (event.sourceId == IDCMD_RECORD) {
         pViewer->startGrabbing(IDCMD_RECORD);
      }
      else if (event.sourceId == IDCMD_STOP) {
         pViewer->stopGrabbing();
      }
   }
}

void CVideoGrabber::CVvDisplayClient::updateDisplay()
{
   std::map<std::string, std::string> fields;
   m_frmSettings.get(fields);
   setHtmlFormData(IDOBJ_SETTINGS, IDPART_SETTINGS_FORM, fields);
   showCurrentSettings();
}

std::string CVideoGrabber::CVvDisplayClient::getControlState(const std::string& ctrlId)
{
   if (ctrlId == IDCTRL_STREAMING) {
      return pViewer->m_bReceiving ? "1" : "0";
   }
   return "";
}

void CVideoGrabber::CVvDisplayClient::handleForm(const std::string& id,
      const std::string& partId, const std::map<std::string, std::string>& fields)
{
   std::cout << " *** handleForm " << partId << std::endl;
   if (partId == IDPART_SETTINGS_FORM) {
      m_frmSettings.apply(fields);
      showCurrentSettings();
   }
}

bool CVideoGrabber::CVvDisplayClient::getFormData(const std::string& id,
      const std::string& partId, std::map<std::string, std::string>& fields)
{
   if (partId == IDPART_SETTINGS_FORM) {
      m_frmSettings.get(fields);
      return true;
   }
   return false;
}

void CVideoGrabber::releaseCanvas()
{
   if (m_pDisplayCanvas) {
      // HACK: Share data between IplImage and vector
      m_pDisplayCanvas->imageData = NULL;
      m_pDisplayCanvas->imageDataOrigin = NULL;
      cvReleaseImage(&m_pDisplayCanvas);
   }
}

void CVideoGrabber::prepareCanvas(int width, int height)
{
   if (m_pDisplayCanvas == NULL
         || m_pDisplayCanvas->width != width || m_pDisplayCanvas->height != height)
   {
      releaseCanvas();
      // HACK: Share data between IplImage and vector
      m_pDisplayCanvas = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
      m_DisplayBuffer.resize(width * height * 3);
      m_pDisplayCanvas->imageData = (char*) &(m_DisplayBuffer[0]);
      m_pDisplayCanvas->imageDataOrigin = m_pDisplayCanvas->imageData;
      m_pDisplayCanvas->widthStep = width * 3;
      m_pDisplayCanvas->imageSize = m_pDisplayCanvas->widthStep * height;
   }
}

#endif

void CVideoGrabber::destroy()
{
}

void CVideoGrabber::sendCachedImages()
{
#ifdef FEAT_VISUALIZATION
   std::vector<CPreview> previews;
   std::vector<CDataSource*> clients;
   getClients(clients);
   for (auto pClient : clients){
      pClient->getPreviews(previews, 320, 240, isGrabbing());
   }
   int w = 0, h = 0;
   for (auto prv : previews) {
      IplImage* pImg = prv.getImage();
      if (w < pImg->width) {
         w = pImg->width;
      }
      h += pImg->height;
   }

   prepareCanvas(w, h);
   IplImage *pDisp = m_pDisplayCanvas;
   CvFont fntSimplex, fntPlain;
   cvInitFont(&fntSimplex, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.8, 0, 1.6);
   cvInitFont(&fntPlain, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1.5);
   std::vector<std::string> devnames = getDeviceNames();

   int vp = 0;
   int i = 0;
   for (auto prv : previews) {
      int factor = 1;
      int wi = (int) (prv.getImage()->width * factor);
      int hi = (int) (prv.getImage()->height * factor);
      cvSetImageROI(pDisp, cvRect(0, vp, wi, vp+hi));
      cvResize(prv.getImage(), pDisp);

      std::string sMsg = _str_(i) + " " + prv.deviceName;
      if (i < devnames.size()) sMsg += ": " + devnames[i];
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, 25), &fntSimplex, cvScalar(255,255,0));
      
      sMsg = prv.deviceInfo;
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, hi-2), &fntPlain, cvScalar(255,255,0));

      cvResetImageROI(pDisp);
      vp += hi;
      ++i;
   }
   m_display.setImage(IDOBJ_GRABBER, w, h, 3, m_DisplayBuffer);

   for (auto pClient : clients){
      pClient->displayExtra(m_display);
   }
#endif

#if 0
#ifdef FEAT_VISUALIZATION
   std::vector<Video::CCachedImagePtr> images;
   for (unsigned int i = 0; i < m_video.size(); i++) {
      Video::CVideoClient2& v = *m_video[i];
      std::vector<Video::CCachedImagePtr> timgs;
      v.getCachedImages(timgs);
      std::vector<Video::CCachedImagePtr>::iterator itt;
      for(itt = timgs.begin(); itt != timgs.end(); itt++) {
         images.push_back(*itt);
      }
   }

   int w = 0, h = 0;
   double factor = 1.0;
   for (unsigned int i = 0; i < images.size(); i++) {
      if (images[i]->width > w) w = images[i]->width;
      h += images[i]->height;
   }
   if (w > 320) {
      factor = 320.0 / w;
      w = (int) (w * factor + 0.5);
      h = (int) (h * factor + 0.5);
   }
   // TODO: every image has its own scale factor

   CvFont fntSimplex, fntPlain;
   cvInitFont(&fntSimplex, CV_FONT_HERSHEY_SIMPLEX, 1.0, 1.0, 0, 2);
   cvInitFont(&fntPlain, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1.5);
   std::vector<std::string> devnames = getDeviceNames();
   int vp = 0;
   prepareCanvas(w, h);
   IplImage *pDisp = m_pDisplayCanvas;
   for (unsigned int i = 0; i < images.size(); i++) {
      IplImage *iplImage = cloneVideoImage(*images[i]);
      int wi = (int) (images[i]->width * factor);
      int hi = (int) (images[i]->height * factor);
      cvSetImageROI(pDisp, cvRect(0, vp, wi, vp+hi));
      cvResize(iplImage, pDisp);
      releaseClonedImage(&iplImage);

      std::string sMsg = _str_(i);
      if (i < devnames.size()) sMsg += ":" + devnames[i];
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, 25), &fntSimplex, cvScalar(255,255,0));
      
      sMsg = _str_(images[i]->width) + "x" + _str_(images[i]->height);
      cvPutText (pDisp, sMsg.c_str(), cvPoint(10, hi-2), &fntPlain, cvScalar(255,255,0));

      cvResetImageROI(pDisp);
      vp += hi;
   }
   m_display.setImage(IDOBJ_GRABBER, w, h, 3, m_DisplayBuffer);
#endif
#endif
}

void CVideoGrabber::receiveImages(const std::string& serverName, const std::vector<Video::Image>& _images)
{
#if 0
#ifdef FEAT_VISUALIZATION
   //sendCachedImages();
#else
   IplImage *iplImage = convertImageToIpl(images[0]);
   cvShowImage(getComponentID().c_str(), iplImage);
   cvReleaseImage(&iplImage);
#endif
#endif
}

std::vector<std::string> CVideoGrabber::getDeviceNames()
{
   std::vector<std::string> devnames = _s_::split(m_display.getDeviceNames(), " ", 0, false);
   // TODO: add missing device names, form m_video
   //while (devnames.size() < m_camIds.size()) {
   //   int i = m_camIds[devnames.size()];
   //   devnames.push_back(_str_(i));
   //}
   return devnames;
}

void CVideoGrabber::fillRecordingInfo(CRecordingInfo &info)
{
   info.directory = m_display.getDirectory();
   info.directoryStatus = 0;
   if (m_display.getCreateDirectory()) {
      info.directoryStatus = 1;
   }
   std::string fname = m_display.getImageFilenamePatt();
   std::string model = m_display.getModelName();
   _s_::replace(fname, "%m", model);
   info.filenamePatt = fname;

   long digits = m_display.getCounterDigits();
   if (digits < 1) digits = 1;
   if (digits > 9) digits = 9;
   info.counterDigits = digits;

   info.counterStart = m_display.getCounterValue();
   info.counterEnd = info.counterStart; // TODO: calculate from settings
   info.counter = info.counterStart;

   long reclimit = m_display.getRecordTimeLimit();
   info.tmStart = IceUtil::Time::now();
   info.tmEnd = info.tmStart + IceUtil::Time::milliSeconds(reclimit);

   info.deviceNames = getDeviceNames();
}

void CVideoGrabber::startGrabbing(const std::string& command)
{
   if (isGrabbing()) return;
   fillRecordingInfo(m_RecordingInfo);
   if (command == IDCMD_GRAB) {
      // On single-shot grabbing don't install the timer, just grab right now
      dynamic_cast<CSaveQueThread*>(m_pQueue.get())->grab();
      m_display.updateDisplay(); // send counter value
   }
   else if (command == IDCMD_RECORD) {
      m_RecordingInfo.recording = true;
      m_pTimer->scheduleRepeated(m_pQueueTick.get(), IceUtil::Time::milliSeconds(m_frameGrabMs));
   }
}

void CVideoGrabber::stopGrabbing()
{
   if (!isGrabbing()) return;
   m_pTimer->cancel(m_pQueueTick.get());
   m_RecordingInfo.recording = false;
   sleepComponent(100);
   m_display.updateDisplay();
}

void CVideoGrabber::checkStopGrabbing()
{
   CRecordingInfo& ri = m_RecordingInfo;
   if (ri.counterEnd > ri.counterStart && ri.counter >= ri.counterEnd) stopGrabbing();
   else if (ri.tmEnd > ri.tmStart && IceUtil::Time::now() >= ri.tmEnd) stopGrabbing();
}

#if 0
// TODO: frameInfo should be const
void CVideoGrabber::saveQueuedImages(const std::vector<Video::CCachedImagePtr>& images,
      CRecordingInfo& frameInfo)
{
   if (frameInfo.directoryStatus == 0 || frameInfo.directoryStatus == 1) {
      struct stat finfo;
      string& dir = frameInfo.directory;
      bool exists = (0 == stat(dir.c_str(), &finfo));
      if (exists) frameInfo.directoryStatus = 2;
      else {
         if (frameInfo.directoryStatus == 0) 
            frameInfo.directoryStatus = -1;
         else {
            mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            exists = (0 == stat(dir.c_str(), &finfo));
            frameInfo.directoryStatus = exists ? 2 : -1;
         }
      }
   }
   //if (frameInfo.directoryStatus < 0) return;

   std::string fname = frameInfo.filenamePatt;
   std::string sval = _str_(frameInfo.counter, frameInfo.counterDigits, '0');
   _s_::replace(fname, "%c", sval);

   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   for (unsigned int i = 0; i < images.size(); i++) {
      std::string fullname = fname;
      std::string devname;
      if (i < frameInfo.deviceNames.size()) devname = frameInfo.deviceNames[i];
      else devname = "d" + _str_(i, 2, '0');
      _s_::replace(fullname, "%d", devname);

      fullname = frameInfo.directory + "/" + fullname;
      println("Saving image: %s", fullname.c_str());
      if (!m_fakeRecording) {
         IplImage *iplImage = cloneVideoImage(*images[i]);
         cvSaveImage(fullname.c_str(), iplImage);
         releaseClonedImage(&iplImage);
      }
   }
}
#else
void CVideoGrabber::saveQueuedImages(const std::vector<CGrabbedItemPtr>& images,
      CRecordingInfo& frameInfo)
{
   if (frameInfo.directoryStatus == 0 || frameInfo.directoryStatus == 1) {
      struct stat finfo;
      string& dir = frameInfo.directory;
      bool exists = (0 == stat(dir.c_str(), &finfo));
      if (exists) frameInfo.directoryStatus = 2;
      else {
         if (frameInfo.directoryStatus == 0) 
            frameInfo.directoryStatus = -1;
         else {
            mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            exists = (0 == stat(dir.c_str(), &finfo));
            frameInfo.directoryStatus = exists ? 2 : -1;
         }
      }
   }

   int devid = 0;
   for (auto pitem : images) {
      pitem->save(frameInfo, devid);
      devid += pitem->numDevices();
   }
}
#endif

void CVideoGrabber::saveImages(const std::vector<Video::Image>& images)
{
   std::string dir = m_display.getDirectory();
   if (m_display.getCreateDirectory()) {
      struct stat finfo;
      int rv = stat(dir.c_str(), &finfo);
      bool exists = rv == 0;
      if (! exists) {
         mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
      }
   }
   std::string fname = m_display.getImageFilenamePatt();
   std::vector<std::string> devnames = getDeviceNames();
   long digits = m_display.getCounterDigits();
   long val = m_display.getCounterValue();
   if (digits < 1) digits = 1;
   if (digits > 9) digits = 9;
   std::string sval = _str_(val, digits, '0');
   _s_::replace(fname, "%c", sval);

   std::string model = m_display.getModelName();
   _s_::replace(fname, "%m", model);
 
   // TODO: conversion to GS when saving;
   // TODO: compression parameters for jpeg and png
   for (unsigned int i = 0; i < images.size(); i++) {
      std::string fullname = fname;
      _s_::replace(fullname, "%d", devnames[i]);
      fullname = dir + "/" + fullname;
      println("Saving image: %s", fullname.c_str());
      IplImage *iplImage = convertImageToIpl(images[i]);
      cvSaveImage(fullname.c_str(), iplImage);
      cvReleaseImage(&iplImage);
   }
}


CVideoGrabber::CSaveQueThread::CSaveQueThread(CVideoGrabber *pGrabber)
{
   m_pGrabber = pGrabber;
}

void CVideoGrabber::CSaveQueThread::getItems(
      std::vector<CVideoGrabber::CSaveQueThread::CFramePack>& items,
      unsigned int maxItems)
{
   IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_itemsLock);
   if (maxItems < 1 || maxItems >= m_items.size()) {
      items = m_items;
      m_items.clear();
      return;
   }
   items.clear();
   items.insert(items.end(), m_items.begin(), m_items.begin() + (maxItems-1));
   m_items.erase(m_items.begin(), m_items.begin() + (maxItems-1));
}

void CVideoGrabber::CSaveQueThread::grab()
{
   if (! m_pGrabber) return;
   //IceUtil::Time tm = IceUtil::Time::now();

   // TODO: To also grab point-clouds (and other types of data), getClients has to be changed!
   // Refactoring:
   //   - clients is a vector<GrabberClient>
   //   - PcClient is a GrabberClient and a PointCloudClient
   //   - VideoClient is a GrabberClient and a CVideoClient2
   //   - a GrabberClient grabs in grab()
   //   - a GrabberClient adds SaveItem(Ptr)s to the queue
   //   - saveQueuedImages works with SaveItem(Ptr)s instead of CCachedImagePtr-s
   CFramePack pack;
   std::vector<CGrabbedItemPtr> data;
   std::vector<CDataSource*> clients;
   m_pGrabber->getClients(clients);
   for (unsigned int i = 0; i < clients.size(); i++) {
      std::vector<CGrabbedItemPtr> timgs;
      clients[i]->grab(timgs);
      for(auto itt = timgs.begin(); itt != timgs.end(); itt++) {
         pack.images.push_back(*itt);
      }
   }
   pack.frameInfo = m_pGrabber->m_RecordingInfo;
   {
      IceUtil::Monitor<IceUtil::Mutex>::Lock lock(m_itemsLock);
      m_items.push_back(pack);
   }

   m_pGrabber->m_RecordingInfo.counter++;
   m_pGrabber->m_display.setCounterValue(m_pGrabber->m_RecordingInfo.counter);

   //IceUtil::Time tm2 = IceUtil::Time::now() - tm;
   //m_pGrabber->debug("images copied in %lld micros", tm2.toMicroSeconds());
   // result: 2+1 images, 40us
}

void CVideoGrabber::CSaveQueThread::run()
{  
   while (m_pGrabber && m_pGrabber->isRunning()) {
      int ticks = waitForTick(500);
      if (!ticks) continue;
      if (!m_pGrabber->isGrabbing()) continue;
      if (ticks > 1) 
         m_pGrabber->println(string(" *** CSaveQueThread: missed ticks: ") + _str_(ticks-1));

      grab();
      m_pGrabber->checkStopGrabbing();
   }
}

CVideoGrabber::CDrawingThread::CDrawingThread(CVideoGrabber *pGrabber)
{
   m_pGrabber = pGrabber;
}

void CVideoGrabber::CDrawingThread::run()
{  
   while (m_pGrabber && m_pGrabber->isRunning()) {
      int ticks = waitForTick(500);
      if (!ticks) continue;

      m_pGrabber->sendCachedImages();
   }
}

void CVideoGrabber::runComponent()
{
   sleepComponent(1000);
   m_pQueue = new CSaveQueThread(this);
   m_pDrawer = new CDrawingThread(this);

   m_pQueueTick = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pQueue.get()));
   m_pDrawTick  = new CTickerTask(dynamic_cast<CTickSyncedTask*>(m_pDrawer.get()));

   IceUtil::ThreadControl tcQueue = m_pQueue->start();
   IceUtil::ThreadControl tcDrawer = m_pDrawer->start();

   // Unfortunately Timer isn't exact -- (arbitrary) time spent by runTimerTask
   // is added to the time of a period; because of this runTimerTask must be
   // extremely fast so instead of copying image data we implement a locking
   // image cache and copy only the pointers to locked images; this should take
   // under 1 microsecond per period. Synchronization/multithreading may make
   // this time much longer.
   // XXX With CTickSyncedTask the grabber lost 10ms in 10s. Find a more precise implementation of Timer.
   m_pTimer = new IceUtil::Timer();
   m_pTimer->scheduleRepeated(m_pDrawTick.get(), IceUtil::Time::milliSeconds(200));

   while(isRunning()) {
      vector<CSaveQueThread::CFramePack> toSave;
      dynamic_cast<CSaveQueThread*>(m_pQueue.get())->getItems(toSave, 4);
      if (toSave.size() < 1) {
         sleepComponent(200);
         continue;
      }
      debug("Will save %d frames", toSave.size());
      vector<CSaveQueThread::CFramePack>::iterator it;
      for (it = toSave.begin(); it != toSave.end(); it++) {
         IceUtil::Time tm = IceUtil::Time::now();
         saveQueuedImages(it->images, it->frameInfo);
         IceUtil::Time tm2 = IceUtil::Time::now() - tm;
         debug("%d images saved in %lld ms", it->images.size(), tm2.toMilliSeconds());
      }
      toSave.clear();
   }

   debug("joining worker threads");
   tcQueue.join();
   tcDrawer.join();

   m_pTimer->destroy();
}

} // namespace
/* vim: set sw=3 ts=8 et: */
